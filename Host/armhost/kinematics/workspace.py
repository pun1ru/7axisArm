"""关节限位内的 TCP 工作空间采样与体素抽稀。

这里区分两个概念：

* 可达工作区：TCP 至少有一个关节构型能到达的位置；
* 灵活工作区：同一体素中的 TCP 接近方向（`z_tcp`）覆盖至少 N 个方向扇区，
  是可达工作区的子集。它不是“所有姿态都可达”的严格灵巧工作区，
  而是在离散方向覆盖意义上可交互比较的灵活区。

七轴构型用固定种子的均匀随机采样，避免规则网格维度爆炸。点云先按体素合并，
再交给 View3D 绘制，限制 QPainter 的负担。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .dh import DHTable
from .ik import branch_indices_deg

APPROACH_SECTOR_COUNT = 26
_APPROACH_DIRECTIONS = np.asarray(
    [(x, y, z) for x in (-1.0, 0.0, 1.0) for y in (-1.0, 0.0, 1.0) for z in (-1.0, 0.0, 1.0)
     if (x, y, z) != (0.0, 0.0, 0.0)],
    dtype=float,
)
_APPROACH_DIRECTIONS /= np.linalg.norm(_APPROACH_DIRECTIONS, axis=1)[:, None]


@dataclass(frozen=True)
class WorkspaceCloud:
    """抽稀后的 TCP 工作区点云。density 为归一化占用密度，范围 0..1。"""

    points_m: np.ndarray
    density: np.ndarray
    flexible_points_m: np.ndarray
    flexible_density: np.ndarray
    samples: int
    voxel_m: float
    limits_deg: tuple[tuple[float, float], ...]
    elapsed_ms: float
    branch: int | None = None
    flexible_min_sectors: int = 3

    @property
    def bounds_m(self) -> tuple[np.ndarray, np.ndarray]:
        if len(self.points_m) == 0:
            return np.zeros(3), np.zeros(3)
        return self.points_m.min(axis=0), self.points_m.max(axis=0)


def forward_poses_deg(table: DHTable, q_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """批量正运动学，返回 TCP 姿态旋转矩阵 (N,3,3) 和位置 (N,3)。"""

    q = np.asarray(q_deg, dtype=float)
    if q.ndim != 2 or q.shape[1] != table.n:
        raise ValueError(f"关节角矩阵应为 (N, {table.n})，实际 {q.shape}")
    if len(q) == 0:
        return np.empty((0, 3, 3)), np.empty((0, 3))

    R = np.broadcast_to(np.eye(3), (len(q), 3, 3)).copy()
    p = np.zeros((len(q), 3))
    for i, row in enumerate(table.rows):
        th = np.radians(q[:, i] + row.theta_offset_deg)
        al = np.radians(row.alpha_deg)
        ct, st = np.cos(th), np.sin(th)
        ca, sa = np.cos(al), np.sin(al)

        Ri = np.empty((len(q), 3, 3))
        Ri[:, 0, 0], Ri[:, 0, 1], Ri[:, 0, 2] = ct, -st, 0.0
        Ri[:, 1, 0], Ri[:, 1, 1], Ri[:, 1, 2] = st * ca, ct * ca, -sa
        Ri[:, 2, 0], Ri[:, 2, 1], Ri[:, 2, 2] = st * sa, ct * sa, ca
        ti = np.empty((len(q), 3))
        ti[:, 0] = row.a_m
        ti[:, 1] = -sa * row.d_m
        ti[:, 2] = ca * row.d_m

        p = p + np.einsum("nij,nj->ni", R, ti)
        R = np.einsum("nij,njk->nik", R, Ri)

    tool = table.tool_transform
    p = p + np.einsum("nij,j->ni", R, tool[:3, 3])
    R = np.einsum("nij,jk->nik", R, tool[:3, :3])
    return R, p


def forward_positions_deg(table: DHTable, q_deg: np.ndarray) -> np.ndarray:
    """批量正运动学，只返回 TCP 位置；q_deg 形状为 (N, n)。"""
    return forward_poses_deg(table, q_deg)[1]


def sample_joint_configurations(table: DHTable, limits_deg: tuple[tuple[float, float], ...],
                                samples: int = 100_000, seed: int = 20261107,
                                branch: int | None = None) -> np.ndarray:
    """采样关节构型；指定 branch 时只在对应 λ2/λ4/λ6 符号区间内取样。"""

    if len(limits_deg) != table.n:
        raise ValueError(f"需要 {table.n} 组关节限位，实际 {len(limits_deg)}")
    if samples < 1:
        raise ValueError("采样数必须大于 0")
    if branch is not None and not 0 <= int(branch) < 8:
        raise ValueError(f"分支编号应为 0..7，实际 {branch}")

    limits = tuple((float(lo), float(hi)) for lo, hi in limits_deg)
    for i, (lo, hi) in enumerate(limits):
        if not np.isfinite(lo) or not np.isfinite(hi) or lo >= hi:
            raise ValueError(f"J{i} 限位无效：{lo:g} .. {hi:g}°")

    rng = np.random.default_rng(seed)
    lo = np.array([v[0] for v in limits])
    span = np.array([v[1] - v[0] for v in limits])
    if branch is None:
        return lo + rng.random((samples, table.n)) * span

    chosen = int(branch)
    parts: list[np.ndarray] = []
    accepted = 0
    attempts = 0
    max_attempts = samples * 32
    chunk = max(4096, min(50_000, samples * 2))
    while accepted < samples and attempts < max_attempts:
        n = min(chunk, max_attempts - attempts)
        q = lo + rng.random((n, table.n)) * span
        keep = q[branch_indices_deg(table, q) == chosen]
        if len(keep):
            parts.append(keep)
            accepted += len(keep)
        attempts += n
    if not parts:
        raise ValueError(f"当前关节限位内没有分支 #{chosen} 的构型")
    return np.concatenate(parts, axis=0)[:samples]


def sample_workspace(table: DHTable, limits_deg: tuple[tuple[float, float], ...],
                     samples: int = 100_000, voxel_m: float = 0.015,
                     seed: int = 20261107, max_points: int = 18_000,
                     branch: int | None = None,
                     flexible_min_sectors: int = 3) -> WorkspaceCloud:
    """采样 TCP 可达/灵活工作区；branch=None 表示全部 8 个分支。

    ``flexible_min_sectors`` 是同一体素要求的接近方向扇区数。TCP 的 z 轴
    在 26 个最近方向中离散化；取 1 时灵活区与可达区相同。
    """

    if voxel_m <= 0.0:
        raise ValueError("体素尺寸必须大于 0")
    if not 1 <= int(flexible_min_sectors) <= APPROACH_SECTOR_COUNT:
        raise ValueError(f"灵活方向扇区阈值应为 1..{APPROACH_SECTOR_COUNT}")

    t0 = time.perf_counter()
    q = sample_joint_configurations(table, limits_deg, samples, seed, branch)
    rotations, points = forward_poses_deg(table, q)

    # 体素太细时自动放粗一档，保证交给 QPainter 的点数有上限。
    actual_voxel = float(voxel_m)
    for _ in range(4):
        centers, counts, _, _, _ = _voxelize(points, actual_voxel)
        if len(centers) <= max_points:
            break
        actual_voxel *= max(1.2, (len(centers) / max_points) ** (1.0 / 3.0))

    centers, counts, keys, inverse, _ = _voxelize(points, actual_voxel)
    density = np.log1p(counts)
    density /= density.max()

    approach = rotations[:, :, 2]
    sectors = np.argmax(approach @ _APPROACH_DIRECTIONS.T, axis=1)
    sample_keys = keys[inverse]
    pairs = np.unique(sample_keys.astype(np.int64) * APPROACH_SECTOR_COUNT + sectors)
    covered_keys = pairs // APPROACH_SECTOR_COUNT
    center_keys = np.unique(keys)
    coverage = np.bincount(np.searchsorted(center_keys, covered_keys), minlength=len(center_keys))
    flexible = coverage >= int(flexible_min_sectors)

    return WorkspaceCloud(
        points_m=centers,
        density=density.astype(np.float32),
        flexible_points_m=centers[flexible],
        flexible_density=density[flexible].astype(np.float32),
        samples=len(q),
        voxel_m=actual_voxel,
        limits_deg=tuple((float(lo), float(hi)) for lo, hi in limits_deg),
        elapsed_ms=(time.perf_counter() - t0) * 1e3,
        branch=None if branch is None else int(branch),
        flexible_min_sectors=int(flexible_min_sectors),
    )


def _voxelize(points: np.ndarray, voxel_m: float):
    lo = points.min(axis=0)
    idx = np.floor((points - lo) / voxel_m).astype(np.int64)
    dims = idx.max(axis=0) + 1
    keys = (idx[:, 0] * dims[1] + idx[:, 1]) * dims[2] + idx[:, 2]
    unique_keys, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
    centers = np.empty((len(unique_keys), 3))
    for axis in range(3):
        centers[:, axis] = np.bincount(inverse, weights=points[:, axis], minlength=len(unique_keys))
    centers /= counts[:, None]
    return centers, counts, unique_keys, inverse, lo


def _voxel_centers(points: np.ndarray, voxel_m: float) -> tuple[np.ndarray, np.ndarray]:
    """兼容旧调用：只返回体素中心和占用计数。"""
    centers, counts, _, _, _ = _voxelize(points, voxel_m)
    return centers, counts
