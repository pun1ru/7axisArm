"""关节限位内的工作空间采样与体素抽稀。

工作空间本身是六维位姿集合，三维图里能直观显示的是**末端位置可达集**：
在关节限位内随机采样 q，做正运动学，只保留末端位置。七轴构型用规则网格会导致
维度爆炸，因此这里使用固定种子的均匀随机采样；结果可复现，也足够看包络形状。

采样点先按体素合并，再交给 View3D 画点云。这样 10 万级样本最终通常只剩
1~2 万个点，拖拽视角时 QPainter 的负担可控。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .dh import DHTable
from .ik import branch_indices_deg


@dataclass(frozen=True)
class WorkspaceCloud:
    """抽稀后的末端位置点云。density 为归一化占用密度，范围 0..1。"""

    points_m: np.ndarray
    density: np.ndarray
    samples: int
    voxel_m: float
    limits_deg: tuple[tuple[float, float], ...]
    elapsed_ms: float
    branch: int | None = None

    @property
    def bounds_m(self) -> tuple[np.ndarray, np.ndarray]:
        if len(self.points_m) == 0:
            return np.zeros(3), np.zeros(3)
        return self.points_m.min(axis=0), self.points_m.max(axis=0)


def forward_positions_deg(table: DHTable, q_deg: np.ndarray) -> np.ndarray:
    """批量正运动学，只返回末端位置；q_deg 形状为 (N, n)。"""

    q = np.asarray(q_deg, dtype=float)
    if q.ndim != 2 or q.shape[1] != table.n:
        raise ValueError(f"关节角矩阵应为 (N, {table.n})，实际 {q.shape}")
    if len(q) == 0:
        return np.empty((0, 3))

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
    return p


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
                     branch: int | None = None) -> WorkspaceCloud:
    """在关节限位内采样末端位置，并按体素抽稀；branch=None 表示全部 8 个分支。"""

    if voxel_m <= 0.0:
        raise ValueError("体素尺寸必须大于 0")

    t0 = time.perf_counter()
    q = sample_joint_configurations(table, limits_deg, samples, seed, branch)
    points = forward_positions_deg(table, q)

    # 体素太细时自动放粗一档，保证交给 QPainter 的点数有上限。
    actual_voxel = float(voxel_m)
    for _ in range(4):
        centers, counts = _voxel_centers(points, actual_voxel)
        if len(centers) <= max_points:
            break
        actual_voxel *= max(1.2, (len(centers) / max_points) ** (1.0 / 3.0))

    density = np.log1p(counts)
    density /= density.max()
    return WorkspaceCloud(
        points_m=centers,
        density=density.astype(np.float32),
        samples=len(q),
        voxel_m=actual_voxel,
        limits_deg=tuple((float(lo), float(hi)) for lo, hi in limits_deg),
        elapsed_ms=(time.perf_counter() - t0) * 1e3,
        branch=None if branch is None else int(branch),
    )


def _voxel_centers(points: np.ndarray, voxel_m: float) -> tuple[np.ndarray, np.ndarray]:
    lo = points.min(axis=0)
    idx = np.floor((points - lo) / voxel_m).astype(np.int64)
    dims = idx.max(axis=0) + 1
    keys = (idx[:, 0] * dims[1] + idx[:, 1]) * dims[2] + idx[:, 2]
    keys, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
    centers = np.empty((len(keys), 3))
    for axis in range(3):
        centers[:, axis] = np.bincount(inverse, weights=points[:, axis], minlength=len(keys))
    centers /= counts[:, None]
    return centers, counts
