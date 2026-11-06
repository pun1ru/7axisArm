"""S-R-S 七轴臂解析逆解与臂角可行域。

本模块逐项镜像 ``Simulation/ArmSim/Algorithm/ik.cpp``，只用于 Host 的交互预览：
实际闭环目标仍由 ArmSim 的 ``SolveIk`` 求解并做最终校验。两边的 DH 和关节限位
都从设备参数读取，避免使用 Host 内置的旧固件参数。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from .dh import DHTable

_DEG = math.pi / 180.0
_SINGULAR_EPS = 1e-6
_REACH_EPS = 1e-5
_MIN_LEN = 1e-9
_REF_EPS = 1e-9
_ARM_ANGLE_MIN_SIN = 0.02
_NON_SINGULAR_MIN_SIN = 0.02   # |sinθ2| / |sinθ6| 低于约 1.15° 视为近奇异


class IkStatus(Enum):
    OK = "ok"
    OUT_OF_REACH = "out_of_reach"
    REF_DEGENERATE = "ref_degenerate"


@dataclass(frozen=True)
class SrsParam:
    shoulder_m: tuple[float, float, float]
    upper_arm_m: float
    elbow_offset_m: float
    forearm_m: float
    tool_m: float
    theta_offset_rad: tuple[float, ...]


@dataclass(frozen=True)
class IkResult:
    status: IkStatus
    q_deg: tuple[float, ...] = ()
    shoulder_singular: bool = False
    wrist_singular: bool = False


def transform_from_xyz_rpy(x: float, y: float, z: float,
                           roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """与固件 TransformFromXyzRpy 相同：R = Rz(yaw)·Ry(pitch)·Rx(roll)。"""
    rr, pp, yy = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(rr), math.sin(rr)
    cp, sp = math.cos(pp), math.sin(pp)
    cy, sy = math.cos(yy), math.sin(yy)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
                     [-sp, cp * sr, cp * cr, z],
                     [0.0, 0.0, 0.0, 1.0]])


def _near(value: float, target: float, eps: float = 1e-3) -> bool:
    return abs(value - target) < eps


def _is_srs_structure(table: DHTable) -> bool:
    r = table.rows
    if len(r) != 7:
        return False
    shoulder = (_near(r[1].a_m, 0.0) and _near(r[1].d_m, 0.0) and _near(r[2].a_m, 0.0)
                and _near(abs(r[1].alpha_deg), 90.0) and _near(abs(r[2].alpha_deg), 90.0))
    elbow = (_near(r[3].a_m, 0.0) and _near(r[4].a_m, 0.0)
             and _near(abs(r[3].alpha_deg), 90.0) and _near(abs(r[4].alpha_deg), 90.0))
    wrist = (_near(r[5].a_m, 0.0) and _near(r[5].d_m, 0.0) and _near(r[6].a_m, 0.0)
             and _near(abs(r[5].alpha_deg), 90.0) and _near(abs(r[6].alpha_deg), 90.0))
    return shoulder and elbow and wrist


def make_srs_param(table: DHTable) -> SrsParam | None:
    if not _is_srs_structure(table) or not _near(table.rows[0].alpha_deg, 0.0):
        return None
    if any(not _near(table.rows[k].alpha_deg, 90.0) for k in range(1, 7)):
        return None
    if not table.rows[2].d_m > 0.0 or not table.rows[4].d_m > 0.0:
        return None
    r0 = table.rows[0]
    return SrsParam(
        shoulder_m=(r0.a_m, 0.0, r0.d_m),
        upper_arm_m=table.rows[2].d_m,
        elbow_offset_m=table.rows[3].d_m,
        forearm_m=table.rows[4].d_m,
        tool_m=table.rows[6].d_m,
        theta_offset_rad=tuple(row.theta_offset_deg * _DEG for row in table.rows),
    )


def _wrap_rad(value: float) -> float:
    return math.remainder(value, 2.0 * math.pi)


def _wrap_deg(value: float) -> float:
    return math.remainder(value, 360.0)


def _rx90() -> np.ndarray:
    return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


def _rotz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = (float(v) for v in axis)
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return np.array([[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
                     [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
                     [z * x * v - y * s, z * y * v + x * s, c + z * z * v]])


def _decompose_zxzxz(rot: np.ndarray, lam: int) -> tuple[float, float, float, bool]:
    cb = -float(rot[2, 2])
    sb = math.hypot(float(rot[0, 2]), float(rot[1, 2]))
    if sb < _SINGULAR_EPS:
        b = 0.0 if cb > 0.0 else math.pi
        return 0.0, b, math.atan2(-float(rot[1, 0]), cb * float(rot[0, 0])), True
    sign = -1.0 if lam < 0 else 1.0
    return (math.atan2(sign * float(rot[1, 2]), sign * float(rot[0, 2])),
            math.atan2(sign * sb, cb),
            math.atan2(-sign * float(rot[2, 1]), sign * float(rot[2, 0])), False)


def _reference_elbow_axis(param: SrsParam, p: np.ndarray) -> np.ndarray | None:
    length = float(np.linalg.norm(p))
    if length < _MIN_LEN:
        return None
    unit = p / length
    sin_gamma = math.hypot(float(unit[0]), float(unit[1]))
    if sin_gamma < _ARM_ANGLE_MIN_SIN:
        return None
    cos_phi = -(param.elbow_offset_m / length) / sin_gamma
    if abs(cos_phi) > 1.0 - _REF_EPS:
        return None
    sin_phi = math.sqrt(max(0.0, 1.0 - cos_phi * cos_phi))
    h = np.array([unit[0] / sin_gamma, unit[1] / sin_gamma, 0.0])
    n = np.array([-h[1], h[0], 0.0])
    return h * cos_phi + n * sin_phi


def _reference_r03(param: SrsParam, p: np.ndarray, y3_ref: np.ndarray,
                   theta4: float) -> np.ndarray | None:
    q = p + y3_ref * param.elbow_offset_m
    length = float(np.linalg.norm(q))
    if length < _MIN_LEN:
        return None
    q_hat = q / length
    delta = math.atan2(param.forearm_m * math.sin(theta4),
                       param.upper_arm_m - param.forearm_m * math.cos(theta4))
    z3 = _rodrigues(y3_ref, -delta) @ q_hat
    x3 = np.cross(y3_ref, z3)
    return np.column_stack((x3, y3_ref, z3))


def solve_ik(param: SrsParam, target: np.ndarray, branch: int, psi_deg: float) -> IkResult:
    """给定目标位姿、分支编号 0..7 和臂角 ψ，返回单圈关节角。"""
    if not 0 <= branch < 8 or not math.isfinite(psi_deg):
        return IkResult(IkStatus.OUT_OF_REACH)

    rot = np.asarray(target, dtype=float)[:3, :3]
    origin = np.asarray(target, dtype=float)[:3, 3]
    wrist = origin - rot[:, 2] * param.tool_m
    p = wrist - np.asarray(param.shoulder_m, dtype=float)
    length = float(np.linalg.norm(p))
    d3, d4, d5 = param.upper_arm_m, param.elbow_offset_m, param.forearm_m
    cos_elbow = (d3 * d3 + d5 * d5 + d4 * d4 - length * length) / (2.0 * d3 * d5)
    if not abs(cos_elbow) <= 1.0 + _REACH_EPS:
        return IkResult(IkStatus.OUT_OF_REACH)
    theta4 = math.acos(max(-1.0, min(1.0, cos_elbow)))
    if branch & 0x02:
        theta4 = -theta4

    y_ref = _reference_elbow_axis(param, p)
    if y_ref is None:
        return IkResult(IkStatus.REF_DEGENERATE)
    r03_ref = _reference_r03(param, p, y_ref, theta4)
    if r03_ref is None:
        return IkResult(IkStatus.REF_DEGENERATE)

    unit = p / length
    r03 = _rodrigues(unit, psi_deg * _DEG) @ r03_ref
    th1, th2, th3, shoulder_singular = _decompose_zxzxz(r03, -1 if branch & 0x01 else 1)

    rx90 = _rx90()
    r04 = r03 @ rx90 @ _rotz(theta4)
    wrist_rot = rx90.T @ r04.T @ rot
    th5, th6, th7, wrist_singular = _decompose_zxzxz(wrist_rot, -1 if branch & 0x04 else 1)

    theta = (th1, th2, th3, theta4, th5, th6, th7)
    q_deg = tuple(math.degrees(_wrap_rad(theta[k] - param.theta_offset_rad[k])) for k in range(7))
    return IkResult(IkStatus.OK, q_deg, shoulder_singular, wrist_singular)


def branch_indices_deg(table: DHTable, q_deg) -> np.ndarray:
    """批量计算分支编号：q_deg 形状 (N, n)，返回 (N,) 的 0..7。

    与固件一致：bit0/bit1/bit2 分别由 sinθ2、sinθ4、sinθ6 是否为负决定。
    """
    q = np.asarray(q_deg, dtype=float)
    if q.ndim == 1:
        q = q[None, :]
    if q.ndim != 2 or q.shape[1] != table.n:
        raise ValueError(f"关节角矩阵应为 (N, {table.n})，实际 {q.shape}")
    out = np.zeros(len(q), dtype=np.int8)
    for bit, k in enumerate((1, 3, 5)):
        theta = np.radians(q[:, k] + table.rows[k].theta_offset_deg)
        out |= (np.sin(theta) < 0.0).astype(np.int8) << bit
    return out


def compute_branch(table: DHTable, q_deg) -> int:
    return int(branch_indices_deg(table, q_deg)[0])


def compute_arm_angle(param: SrsParam, frames: list[np.ndarray]) -> float | None:
    """由 forward_frames 的结果计算臂角 ψ，定义与 solve_ik 一致。"""
    # forward_frames 包含基座 I，因此 C++ 的 frames[0] / [2] / [4] 对应这里的 1 / 3 / 5。
    s = np.asarray(frames[1], dtype=float)[:3, 3]
    w = np.asarray(frames[5], dtype=float)[:3, 3]
    p = w - s
    length = float(np.linalg.norm(p))
    y_ref = _reference_elbow_axis(param, p)
    if length < _MIN_LEN or y_ref is None:
        return None
    unit = p / length
    y_actual = np.asarray(frames[3], dtype=float)[:3, 1]
    a = float(np.dot(y_ref, unit))
    b = float(np.dot(y_actual, unit))
    psi = math.atan2(float(np.dot(unit, np.cross(y_ref, y_actual))),
                     float(np.dot(y_ref, y_actual)) - a * b)
    return math.degrees(psi)


def _joint_reachable(q_deg: float, current_deg: float, lo_deg: float, hi_deg: float) -> bool:
    if not lo_deg < hi_deg:
        return False
    base = current_deg + _wrap_deg(q_deg - _wrap_deg(current_deg))
    return any(lo_deg - 1e-6 <= base + 360.0 * turns <= hi_deg + 1e-6
               for turns in range(-2, 3))


def is_psi_feasible(param: SrsParam, target: np.ndarray, branch: int, psi_deg: float,
                    limits: tuple[tuple[float, float], ...], current_deg) -> bool:
    """ψ 是否同时满足解析逆解、关节限位以及肩/腕非奇异条件。"""
    if len(limits) != 7 or len(current_deg) != 7:
        return False
    solved = solve_ik(param, target, branch, psi_deg)
    if solved.status != IkStatus.OK or solved.shoulder_singular or solved.wrist_singular:
        return False
    for joint in (1, 5):
        theta = solved.q_deg[joint] * _DEG + param.theta_offset_rad[joint]
        if abs(math.sin(theta)) < _NON_SINGULAR_MIN_SIN:
            return False
    return all(_joint_reachable(solved.q_deg[i], float(current_deg[i]), *limits[i])
               for i in range(7))


def feasible_psi_intervals(param: SrsParam, target: np.ndarray, branch: int,
                           limits: tuple[tuple[float, float], ...], current_deg,
                           step_deg: float = 1.0) -> list[tuple[float, float]]:
    """在 [-180, 180] 内计算满足 IK、关节限位和肩/腕非奇异条件的 ψ 区间。

    返回连续区间列表，端点保留采样/二分误差；空列表表示当前位姿和分支无可行 ψ。
    """
    if len(limits) != 7 or len(current_deg) != 7:
        return []
    samples = np.arange(-180.0, 180.0 + step_deg * 0.5, step_deg)
    feasible = [is_psi_feasible(param, target, branch, float(psi), limits, current_deg)
                for psi in samples]

    def refine(ok_psi: float, bad_psi: float) -> float:
        good, bad = ok_psi, bad_psi
        for _ in range(12):
            mid = 0.5 * (good + bad)
            if is_psi_feasible(param, target, branch, mid, limits, current_deg):
                good = mid
            else:
                bad = mid
        return good

    intervals: list[tuple[float, float]] = []
    start: float | None = None
    for i, ok in enumerate(feasible):
        psi = float(samples[i])
        if ok and start is None:
            start = psi if i == 0 else refine(psi, float(samples[i - 1]))
        if not ok and start is not None:
            end = psi if i == 0 else refine(float(samples[i - 1]), psi)
            intervals.append((start, end))
            start = None
    if start is not None:
        intervals.append((start, 180.0))
    return intervals
