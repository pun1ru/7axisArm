"""七轴臂重力参数拟合：tau = Y(q) · pi（静止、只有重力）。命令行与上位机面板共用。

输入：上位机"采集一点"生成的 CSV（logs/gravity/gravity_*.csv），每行一个静止位姿：

    J{i}_angle_deg           关节角 q（deg）
    J{i}_torque_nm           实测力矩（默认拟合它）
    J{i}_torque_cmd_nm       指令力矩（torque_col="cmd" 时拟合它）
    J{i}_velocity_dps        仅 max_vel 过滤运动中的行用
    arm_state                仅 arm_states 按模式过滤用

模型（见 重力补偿推导.md）：
    pi_i = [m_i*cx, m_i*cy, m_i*cz, m_i]，pi 共 4n 维（n=7 -> 28 维）
    tau_g,j = g * sum_{i>=j} sz^T · T_{j-1} · dA_j · A_{j+1}...A_i · pi_i = Y_j(q) · pi

28 个参数里只有约 12 个独立（基参数），数据激励不足时更少，所以用截断 SVD 求最小范数解（rcond）。
报告的是"能复现力矩的参数"，不是唯一的物理质量/质心。bias=True 时同时拟合每个关节的力矩零偏。
"""

from __future__ import annotations

import csv
import glob
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..kinematics import DHTable

G = 9.81


# ============================================================ 回归矩阵
def _link_matrices(row, qi: float) -> tuple[np.ndarray, np.ndarray]:
    al = math.radians(row.alpha_deg)
    th = float(qi) + math.radians(row.theta_offset_deg)
    c, s = math.cos(th), math.sin(th)
    ca, sa = math.cos(al), math.sin(al)
    d = row.d_m
    A = np.array([[c, -s, 0.0, row.a_m],
                  [s * ca, c * ca, -sa, -sa * d],
                  [s * sa, c * sa, ca, ca * d],
                  [0.0, 0.0, 0.0, 1.0]])
    dA = np.array([[-s, -c, 0.0, 0.0],
                   [c * ca, -s * ca, 0.0, 0.0],
                   [c * sa, -s * sa, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0]])
    return A, dA


def gravity_regressor(table: DHTable, q_rad, g: float = G) -> np.ndarray:
    """返回 Y(q)，形状 (n, 4n)；q 单位 rad，关节 i 对应 table 第 i 行。tau_g = Y @ pi。"""
    n = table.n
    A, dA = [], []
    for row, qi in zip(table.rows, q_rad):
        a, da = _link_matrices(row, qi)
        A.append(a)
        dA.append(da)
    T = [np.eye(4)]                       # T[j] = A_0..A_{j-1}（0 起，即文档的 T_{j-1}）
    for Ai in A:
        T.append(T[-1] @ Ai)

    Y = np.zeros((n, 4 * n))
    for j in range(n):
        M = T[j] @ dA[j]
        for i in range(j, n):
            if i > j:
                M = M @ A[i]
            Y[j, 4 * i:4 * i + 4] = g * M[2]      # sz^T · M，取第 3 行
    return Y


def potential_energy(table: DHTable, q_rad, pi, g: float = G) -> float:
    """U = g * sum_i sz^T T_i pi_i，仅用于数值校验 Y。"""
    T = np.eye(4)
    U = 0.0
    for i, (row, qi) in enumerate(zip(table.rows, q_rad)):
        T = T @ _link_matrices(row, qi)[0]
        U += g * (T @ np.asarray(pi[4 * i:4 * i + 4]))[2]
    return U


# ============================================================ 读数据
@dataclass
class Dataset:
    name: str
    q_deg: np.ndarray                     # (N, n)
    tau: np.ndarray                       # (N, n)
    vel_dps: np.ndarray | None = None     # (N, n)
    arm_state: np.ndarray | None = None   # (N,)
    skipped: dict = field(default_factory=dict)


@dataclass
class FitOptions:
    torque_col: str = "torque"            # "torque" 实测 | "cmd" 指令
    sign: float = 1.0                     # CSV 力矩与 tau_g 反号时取 -1
    arm_states: list[int] | None = None
    max_vel: float | None = None          # dps；None 不过滤
    bias: bool = False
    rcond: float = 1e-3
    g: float = G
    cv: bool = True

    def __post_init__(self) -> None:
        if self.torque_col not in ("torque", "cmd"):
            raise ValueError(f"torque_col 只能是 torque / cmd，得到 {self.torque_col!r}")
        if self.sign not in (1.0, -1.0):
            raise ValueError("sign 只能是 +1 / -1")
        if not self.rcond > 0:
            raise ValueError("rcond 必须 > 0")


def _joint_count(cols: list[str]) -> int:
    return max((int(m.group(1)) + 1 for c in cols if (m := re.match(r"J(\d+)_", c))), default=0)


def load_dataset(path: str | Path, torque_col: str = "torque", n_expect: int | None = None) -> Dataset:
    suffix = {"torque": "torque_nm", "cmd": "torque_cmd_nm"}[torque_col]
    with open(path, newline="", encoding="utf-8-sig") as fp:
        rd = csv.DictReader(fp)
        cols = rd.fieldnames or []
        n = _joint_count(cols)
        if n == 0:
            raise ValueError(f"{path}: 没有 J<i>_ 开头的列，不是上位机记录的 CSV")
        if n_expect is not None and n != n_expect:
            raise ValueError(f"{path}: 有 {n} 个关节，DH 表有 {n_expect} 行")
        need = [f"J{j}_angle_deg" for j in range(n)] + [f"J{j}_{suffix}" for j in range(n)]
        missing = [c for c in need if c not in cols]
        if missing:
            raise ValueError(f"{path}: 缺少列 {missing[:4]}")
        has_vel = all(f"J{j}_velocity_dps" in cols for j in range(n))
        q, tau, vel, st = [], [], [], []
        bad = 0
        for r in rd:
            try:
                qi = [float(r[f"J{j}_angle_deg"]) for j in range(n)]
                ti = [float(r[f"J{j}_{suffix}"]) for j in range(n)]
                vi = [float(r[f"J{j}_velocity_dps"]) for j in range(n)] if has_vel else None
                si = int(float(r["arm_state"])) if r.get("arm_state") not in (None, "") else -1
            except (ValueError, TypeError):
                bad += 1
                continue
            if not (np.all(np.isfinite(qi)) and np.all(np.isfinite(ti))):
                bad += 1
                continue
            q.append(qi)
            tau.append(ti)
            st.append(si)
            if vi is not None:
                vel.append(vi)
    if not q:
        raise ValueError(f"{path}: 没有可用的数据行")
    return Dataset(Path(path).name, np.array(q), np.array(tau),
                   np.array(vel) if has_vel else None, np.array(st), {"unparsable": bad})


def filter_dataset(ds: Dataset, arm_states: list[int] | None, max_vel: float | None) -> Dataset:
    keep = np.ones(len(ds.q_deg), dtype=bool)
    skipped = dict(ds.skipped)
    if arm_states:
        m = np.isin(ds.arm_state, arm_states)
        skipped["arm_state"] = int((keep & ~m).sum())
        keep &= m
    if max_vel is not None:
        if ds.vel_dps is None:
            raise ValueError(f"{ds.name}: 没有速度列，不能按速度过滤")
        m = np.all(np.abs(ds.vel_dps) <= max_vel, axis=1)
        skipped["moving"] = int((keep & ~m).sum())
        keep &= m
    return Dataset(ds.name, ds.q_deg[keep], ds.tau[keep],
                   None if ds.vel_dps is None else ds.vel_dps[keep],
                   None if ds.arm_state is None else ds.arm_state[keep], skipped)


def expand_paths(patterns: list[str], default: str | None = None) -> list[str]:
    """通配符展开（Windows 命令行不会替你展开），去重保序。"""
    out: list[str] = []
    for p in patterns or ([default] if default else []):
        out.extend(sorted(glob.glob(p)) if any(ch in p for ch in "*?[") else [p])
    return list(dict.fromkeys(out))


def load_datasets(paths: list[str | Path], table: DHTable, opts: FitOptions) -> tuple[list[Dataset], list[str]]:
    """读入并过滤。返回 (数据集, 警告)；任何文件读不了直接抛 ValueError/OSError。"""
    datasets, warnings = [], []
    for p in paths:
        ds = filter_dataset(load_dataset(p, opts.torque_col, table.n), opts.arm_states, opts.max_vel)
        if len(ds.q_deg) == 0:
            warnings.append(f"{ds.name} 过滤后没有数据，已跳过")
            continue
        datasets.append(ds)
    if not datasets:
        raise ValueError("没有可用数据（文件为空，或全被过滤掉）")
    return datasets, warnings


# ============================================================ 拟合
def build_system(table: DHTable, datasets: list[Dataset], bias: bool, g: float,
                 sign: float = 1.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """堆叠成 (N*n, 4n[+n]) 的 Y 和 (N*n,) 的 tau；同时返回每行属于第几个数据集。"""
    n = table.n
    Ys, ts, owner = [], [], []
    for k, ds in enumerate(datasets):
        for q, t in zip(np.radians(ds.q_deg), ds.tau):
            Y = gravity_regressor(table, q, g)
            if bias:
                Y = np.hstack([Y, np.eye(n)])
            Ys.append(Y)
            ts.append(sign * t)
            owner.append(np.full(n, k))
    return np.vstack(Ys), np.concatenate(ts), np.concatenate(owner)


def solve(Y: np.ndarray, tau: np.ndarray, rcond: float) -> tuple[np.ndarray, int, np.ndarray]:
    x, _, rank, sv = np.linalg.lstsq(Y, tau, rcond=rcond)
    return x, int(rank), sv


def per_joint_stats(Y, tau, x, n: int) -> list[dict]:
    res = (tau - Y @ x).reshape(-1, n)
    t = tau.reshape(-1, n)
    out = []
    for j in range(n):
        ss_tot = float(np.sum((t[:, j] - t[:, j].mean()) ** 2))
        out.append({
            "rmse": float(np.sqrt(np.mean(res[:, j] ** 2))),
            "max_abs": float(np.max(np.abs(res[:, j]))),
            "tau_rms": float(np.sqrt(np.mean(t[:, j] ** 2))),
            "r2": float(1 - np.sum(res[:, j] ** 2) / ss_tot) if ss_tot > 1e-6 else float("nan"),
        })
    return out


@dataclass
class FitResult:
    table: DHTable
    opts: FitOptions
    datasets: list[Dataset]
    warnings: list[str]
    x: np.ndarray
    rank: int
    sv: np.ndarray
    stats: list[dict]
    rmse: float
    tau_rms: float
    cv: list[dict] | None

    @property
    def n(self) -> int:
        return self.table.n

    @property
    def pi(self) -> np.ndarray:
        return self.x[:4 * self.n]

    @property
    def bias(self) -> np.ndarray:
        return self.x[4 * self.n:] if self.opts.bias else np.zeros(self.n)

    @property
    def n_poses(self) -> int:
        return int(sum(len(d.q_deg) for d in self.datasets))

    @property
    def n_unknowns(self) -> int:
        return len(self.x)

    def to_dict(self) -> dict:
        o = self.opts
        return {"dh": self.table.to_dict(), "g": o.g, "sign": o.sign, "torque_col": o.torque_col,
                "rcond": o.rcond, "bias_fitted": o.bias, "files": [d.name for d in self.datasets],
                "pi_layout": "per link [m*cx, m*cy, m*cz, m]; link i = joint i (0-based)",
                "rank": self.rank, "singular_values": self.sv.tolist(), "rmse": self.rmse,
                "pi": self.pi.tolist(), "bias": self.bias.tolist(), "stats": self.stats, "cv": self.cv}


def fit(table: DHTable, datasets: list[Dataset], opts: FitOptions, warnings: list[str] | None = None) -> FitResult:
    if not datasets:
        raise ValueError("没有数据")
    Y, tau, owner = build_system(table, datasets, opts.bias, opts.g, opts.sign)
    x, rank, sv = solve(Y, tau, opts.rcond)
    n = table.n
    cv = None
    if opts.cv and len(datasets) >= 2:
        cv = []
        for k, d in enumerate(datasets):
            tr, te = owner != k, owner == k
            xk, rk, _ = solve(Y[tr], tau[tr], opts.rcond)
            res = (tau[te] - Y[te] @ xk).reshape(-1, n)
            cv.append({"file": d.name, "rank": rk, "rmse": float(np.sqrt(np.mean(res ** 2))),
                       "rmse_per_joint": np.sqrt(np.mean(res ** 2, axis=0)).tolist(),
                       "tau_rms_per_joint": np.sqrt(np.mean(tau[te].reshape(-1, n) ** 2, axis=0)).tolist()})
    return FitResult(table, opts, datasets, list(warnings or []), x, rank, sv,
                     per_joint_stats(Y, tau, x, n), float(np.sqrt(np.mean((tau - Y @ x) ** 2))),
                     float(np.sqrt(np.mean(tau ** 2))), cv)


def run(paths: list[str | Path], table: DHTable, opts: FitOptions | None = None) -> FitResult:
    """读文件 -> 过滤 -> 拟合。"""
    opts = opts or FitOptions()
    datasets, warnings = load_datasets(paths, table, opts)
    return fit(table, datasets, opts, warnings)


# ============================================================ 报告
def _fmt(v: float, w: int = 8, p: int = 4) -> str:
    return f"{'nan':>{w}}" if not math.isfinite(v) else f"{v:{w}.{p}f}"


def format_report(r: FitResult) -> str:
    n, o = r.n, r.opts
    L: list[str] = []
    L.append(f"DH 来源: {r.table.source}    关节数 {n}    g={o.g}    力矩列: {o.torque_col}    sign={o.sign:+g}")
    L.append(f"数据: {len(r.datasets)} 个文件，{r.n_poses} 个位姿 -> {r.n_poses * n} 个方程，{r.n_unknowns} 个未知数")
    for d in r.datasets:
        sk = ", ".join(f"{k}:{v}" for k, v in d.skipped.items() if v)
        L.append(f"  {d.name}: {len(d.q_deg)} 点" + (f"（丢弃 {sk}）" if sk else ""))
    for w in r.warnings:
        L.append(f"警告: {w}")
    rank, sv = r.rank, r.sv
    cond = sv[0] / sv[rank - 1] if rank else float("inf")
    L.append(f"秩 {rank} / {r.n_unknowns}（rcond={o.rcond:g}），"
             f"保留奇异值范围 {sv[0]:.3g} .. {sv[rank - 1]:.3g}，条件数 {cond:.3g}")
    nxt = sv[rank] if rank < len(sv) else 0.0
    L.append(f"被截断的最大奇异值 {nxt:.3g}"
             f"{'  <- 与最小保留值很接近，rcond 可能卡在边界上' if rank and nxt > 0.3 * sv[rank - 1] else ''}")
    L.append("\n关节   力矩RMS   拟合RMSE    最大残差      R2")
    for j, s in enumerate(r.stats):
        L.append(f"  J{j}  {_fmt(s['tau_rms'])}  {_fmt(s['rmse'], 9, 5)}  {_fmt(s['max_abs'], 9, 5)}  {_fmt(s['r2'], 7, 3)}")
    L.append(f"总体 RMSE {r.rmse:.5f} N·m（力矩 RMS {r.tau_rms:.5f}）")
    if o.bias:
        L.append("\n力矩零偏 [N·m]: " + "  ".join(f"J{j}={v:+.4f}" for j, v in enumerate(r.bias)))
    L.append("\npi 最小范数解（每连杆 [m*cx, m*cy, m*cz, m]；不是唯一物理解，只保证复现 tau）:")
    for i in range(n):
        L.append(f"  link{i}: " + "  ".join(f"{v:+.5f}" for v in r.pi[4 * i:4 * i + 4]))
    if r.cv:
        L.append("\n留一文件交叉验证（用其余文件拟合，预测该文件）:")
        for c in r.cv:
            L.append(f"  {c['file']}: 秩 {c['rank']}，RMSE {c['rmse']:.5f}   各关节 "
                     + " ".join(f"{v:.3f}" for v in c["rmse_per_joint"])
                     + "   （力矩RMS " + " ".join(f"{v:.3f}" for v in c["tau_rms_per_joint"]) + "）")
    return "\n".join(L)
