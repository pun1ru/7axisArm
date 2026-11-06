"""改进 DH（Craig）正运动学，与固件 LibSE/Algorithm/Kinematics/dh.c 逐项一致。

固件约定（dh.c DH_Transform）：
    theta = (q_deg + theta_offset_deg)，alpha、a、d 取第 i 行
    T_i = RotX(alpha) · TransX(a) · RotZ(theta) · TransZ(d)
    末端 = T_0 · T_1 · … · T_{n-1}（第 i 行的 q 对应第 i 个关节）

所以帧 i+1 的原点就是关节 i 的位置，关节 i 绕帧 i+1 的 z 轴旋转。
角度：协议与 DH 表都用 deg，这里只在算三角函数时转成 rad。
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_MODULE_INIT = REPO_ROOT / "Real" / "DIYcontroller" / "App" / "module_init.c"
ARMSIM_MAIN = REPO_ROOT / "Simulation" / "ArmSim" / "main.cpp"


@dataclass
class DHRow:
    alpha_deg: float = 0.0
    a_m: float = 0.0
    d_m: float = 0.0
    theta_offset_deg: float = 0.0

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.alpha_deg, self.a_m, self.d_m, self.theta_offset_deg)


# 旧固件 module_init.c 的副本（2026-10-30），只用于固件对照测试 / 显式恢复
BUILTIN_DH = (
    DHRow(0.0, 0.0, 0.0626, 0.0),
    DHRow(90.0, 0.0, 0.0, 0.0),
    DHRow(90.0, 0.0, -0.148, 0.0),
    DHRow(90.0, 0.0, 0.0, 0.0),
    DHRow(90.0, 0.0, 0.1, 0.0),
    DHRow(90.0, 0.0, 0.0, 0.0),
    DHRow(90.0, 0.0, 0.0, 0.0),
)

# 当前 BigArm（Simulation/mujoco/BigArm）的副本。源码在 ArmSim main.cpp 的 MakeArmConfig()；
# 解析不到源码时用它，避免 Host 又退回旧 DIY 臂参数。
BIGARM_DH = (
    DHRow(0.0, 0.4105, -0.0897, -72.5122038),
    DHRow(90.0, 0.0, 0.0, -90.0),
    DHRow(90.0, 0.0, 0.188, -90.0),
    DHRow(90.0, 0.0, -0.0025, 170.7537190),
    DHRow(90.0, 0.0, 0.207, 10.6949890),
    DHRow(90.0, 0.0, 0.0, -148.8585943),
    DHRow(90.0, 0.0, 0.0, 0.0),
)
BIGARM_BASE_XYZ_M = (0.000558840342, -0.0208529944, 0.038)
BIGARM_BASE_RPY_DEG = (90.0, -90.0, 0.0)


@dataclass
class DHTable:
    rows: list[DHRow] = field(default_factory=lambda: [DHRow(*r.as_tuple()) for r in BUILTIN_DH])
    source: str = "内置副本"
    # base -> DH 基座（腰部连杆坐标系）的安装位姿，与 URDF 的 joint1 origin 同口径
    base_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def n(self) -> int:
        return len(self.rows)

    def copy(self) -> "DHTable":
        return DHTable([DHRow(*r.as_tuple()) for r in self.rows], self.source,
                       self.base_xyz_m, self.base_rpy_deg)

    @property
    def base_transform(self) -> np.ndarray:
        """世界（URDF/MJCF base）到 DH 基座的 4x4 齐次变换。"""
        return transform_from_xyz_rpy_deg(*self.base_xyz_m, *self.base_rpy_deg)

    # ------------------------------------------------------------ 序列化
    def to_dict(self) -> dict:
        return {"convention": "modified-DH (Craig), row i = joint i, units deg/m",
                "rows": [asdict(r) for r in self.rows],
                "base": {"xyz_m": list(self.base_xyz_m), "rpy_deg": list(self.base_rpy_deg)}}

    @classmethod
    def from_dict(cls, d: dict, source: str = "文件") -> "DHTable":
        rows = [DHRow(float(r["alpha_deg"]), float(r["a_m"]), float(r["d_m"]),
                      float(r.get("theta_offset_deg", 0.0))) for r in d["rows"]]
        if not rows:
            raise ValueError("DH 表为空")
        base = d.get("base", {})
        xyz = tuple(float(v) for v in base.get("xyz_m", (0.0, 0.0, 0.0)))
        rpy = tuple(float(v) for v in base.get("rpy_deg", (0.0, 0.0, 0.0)))
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError("base.xyz_m / base.rpy_deg 必须各含 3 个数")
        return cls(rows, source, xyz, rpy)

    def save_json(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")

    @classmethod
    def load_json(cls, path: str | Path) -> "DHTable":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")), f"文件 {Path(path).name}")

    # ------------------------------------------------------------ 来自固件
    _DEF = re.compile(r"#define\s+ARM_DH_J(\d+)\s*\{([^}]*)\}")
    _NUM = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")

    @classmethod
    def from_firmware_source(cls, path: str | Path = FIRMWARE_MODULE_INIT) -> "DHTable":
        """解析 module_init.c 里的 #define ARM_DH_Jx {alpha, a, d, theta}（只读，不改固件）"""
        text = Path(path).read_text(encoding="utf-8", errors="replace")
        found: dict[int, DHRow] = {}
        for m in cls._DEF.finditer(text):
            nums = [float(x) for x in cls._NUM.findall(m.group(2).replace("f", ""))]
            if len(nums) != 4:
                raise ValueError(f"ARM_DH_J{m.group(1)} 应有 4 个数，实际 {nums}")
            found[int(m.group(1))] = DHRow(*nums)
        if not found or sorted(found) != list(range(len(found))):
            raise ValueError(f"{path} 里没有连续的 ARM_DH_J0.. 定义")
        return cls([found[i] for i in range(len(found))], "单片机工程 module_init.c")

    @classmethod
    def from_armsim_source(cls, path: str | Path = ARMSIM_MAIN) -> "DHTable":
        """解析 ArmSim main.cpp 的 MakeArmConfig()：当前 BigArm DH + 腰部安装位姿。"""
        text = Path(path).read_text(encoding="utf-8", errors="replace")
        m_dh = re.search(r"c\.dh\s*=\s*\{\{", text)
        if m_dh is None:
            raise ValueError(f"{path} 里没有 c.dh = {{{{ ... }}}}")
        end = text.find("}};", m_dh.end())
        if end < 0:
            raise ValueError(f"{path} 的 c.dh 块没有闭合")
        rows = []
        for body in re.findall(r"\{([^{}]*)\}", text[m_dh.end():end]):
            nums = [float(v) for v in cls._NUM.findall(body.replace("f", ""))]
            if len(nums) != 4:
                raise ValueError(f"DH 行应有 4 个数，实际 {nums}")
            rows.append(DHRow(*nums))
        if not rows:
            raise ValueError(f"{path} 的 c.dh 为空")

        m_mount = re.search(
            r"c\.waist_mount\s*=\s*TransformFromXyzRpy\(\s*\{([^}]*)\}\s*,\s*\{([^}]*)\}\s*\)",
            text)
        if m_mount is None:
            raise ValueError(f"{path} 里没有 c.waist_mount = TransformFromXyzRpy(...)")
        xyz = tuple(float(v) for v in cls._NUM.findall(m_mount.group(1).replace("f", "")))
        rpy_rad = tuple(float(v) for v in cls._NUM.findall(m_mount.group(2).replace("f", "")))
        if len(xyz) != 3 or len(rpy_rad) != 3:
            raise ValueError(f"腰部安装位姿应有 xyz/rpy 各 3 个数，实际 {xyz} {rpy_rad}")
        # main.cpp 的 TransformFromXyzRpy 使用 rad；DHTable 对外统一存 deg。
        rpy = tuple(math.degrees(v) for v in rpy_rad)
        return cls(rows, "ArmSim BigArm main.cpp", xyz, rpy)


def default_dh() -> DHTable:
    """旧固件默认 DH；保留给固件对照、重力拟合和显式恢复操作。"""
    try:
        return DHTable.from_firmware_source()
    except (OSError, ValueError):
        return DHTable()


def current_dh() -> DHTable:
    """当前 ArmSim/BigArm 模型（含腰部安装位姿）；读不到源码时用内置 BigArm 副本。"""
    try:
        return DHTable.from_armsim_source()
    except (OSError, ValueError):
        pass
    return DHTable([DHRow(*r.as_tuple()) for r in BIGARM_DH], "内置 BigArm",
                   BIGARM_BASE_XYZ_M, BIGARM_BASE_RPY_DEG)


def transform_from_xyz_rpy_deg(x: float, y: float, z: float,
                               roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """URDF/MJCF 的 base -> child：T = Trans(xyz) · Rz(yaw) · Ry(pitch) · Rx(roll)。"""
    rr, pp, yy = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(rr), math.sin(rr)
    cp, sp = math.cos(pp), math.sin(pp)
    cy, sy = math.cos(yy), math.sin(yy)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
                     [-sp, cp * sr, cp * cr, z],
                     [0.0, 0.0, 0.0, 1.0]])


# ---------------------------------------------------------------- 运动学
def link_transform(row: DHRow, q_deg: float) -> np.ndarray:
    th = math.radians(q_deg + row.theta_offset_deg)
    al = math.radians(row.alpha_deg)
    ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return np.array([[ct, -st, 0.0, row.a_m],
                     [st * ca, ct * ca, -sa, -sa * row.d_m],
                     [st * sa, ct * sa, ca, ca * row.d_m],
                     [0.0, 0.0, 0.0, 1.0]])


def forward_frames(table: DHTable, q_deg) -> list[np.ndarray]:
    """返回 n+1 个基座系下的位姿：[T_base(=I), T_1, …, T_n]；关节 i 位于 frames[i+1]，绕其 z 轴转。

    q 可以是多圈角（固件 joint_deg 已改为多圈），三角函数天然处理。
    """
    T = np.eye(4)
    out = [T]
    for i, row in enumerate(table.rows):
        q = float(q_deg[i]) if i < len(q_deg) else 0.0
        T = T @ link_transform(row, q)
        out.append(T)
    return out


def euler_zyx_deg(R: np.ndarray, lock_eps: float = 1e-6) -> tuple[float, float, float]:
    """逐行照抄固件 position_tran.h Matrix_ToEuler：返回 (roll, pitch, yaw) deg，Z-Y-X 顺序"""
    horizon = math.hypot(R[0, 0], R[1, 0])
    pitch = math.atan2(-R[2, 0], horizon)
    if horizon > lock_eps:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                    # 万向锁：yaw 置 0
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
