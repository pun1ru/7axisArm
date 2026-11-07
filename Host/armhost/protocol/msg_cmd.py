"""控制与参数消息：SET_JOINT_TORQUE / CLEAR_TORQUE / ESTOP / PARAM_*。

安全语义（固件必须实现，见 docs/02_protocol.md 第 7 节）：
* 力矩设定只是"设定"，是否真的经 CAN 下发由固件状态机决定。
* 每个关节的设定带有效期 hold_ms，过期自动清零；链路超时清零全部设定。
* 急停锁存期间拒绝所有力矩设定（ACK=REJECT_ESTOP）。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import ClassVar

from .enums import (AckResult, EstopAction, ExchangeAction, HostIkAction, HostTargetAction,
                    MsgType, ParamType)
from .msg_link import DecodeError, as_enum, need

_VALUE_FMT = {ParamType.U8: "<B3x", ParamType.I8: "<b3x", ParamType.U16: "<H2x",
              ParamType.I16: "<h2x", ParamType.U32: "<I", ParamType.I32: "<i",
              ParamType.F32: "<f"}

SET_HOST_TARGET_MULTI_TURN = 0x0001


def pack_value(ptype: ParamType, value) -> bytes:
    return struct.pack(_VALUE_FMT[ParamType(ptype)], value)


def unpack_value(ptype: int, raw: bytes):
    try:
        return struct.unpack(_VALUE_FMT[ParamType(ptype)], raw)[0]
    except ValueError:
        return int.from_bytes(raw, "little")


@dataclass(slots=True)
class TorqueItem:
    joint: int
    torque_nm: float


@dataclass(slots=True)
class SetJointTorque:
    """0 hold_ms u16 | 2 count u8 | 3 reserved u8 | 4 起 count 条 {joint u8, torque_nm f32}（5 字节，紧凑）
    hold_ms：本条设定的有效期，0 表示使用固件默认值，超过 torque_hold_max_ms 由固件截断。
    未列出的关节保持各自原有设定与有效期。"""
    TYPE: ClassVar[int] = MsgType.SET_JOINT_TORQUE
    H: ClassVar[struct.Struct] = struct.Struct("<HBB")
    I: ClassVar[struct.Struct] = struct.Struct("<Bf")
    hold_ms: int = 100
    items: list[TorqueItem] = field(default_factory=list)

    def encode(self) -> bytes:
        out = self.H.pack(self.hold_ms & 0xFFFF, len(self.items), 0)
        return out + b"".join(self.I.pack(i.joint, i.torque_nm) for i in self.items)

    @classmethod
    def decode(cls, p: bytes) -> "SetJointTorque":
        need(p, cls.H.size, "SET_JOINT_TORQUE")
        hold, n, _ = cls.H.unpack_from(p)
        need(p, cls.H.size + n * cls.I.size, "SET_JOINT_TORQUE")
        items = [TorqueItem(*cls.I.unpack_from(p, cls.H.size + k * cls.I.size)) for k in range(n)]
        return cls(hold, items)


@dataclass(slots=True)
class ClearTorque:
    """0 joint_mask u32  位 i=1 清除关节 i 的设定；0xFFFFFFFF 全部"""
    TYPE: ClassVar[int] = MsgType.CLEAR_TORQUE
    S: ClassVar[struct.Struct] = struct.Struct("<I")
    joint_mask: int = 0xFFFFFFFF

    def encode(self) -> bytes:
        return self.S.pack(self.joint_mask & 0xFFFFFFFF)

    @classmethod
    def decode(cls, p: bytes) -> "ClearTorque":
        need(p, cls.S.size, "CLEAR_TORQUE")
        return cls(*cls.S.unpack_from(p))


@dataclass(slots=True)
class Estop:
    """0 action u8  1=锁存急停（清全部设定并拒绝新设定）0=解除"""
    TYPE: ClassVar[int] = MsgType.ESTOP
    S: ClassVar[struct.Struct] = struct.Struct("<B")
    action: EstopAction | int = EstopAction.LATCH

    def encode(self) -> bytes:
        return self.S.pack(int(self.action))

    @classmethod
    def decode(cls, p: bytes) -> "Estop":
        need(p, cls.S.size, "ESTOP")
        return cls(as_enum(EstopAction, cls.S.unpack_from(p)[0]))


@dataclass(slots=True)
class SetHostTarget:
    """0 joint u8 | 1 action u8 | 2 flags u16 | 4 target_deg f32

    一帧只更新一个关节。SET_OPEN/SET_CLOSED 要求 joint in [0, 6]；
    STOP_ALL 忽略 joint 和 target_deg，让全部关节退出闭环。
    flags bit0 = 1 时 target_deg 为多圈角；0 为旧协议的 单圈绝对角。"""
    TYPE: ClassVar[int] = MsgType.SET_HOST_TARGET
    S: ClassVar[struct.Struct] = struct.Struct("<BBHf")
    joint: int = 0
    action: HostTargetAction | int = HostTargetAction.SET_OPEN
    target_deg: float = 0.0
    multi_turn: bool = False

    def encode(self) -> bytes:
        action = HostTargetAction(self.action)
        if action != HostTargetAction.STOP_ALL and not 0 <= self.joint < 7:
            raise ValueError(f"SET_HOST_TARGET joint {self.joint} out of range")
        if action != HostTargetAction.STOP_ALL and not math.isfinite(float(self.target_deg)):
            raise ValueError("SET_HOST_TARGET target_deg must be finite")
        flags = (SET_HOST_TARGET_MULTI_TURN
                 if self.multi_turn and action != HostTargetAction.STOP_ALL else 0)
        return self.S.pack(self.joint, int(action), flags, float(self.target_deg))

    @classmethod
    def decode(cls, p: bytes) -> "SetHostTarget":
        need(p, cls.S.size, "SET_HOST_TARGET")
        joint, action, flags, target_deg = cls.S.unpack_from(p)
        return cls(joint, as_enum(HostTargetAction, action), target_deg,
                   bool(flags & SET_HOST_TARGET_MULTI_TURN))


@dataclass(slots=True)
class SetIkTarget:
    """0 action u8 | 1 branch u8 | 2 psi_mode u8 | 3 reserved u8
    4 x f32 | 8 y f32 | 12 z f32
    16 roll f32 | 20 pitch f32 | 24 yaw f32 | 28 psi_deg f32
    32 cost_weight[7] f32 | 60 reserved u32，共 64 B

    SET：从 HOST 进入或更新 HOST_IK。branch = bit0 肩 / bit1 肘 / bit2 腕，
    置位表示对应 λ = -1。STOP：退出 HOST_IK 并让全部上位机闭环退出。
    psi_mode=0 使用 psi_deg；psi_mode=1 在当前分支内按权重自动选择 ψ。
    位姿使用 ForwardKinematics 的腰部连杆坐标系，角度单位 deg。"""
    TYPE: ClassVar[int] = MsgType.SET_IK_TARGET
    LEGACY_S: ClassVar[struct.Struct] = struct.Struct("<BBH7f")
    S: ClassVar[struct.Struct] = struct.Struct("<BBBB7f7f4x")
    action: HostIkAction | int = HostIkAction.SET
    branch: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    psi_deg: float = 0.0
    auto_psi: bool = False
    cost_weights: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

    def encode(self) -> bytes:
        action = HostIkAction(self.action)
        weights = tuple(float(v) for v in self.cost_weights)
        if len(weights) != 7:
            raise ValueError("SET_IK_TARGET cost_weights must contain 7 values")
        if action == HostIkAction.SET:
            if not 0 <= self.branch < 8:
                raise ValueError(f"SET_IK_TARGET branch {self.branch} out of range")
            values = (self.x, self.y, self.z, self.roll, self.pitch, self.yaw, self.psi_deg)
            if not all(math.isfinite(float(v)) for v in values):
                raise ValueError("SET_IK_TARGET contains non-finite value")
            if not all(math.isfinite(v) and v >= 0.0 for v in weights):
                raise ValueError("SET_IK_TARGET cost_weights must be finite and non-negative")
            if self.auto_psi and sum(weights) <= 0.0:
                raise ValueError("SET_IK_TARGET auto psi needs at least one positive cost weight")
        return self.S.pack(int(action), int(self.branch), int(bool(self.auto_psi)), 0,
                           float(self.x), float(self.y), float(self.z), float(self.roll),
                           float(self.pitch), float(self.yaw), float(self.psi_deg), *weights)

    @classmethod
    def decode(cls, p: bytes) -> "SetIkTarget":
        if len(p) == cls.LEGACY_S.size:
            action, branch, _reserved, x, y, z, roll, pitch, yaw, psi = cls.LEGACY_S.unpack_from(p)
            return cls(as_enum(HostIkAction, action), branch, x, y, z, roll, pitch, yaw, psi)
        need(p, cls.S.size, "SET_IK_TARGET")
        action, branch, auto_psi, _reserved, x, y, z, roll, pitch, yaw, psi, *weights = cls.S.unpack_from(p)
        if auto_psi not in (0, 1):
            raise DecodeError(f"SET_IK_TARGET psi_mode {auto_psi} is invalid")
        return cls(as_enum(HostIkAction, action), branch, x, y, z, roll, pitch, yaw, psi,
                   bool(auto_psi), tuple(weights))


@dataclass(slots=True)
class ExchangeCommand:
    """ArmSim 兑换站扩展命令，共 40 B。

    0 version u8 | 1 action u8 | 2 difficulty u8 | 3 flags u8
    4 q_target_deg f32 | 8 seed u32
    12 x_mm | 16 y_mm | 20 z_mm | 24 theta_deg | 28 phi_deg | 32 alpha_deg（均为 f32）
    36 reserved u32

    flags bit0=1 表示携带手动位姿；MOVE_TO_POSE 使用该位姿，RANDOMIZE 只使用 difficulty/seed。
    seed=0 时由 ArmSim 生成随机种子，状态报文会回报实际种子以便复现。
    """
    TYPE: ClassVar[int] = MsgType.EXCHANGE_COMMAND
    S: ClassVar[struct.Struct] = struct.Struct("<BBBBfI6fI")
    action: ExchangeAction | int = ExchangeAction.NONE
    difficulty: int = 1
    q_target_deg: float = 0.0
    x_mm: float = -200.0
    y_mm: float = 200.0
    z_mm: float = 800.0
    theta_deg: float = 0.0
    phi_deg: float = 90.0
    alpha_deg: float = 0.0
    seed: int = 0
    use_pose: bool = False

    def encode(self) -> bytes:
        action = ExchangeAction(self.action)
        if not 1 <= int(self.difficulty) <= 4:
            raise ValueError("EXCHANGE_COMMAND difficulty must be in [1, 4]")
        values = (self.q_target_deg, self.x_mm, self.y_mm, self.z_mm,
                  self.theta_deg, self.phi_deg, self.alpha_deg)
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError("EXCHANGE_COMMAND contains non-finite value")
        flags = 1 if self.use_pose else 0
        return self.S.pack(1, int(action), int(self.difficulty), flags, float(self.q_target_deg),
                           int(self.seed) & 0xFFFFFFFF, float(self.x_mm), float(self.y_mm),
                           float(self.z_mm), float(self.theta_deg), float(self.phi_deg),
                           float(self.alpha_deg), 0)

    @classmethod
    def decode(cls, p: bytes) -> "ExchangeCommand":
        need(p, cls.S.size, "EXCHANGE_COMMAND")
        version, action, difficulty, flags, q_target, seed, x, y, z, theta, phi, alpha, _ = \
            cls.S.unpack_from(p)
        if version != 1:
            raise DecodeError(f"EXCHANGE_COMMAND version {version} is unsupported")
        return cls(as_enum(ExchangeAction, action), difficulty, q_target, x, y, z,
                   theta, phi, alpha, seed, bool(flags & 1))


@dataclass(slots=True)
class ParamRead:
    """0 param_id u16 | 2 index u8 | 3 reserved u8"""
    TYPE: ClassVar[int] = MsgType.PARAM_READ
    S: ClassVar[struct.Struct] = struct.Struct("<HBx")
    param_id: int = 0
    index: int = 0

    def encode(self) -> bytes:
        return self.S.pack(self.param_id, self.index)

    @classmethod
    def decode(cls, p: bytes) -> "ParamRead":
        need(p, cls.S.size, "PARAM_READ")
        return cls(*cls.S.unpack_from(p))


@dataclass(slots=True)
class ParamWrite:
    """0 param_id u16 | 2 index u8 | 3 type u8 | 4 value 4 字节（按 type 解释，不足 4 字节高位补 0）"""
    TYPE: ClassVar[int] = MsgType.PARAM_WRITE
    S: ClassVar[struct.Struct] = struct.Struct("<HBB4s")
    param_id: int = 0
    index: int = 0
    ptype: ParamType | int = ParamType.F32
    value: int | float = 0

    def encode(self) -> bytes:
        return self.S.pack(self.param_id, self.index, int(self.ptype), pack_value(self.ptype, self.value))

    @classmethod
    def decode(cls, p: bytes) -> "ParamWrite":
        need(p, cls.S.size, "PARAM_WRITE")
        pid, idx, t, raw = cls.S.unpack_from(p)
        return cls(pid, idx, as_enum(ParamType, t), unpack_value(t, raw))


@dataclass(slots=True)
class ParamValue:
    """PARAM_READ / PARAM_WRITE 的应答（替代 ACK）
    0 req_seq u16 | 2 param_id u16 | 4 index u8 | 5 type u8 | 6 result u8 | 7 access u8 | 8 value 4 字节
    access: bit0 可读 bit1 可写；result 为 AckResult"""
    TYPE: ClassVar[int] = MsgType.PARAM_VALUE
    S: ClassVar[struct.Struct] = struct.Struct("<HHBBBB4s")
    req_seq: int = 0
    param_id: int = 0
    index: int = 0
    ptype: ParamType | int = ParamType.F32
    result: AckResult | int = AckResult.OK
    access: int = 1
    value: int | float = 0

    def encode(self) -> bytes:
        return self.S.pack(self.req_seq & 0xFFFF, self.param_id, self.index, int(self.ptype),
                           int(self.result), self.access, pack_value(self.ptype, self.value))

    @classmethod
    def decode(cls, p: bytes) -> "ParamValue":
        need(p, cls.S.size, "PARAM_VALUE")
        s, pid, idx, t, r, acc, raw = cls.S.unpack_from(p)
        return cls(s, pid, idx, as_enum(ParamType, t), as_enum(AckResult, r), acc,
                   unpack_value(t, raw))
