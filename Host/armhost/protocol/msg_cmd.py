"""控制与参数消息：SET_JOINT_TORQUE / CLEAR_TORQUE / ESTOP / PARAM_*。

安全语义（固件必须实现，见 docs/02_protocol.md 第 7 节）：
* 力矩设定只是"设定"，是否真的经 CAN 下发由固件状态机决定。
* 每个关节的设定带有效期 hold_ms，过期自动清零；链路超时清零全部设定。
* 急停锁存期间拒绝所有力矩设定（ACK=REJECT_ESTOP）。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import ClassVar

from .enums import AckResult, EstopAction, HostTargetAction, MsgType, ParamType
from .msg_link import DecodeError, as_enum, need

_VALUE_FMT = {ParamType.U8: "<B3x", ParamType.I8: "<b3x", ParamType.U16: "<H2x",
              ParamType.I16: "<h2x", ParamType.U32: "<I", ParamType.I32: "<i",
              ParamType.F32: "<f"}


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
    """0 joint u8 | 1 action u8 | 2 reserved u16 | 4 target_deg f32

    一帧只更新一个关节。SET_OPEN/SET_CLOSED 要求 joint in [0, 6]；
    STOP_ALL 忽略 joint 和 target_deg，让全部关节退出闭环。"""
    TYPE: ClassVar[int] = MsgType.SET_HOST_TARGET
    S: ClassVar[struct.Struct] = struct.Struct("<BBHf")
    joint: int = 0
    action: HostTargetAction | int = HostTargetAction.SET_OPEN
    target_deg: float = 0.0

    def encode(self) -> bytes:
        action = HostTargetAction(self.action)
        if action != HostTargetAction.STOP_ALL and not 0 <= self.joint < 7:
            raise ValueError(f"SET_HOST_TARGET joint {self.joint} out of range")
        return self.S.pack(self.joint, int(action), 0, float(self.target_deg))

    @classmethod
    def decode(cls, p: bytes) -> "SetHostTarget":
        need(p, cls.S.size, "SET_HOST_TARGET")
        joint, action, _reserved, target_deg = cls.S.unpack_from(p)
        return cls(joint, as_enum(HostTargetAction, action), target_deg)


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
