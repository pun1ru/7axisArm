"""消息负载编解码（第 1 部分：通用工具 + 链路消息）。

每个 dataclass 的字段顺序和 struct 格式就是 docs/02_protocol.md 的偏移表，
改任何一处都要同步 firmware_ref/arm_proto.h。

兼容规则：同一 major 版本内只允许在负载末尾追加字段。
解码时负载比 v1.0 定义长 -> 忽略尾部；比定义短 -> DecodeError。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import ClassVar

from .enums import ArmRole, MsgType


class DecodeError(ValueError):
    pass


def need(p: bytes, n: int, name: str) -> None:
    if len(p) < n:
        raise DecodeError(f"{name}: payload {len(p)} < {n}")


def as_enum(cls, v):
    """未知值原样保留为 int，向前兼容。"""
    try:
        return cls(v)
    except ValueError:
        return v


@dataclass(slots=True)
class Heartbeat:
    TYPE: ClassVar[int] = MsgType.HEARTBEAT
    S: ClassVar[struct.Struct] = struct.Struct("<II")
    uptime_ms: int = 0
    rx_frames_ok: int = 0      # 发送方累计收到的有效帧数，对端可据此估计己方丢帧

    def encode(self) -> bytes:
        return self.S.pack(self.uptime_ms & 0xFFFFFFFF, self.rx_frames_ok & 0xFFFFFFFF)

    @classmethod
    def decode(cls, p: bytes) -> "Heartbeat":
        need(p, cls.S.size, "HEARTBEAT")
        return cls(*cls.S.unpack_from(p))


@dataclass(slots=True)
class DeviceInfoReq:
    TYPE: ClassVar[int] = MsgType.DEVICE_INFO_REQ

    def encode(self) -> bytes:
        return b""

    @classmethod
    def decode(cls, p: bytes) -> "DeviceInfoReq":
        return cls()


@dataclass(slots=True)
class DeviceInfo:
    """偏移: 0 proto_version u8 | 1 joint_count u8 | 2 joint_record_size u8 | 3 arm_role u8
    4 max_frame u16 | 6 state_rate_hz u16 | 8 status_rate_hz u16 | 10 link_timeout_ms u16
    12 torque_hold_max_ms u16 | 14 fw_build u16 | 16 fw_name char[12]   共 28 字节"""
    TYPE: ClassVar[int] = MsgType.DEVICE_INFO
    S: ClassVar[struct.Struct] = struct.Struct("<BBBBHHHHHH12s")
    proto_version: int = 0x10
    joint_count: int = 7
    joint_record_size: int = 28
    arm_role: ArmRole | int = ArmRole.MASTER
    max_frame: int = 256
    state_rate_hz: int = 200
    status_rate_hz: int = 10
    link_timeout_ms: int = 300
    torque_hold_max_ms: int = 500
    fw_build: int = 0
    fw_name: str = ""

    def encode(self) -> bytes:
        return self.S.pack(self.proto_version, self.joint_count, self.joint_record_size,
                           int(self.arm_role), self.max_frame, self.state_rate_hz,
                           self.status_rate_hz, self.link_timeout_ms, self.torque_hold_max_ms,
                           self.fw_build, self.fw_name.encode("ascii", "replace")[:12])

    @classmethod
    def decode(cls, p: bytes) -> "DeviceInfo":
        need(p, cls.S.size, "DEVICE_INFO")
        v = list(cls.S.unpack_from(p))
        v[3] = as_enum(ArmRole, v[3])
        v[10] = v[10].split(b"\0", 1)[0].decode("ascii", "replace")
        return cls(*v)
