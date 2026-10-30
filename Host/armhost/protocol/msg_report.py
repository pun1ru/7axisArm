"""上报类消息：ACK / JOINT_STATE / SYSTEM_STATUS / EVENT。"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import ClassVar

from .enums import (AckResult, ArmRole, ArmState, EventCode, JointFlag, MotorStatus,
                    MsgType, Severity, StatusFlag)
from .msg_link import DecodeError, as_enum, need


@dataclass(slots=True)
class Ack:
    """0 ack_seq u16 | 2 ack_type u8 | 3 result u8 | 4 detail u32   共 8 字节
    detail：SET_JOINT_TORQUE 时为被限幅关节的位掩码，其余命令为 0。"""
    TYPE: ClassVar[int] = MsgType.ACK
    S: ClassVar[struct.Struct] = struct.Struct("<HBBI")
    ack_seq: int = 0
    ack_type: int = 0
    result: AckResult | int = AckResult.OK
    detail: int = 0

    def encode(self) -> bytes:
        return self.S.pack(self.ack_seq & 0xFFFF, self.ack_type, int(self.result), self.detail)

    @classmethod
    def decode(cls, p: bytes) -> "Ack":
        need(p, cls.S.size, "ACK")
        s, t, r, d = cls.S.unpack_from(p)
        return cls(s, t, as_enum(AckResult, r), d)

    @property
    def ok(self) -> bool:
        return self.result in (AckResult.OK, AckResult.OK_CLAMPED)


@dataclass(slots=True)
class JointRecord:
    """单关节记录，v1.0 为 28 字节
    0 angle_deg f32       关节角 deg，(-180,180]，已按 direction 折算到关节 +q 方向
    4 velocity_dps f32    关节角速度 deg/s，同向
    8 torque_nm f32       电机反馈力矩 N*m，同向
    12 target_deg f32     角度环目标 deg
    16 torque_cmd_nm f32  本周期实际写进电机指令的力矩 N*m，同向
    20 encoder u16        原始编码器值
    22 motor_status u8    MotorStatus_t
    23 error_id u8        电机错误码
    24 temperature_c i8   温度 degC
    25 joint_flags u8     JointFlag
    26 feedback_count u16 反馈帧计数低 16 位（Feedback_t.timestamp）"""
    S: ClassVar[struct.Struct] = struct.Struct("<fffffHBBbBH")
    angle_deg: float = 0.0
    velocity_dps: float = 0.0
    torque_nm: float = 0.0
    target_deg: float = 0.0
    torque_cmd_nm: float = 0.0
    encoder: int = 0
    motor_status: MotorStatus | int = MotorStatus.NO_FEEDBACK
    error_id: int = 0
    temperature_c: int = 0
    joint_flags: JointFlag | int = JointFlag.NONE
    feedback_count: int = 0

    def encode(self) -> bytes:
        return self.S.pack(self.angle_deg, self.velocity_dps, self.torque_nm, self.target_deg,
                           self.torque_cmd_nm, self.encoder & 0xFFFF, int(self.motor_status),
                           self.error_id, max(-128, min(127, int(self.temperature_c))),
                           int(self.joint_flags), self.feedback_count & 0xFFFF)

    @classmethod
    def decode(cls, p: bytes, off: int = 0) -> "JointRecord":
        v = list(cls.S.unpack_from(p, off))
        v[6] = as_enum(MotorStatus, v[6])
        v[9] = JointFlag(v[9])
        return cls(*v)


@dataclass(slots=True)
class JointState:
    """头 12 字节:
    0 mcu_time_us u32     采样时刻，MCU 微秒计数，约 71.6 分钟回绕
    4 arm_state u8 | 5 joint_count u8 | 6 record_size u8 | 7 reserved u8
    8 status_flags u16    StatusFlag
    10 host_cmd_age_ms u16 距上一条有效上位机力矩命令的时间，饱和到 65535
    12 起 joint_count 条记录，每条 record_size 字节"""
    TYPE: ClassVar[int] = MsgType.JOINT_STATE
    H: ClassVar[struct.Struct] = struct.Struct("<IBBBBHH")
    mcu_time_us: int = 0
    arm_state: ArmState | int = ArmState.PROTECTED
    status_flags: StatusFlag | int = StatusFlag.NONE
    host_cmd_age_ms: int = 0xFFFF
    joints: list[JointRecord] = field(default_factory=list)

    def encode(self) -> bytes:
        rs = JointRecord.S.size
        out = self.H.pack(self.mcu_time_us & 0xFFFFFFFF, int(self.arm_state), len(self.joints),
                          rs, 0, int(self.status_flags), min(self.host_cmd_age_ms, 0xFFFF))
        return out + b"".join(j.encode() for j in self.joints)

    @classmethod
    def decode(cls, p: bytes) -> "JointState":
        need(p, cls.H.size, "JOINT_STATE")
        t, st, n, rs, _r, fl, age = cls.H.unpack_from(p)
        if rs < JointRecord.S.size:
            raise DecodeError(f"JOINT_STATE: record_size {rs} < {JointRecord.S.size}")
        need(p, cls.H.size + n * rs, "JOINT_STATE")
        joints = [JointRecord.decode(p, cls.H.size + i * rs) for i in range(n)]
        return cls(t, as_enum(ArmState, st), StatusFlag(fl), age, joints)


@dataclass(slots=True)
class CanBusStatus:
    """每路 CAN 12 字节: tx_ok u32 | tx_fail u32 | tx_err_cnt u8 | rx_err_cnt u8 | bus_flags u8 | rsv u8
    bus_flags: bit0 bus_off  bit1 error_passive  bit2 warning"""
    S: ClassVar[struct.Struct] = struct.Struct("<IIBBBB")
    tx_ok: int = 0
    tx_fail: int = 0
    tx_err_cnt: int = 0
    rx_err_cnt: int = 0
    bus_flags: int = 0

    def encode(self) -> bytes:
        return self.S.pack(self.tx_ok & 0xFFFFFFFF, self.tx_fail & 0xFFFFFFFF,
                           min(self.tx_err_cnt, 255), min(self.rx_err_cnt, 255), self.bus_flags, 0)

    @classmethod
    def decode(cls, p: bytes, off: int) -> "CanBusStatus":
        a, b, c, d, e, _ = cls.S.unpack_from(p, off)
        return cls(a, b, c, d, e)


@dataclass(slots=True)
class SystemStatus:
    """0 mcu_time_us u32 | 4 uptime_ms u32 | 8 arm_state u8 | 9 arm_role u8
    10 rc_switch_left u8 | 11 rc_switch_right u8 （DT7: 1 上 3 中 2 下，0 无效）
    12 status_flags u16 | 14 task_count u8 | 15 can_count u8
    16 link: rx_ok u32 | rx_crc_err u32 | rx_other_err u32 | tx_drop u32   (16 字节)
    32 task_offline_count u16 × task_count
    之后 CanBusStatus × can_count（每路 12 字节）"""
    TYPE: ClassVar[int] = MsgType.SYSTEM_STATUS
    H: ClassVar[struct.Struct] = struct.Struct("<IIBBBBHBBIIII")
    mcu_time_us: int = 0
    uptime_ms: int = 0
    arm_state: ArmState | int = ArmState.PROTECTED
    arm_role: ArmRole | int = ArmRole.MASTER
    rc_switch_left: int = 0
    rc_switch_right: int = 0
    status_flags: StatusFlag | int = StatusFlag.NONE
    link_rx_ok: int = 0
    link_rx_crc_err: int = 0
    link_rx_other_err: int = 0
    link_tx_drop: int = 0
    task_offline: list[int] = field(default_factory=list)
    can: list[CanBusStatus] = field(default_factory=list)

    def encode(self) -> bytes:
        out = self.H.pack(self.mcu_time_us & 0xFFFFFFFF, self.uptime_ms & 0xFFFFFFFF,
                          int(self.arm_state), int(self.arm_role), self.rc_switch_left,
                          self.rc_switch_right, int(self.status_flags), len(self.task_offline),
                          len(self.can), self.link_rx_ok, self.link_rx_crc_err,
                          self.link_rx_other_err, self.link_tx_drop)
        out += struct.pack(f"<{len(self.task_offline)}H", *(min(x, 0xFFFF) for x in self.task_offline))
        return out + b"".join(c.encode() for c in self.can)

    @classmethod
    def decode(cls, p: bytes) -> "SystemStatus":
        need(p, cls.H.size, "SYSTEM_STATUS")
        (t, up, st, role, swl, swr, fl, nt, nc, a, b, c, d) = cls.H.unpack_from(p)
        off = cls.H.size
        need(p, off + 2 * nt + CanBusStatus.S.size * nc, "SYSTEM_STATUS")
        tasks = list(struct.unpack_from(f"<{nt}H", p, off))
        off += 2 * nt
        cans = [CanBusStatus.decode(p, off + i * CanBusStatus.S.size) for i in range(nc)]
        return cls(t, up, as_enum(ArmState, st), as_enum(ArmRole, role), swl, swr,
                   StatusFlag(fl), a, b, c, d, tasks, cans)


@dataclass(slots=True)
class Event:
    """0 mcu_time_us u32 | 4 code u16 | 6 severity u8 | 7 source u8 | 8 value i32   共 12 字节"""
    TYPE: ClassVar[int] = MsgType.EVENT
    S: ClassVar[struct.Struct] = struct.Struct("<IHBBi")
    mcu_time_us: int = 0
    code: EventCode | int = EventCode.BOOT
    severity: Severity | int = Severity.INFO
    source: int = 0
    value: int = 0

    def encode(self) -> bytes:
        return self.S.pack(self.mcu_time_us & 0xFFFFFFFF, int(self.code), int(self.severity),
                           self.source, self.value)

    @classmethod
    def decode(cls, p: bytes) -> "Event":
        need(p, cls.S.size, "EVENT")
        t, c, s, src, v = cls.S.unpack_from(p)
        return cls(t, as_enum(EventCode, c), as_enum(Severity, s), src, v)
