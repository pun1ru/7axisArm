"""与单片机工程同名同义的镜像结构体（README #3 第 4 条）。

字段名、单位、枚举值对照：
  Feedback_t      (LibSE/Drivers/Motor/Motor.h)
  ArmObserver_t   (LibSE/Module/Arm/arm.h)
  ArmJointCtrl_t  (LibSE/Module/Arm/arm.h)，pid 内部状态不上报
  Arm_t           (LibSE/Module/Arm/arm.h)

注意：协议里的角速度/力矩已经是关节口径（按 direction 折算），与 Feedback_t 里电机口径的
omega_rads / torque_nm 符号可能相反；这里保留关节口径，并在字段注释里写清楚。
固件改了这些结构体时，同步改这里与 msg_report.JointRecord。

线程：update() 在串口读线程中调用，读方用 snapshot() 拿深拷贝。
"""

from __future__ import annotations

import copy
import math
import threading
from dataclasses import dataclass, field

from ..protocol.enums import ArmRole, ArmState, JointFlag, MotorStatus, StatusFlag
from ..protocol.msg_link import DeviceInfo
from ..protocol.msg_report import Event, JointState, SystemStatus
from .stats import McuClock

JOINT_NAMES = ("J0", "J1", "J2", "J3", "J4", "J5", "J6")
DEG2RAD = math.pi / 180.0


@dataclass
class Feedback_t:  # noqa: N801  与 C 同名
    timestamp: int = 0              # 反馈帧计数（低 16 位）
    error_id: int = 0
    status: MotorStatus | int = MotorStatus.NO_FEEDBACK
    encoder: int = 0
    abs_angle_deg: float = 0.0      # 关节口径 deg
    abs_angle_rad: float = 0.0
    omega_rads: float = 0.0         # 关节口径 rad/s
    torque_nm: float = 0.0          # 关节口径 N*m
    temp: float = 0.0


@dataclass
class ArmJointCtrl_t:  # noqa: N801
    target_deg: float = 0.0
    output: float = 0.0             # 本周期力矩指令 N*m


@dataclass
class ArmObserver_t:  # noqa: N801
    joint_deg: list[float] = field(default_factory=lambda: [0.0] * 7)


@dataclass
class Arm_t:  # noqa: N801
    state: ArmState | int = ArmState.PROTECTED
    role: ArmRole | int = ArmRole.MASTER
    observer: ArmObserver_t = field(default_factory=ArmObserver_t)
    joint_ctrl: list[ArmJointCtrl_t] = field(default_factory=lambda: [ArmJointCtrl_t() for _ in range(7)])
    feedback: list[Feedback_t] = field(default_factory=lambda: [Feedback_t() for _ in range(7)])
    # 以下为协议附加信息，不在固件 Arm_t 里
    joint_flags: list[JointFlag] = field(default_factory=lambda: [JointFlag.NONE] * 7)
    status_flags: StatusFlag = StatusFlag.NONE
    host_cmd_age_ms: int = 0xFFFF
    mcu_time_us: int = 0            # 已展开为 64 位
    host_time_ns: int = 0


@dataclass
class ArmMirror:
    arm: Arm_t = field(default_factory=Arm_t)
    system: SystemStatus | None = None
    device: DeviceInfo | None = None
    events: list[Event] = field(default_factory=list)
    max_events: int = 200
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _clock: McuClock = field(default_factory=McuClock, repr=False)

    def reset_clock(self) -> None:
        with self._lock:
            self._clock.reset()

    def update(self, msg, host_time_ns: int) -> None:
        with self._lock:
            if isinstance(msg, JointState):
                self._apply_joint_state(msg, host_time_ns)
            elif isinstance(msg, SystemStatus):
                self.system = msg
                self.arm.state = msg.arm_state
                self.arm.role = msg.arm_role
            elif isinstance(msg, DeviceInfo):
                self.device = msg
                self.arm.role = msg.arm_role
            elif isinstance(msg, Event):
                self.events.append(msg)
                if len(self.events) > self.max_events:
                    del self.events[: len(self.events) - self.max_events]

    def _apply_joint_state(self, m: JointState, host_time_ns: int) -> None:
        a = self.arm
        n = len(m.joints)
        if len(a.feedback) != n:                      # 关节数以设备上报为准
            a.observer.joint_deg = [0.0] * n
            a.joint_ctrl = [ArmJointCtrl_t() for _ in range(n)]
            a.feedback = [Feedback_t() for _ in range(n)]
            a.joint_flags = [JointFlag.NONE] * n
        a.state = m.arm_state
        a.status_flags = StatusFlag(m.status_flags)
        a.host_cmd_age_ms = m.host_cmd_age_ms
        a.mcu_time_us = self._clock.unwrap(m.mcu_time_us)
        a.host_time_ns = host_time_ns
        for i, r in enumerate(m.joints):
            a.observer.joint_deg[i] = r.angle_deg
            c = a.joint_ctrl[i]
            c.target_deg = r.target_deg
            c.output = r.torque_cmd_nm
            f = a.feedback[i]
            f.timestamp = r.feedback_count
            f.error_id = r.error_id
            f.status = r.motor_status
            f.encoder = r.encoder
            f.abs_angle_deg = r.angle_deg
            f.abs_angle_rad = r.angle_deg * DEG2RAD
            f.omega_rads = r.velocity_dps * DEG2RAD
            f.torque_nm = r.torque_nm
            f.temp = float(r.temperature_c)
            a.joint_flags[i] = JointFlag(r.joint_flags)

    def snapshot(self) -> "ArmMirror":
        with self._lock:
            return ArmMirror(copy.deepcopy(self.arm), self.system, self.device, list(self.events))

    def format_c(self) -> str:
        """按固件变量名打印，便于和 Keil Watch 窗口逐项对照。"""
        s = self.snapshot()
        a = s.arm
        st = a.state.name if isinstance(a.state, ArmState) else a.state
        lines = [f"armInstance.state = ARM_{st}",
                 f"/* mcu_time_us={a.mcu_time_us} flags={a.status_flags!r} host_cmd_age_ms={a.host_cmd_age_ms} */"]
        for i, (q, c, f) in enumerate(zip(a.observer.joint_deg, a.joint_ctrl, a.feedback)):
            name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"J{i}"
            ms = f.status.name if isinstance(f.status, MotorStatus) else f.status
            lines.append(
                f"[{name}] observer.joint_deg={q:9.3f}  joint_ctrl.target_deg={c.target_deg:9.3f} "
                f"output={c.output:7.3f}  | feedback.status=MOTOR_{ms} error_id={f.error_id} "
                f"encoder={f.encoder} omega_rads={f.omega_rads:8.3f} torque_nm={f.torque_nm:7.3f} "
                f"temp={f.temp:.0f} timestamp={f.timestamp}")
        return "\n".join(lines)
