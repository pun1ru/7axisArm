"""可注入的模拟设备：没有真实机械臂时测试通信层与 UI。

MockSerial 实现 serial.Serial 的最小子集（read / write / in_waiting / close / flush /
reset_input_buffer），内部连到一个 SimDevice。SimDevice 用协议字节流说话，并按
docs/02_protocol.md 第 7 节实现与固件相同的安全语义（有效期、链路超时、急停锁存、限幅），
所以通信层测试的就是真实的帧流。

故障注入：noise_prob（插入随机字节）、corrupt_prob（翻转一位）、drop_prob（丢字节）、
chunk_max（随机拆包大小），以及 unplug()/plug() 模拟拔线。
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass, field

from ..protocol import (Ack, AckResult, ArmPosture, ArmRole, ArmState, CanBusStatus, ClearTorque,
                        DeviceInfo, DeviceInfoReq, Estop, EstopAction, Event, EventCode, FrameParser,
                        ExchangeCommand, ExchangePhase, ExchangeState, ExchangeStateFlag, Heartbeat,
                        JointFlag, JointRecord, JointState, MotorStatus, ParamRead, ParamValue,
                        ParamWrite, SetHostTarget, SetIkTarget, SetJointTorque, Severity,
                        StatusFlag, SystemStatus, decode_frame, encode_frame)
from ..protocol.enums import ExchangeAction, HostIkAction, HostTargetAction, ParamType, PostureFlag
from ..protocol.params import BY_ID, W

try:
    from serial import SerialException
except ImportError:
    SerialException = OSError

# DH（module_init.c），index = row*4 + col
_DH = [(0.0, 0.0, 0.0626, 0.0), (90.0, 0.0, 0.0, 0.0), (90.0, 0.0, -0.148, 0.0),
       (90.0, 0.0, 0.0, 0.0), (90.0, 0.0, 0.1, 0.0), (90.0, 0.0, 0.0, 0.0), (90.0, 0.0, 0.0, 0.0)]
_ZERO = [4108, 19268, 6740, 21635, 26560, 32676, 65112]
_DIR = [1, 1, -1, 1, 1, 1, -1]


def _wrap_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


@dataclass
class FaultConfig:
    noise_prob: float = 0.0       # 每次输出时插入 1~8 个随机字节的概率
    corrupt_prob: float = 0.0     # 每帧随机翻转一位的概率
    drop_prob: float = 0.0        # 每帧随机丢一个字节的概率
    chunk_max: int = 0            # >0 时把输出按 1..chunk_max 随机拆块
    seed: int | None = None


@dataclass
class SimJoint:
    q: float = 0.0
    dq: float = 0.0
    tau_fb: float = 0.0
    host_tau: float = 0.0
    host_deadline: float = 0.0
    host_limit: float = 2.0       # 只读：模拟固件编译期常量（演示给 2 N*m；真机填 0 即禁止上位机出力）
    clamped: bool = False
    host_target: float = 0.0


class SimDevice:
    """模拟 STM32 固件的协议端。线程内按 state_rate_hz 产生 JOINT_STATE。"""

    HARD_LIMIT_NM = 5.0
    LINK_TIMEOUT_S = 0.3
    HOLD_DEFAULT_MS = 100
    HOLD_MAX_MS = 500

    def __init__(self, joint_count: int = 7, state_rate_hz: int = 200, status_rate_hz: int = 10,
                 faults: FaultConfig | None = None, motion: bool = True) -> None:
        self.n = joint_count
        self.state_rate_hz = state_rate_hz
        self.status_rate_hz = status_rate_hz
        self.faults = faults or FaultConfig()
        self.motion = motion
        self.rng = random.Random(self.faults.seed)
        self.joints = [SimJoint() for _ in range(joint_count)]
        self.arm_state = ArmState.ENABLE
        self.host_closed_loop_mask = 0
        self.host_target_valid_mask = 0
        self.ik_branch = 0
        self.ik_psi_deg = 0.0
        self.ik_auto_psi = False
        self.ik_cost_weights = (1.0,) * 7
        self.ik_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.exchange_phase = ExchangePhase.PARKED
        self.exchange_free = False
        self.exchange_difficulty = 1
        self.exchange_pose = [-200.0, 200.0, 800.0, 0.0, 90.0, 0.0]
        self.exchange_target = list(self.exchange_pose)
        self.exchange_insert_mm = 0.0
        self.exchange_lift_mm = 0.0
        self.exchange_p_deg = 0.0
        self.exchange_q_deg = 0.0
        self.exchange_q_target = 0.0
        self.exchange_seed = 0
        self.exchange_auto = False
        self.rc_online = True
        self.estop = False
        self.mute = False                 # True 时停止一切输出（模拟设备卡死）
        self.ignore_types: set[int] = set()   # 收到这些类型不回应（测超时）

        self._parser = FrameParser()
        self._out = bytearray()
        self._olock = threading.Lock()
        self._lock = threading.RLock()
        self._seq = 0
        self._t0 = time.monotonic()
        self._last_host_rx = 0.0
        self._link_ok = False
        self._last_cmd = 0.0
        self._acked: dict[int, bytes] = {}     # SEQ 去重：重发时回同样的应答
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.rx_frames_ok = 0
        self.received: list = []               # 收到的消息（测试用）
        self.gain = {k: [0.0] * joint_count for k in range(0x0200, 0x0206)}
        self.limit = {
            0x0102: [-90.0, -20.0, -175.0, -110.0, -175.0, -120.0, -175.0],
            0x0103: [90.0, 175.0, 175.0, 110.0, 175.0, 70.0, 175.0],
        }

    # ------------------------------------------------------------ 线程
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="sim-device", daemon=True)
        self._thread.start()
        self._emit(Event(self._us(), EventCode.BOOT, Severity.INFO, 0, 0))

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(1.0)

    def _us(self) -> int:
        return int((time.monotonic() - self._t0) * 1e6) & 0xFFFFFFFF

    def _loop(self) -> None:
        """按固定 tick 产生数据。操作系统休眠粒度粗（Windows 约 1~15 ms）时一次补齐落后的 tick，
        和真实 MCU 一样不丢采样，只是到达上位机时成批；时间戳按 tick 时刻而不是发送时刻。"""
        period = 1.0 / self.state_rate_hz
        status_div = max(1, round(self.state_rate_hz / self.status_rate_hz))
        k = 0
        tick = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            due = 0
            while tick <= now and due < 100:
                due += 1
                with self._lock:
                    self._safety(tick)
                    self._physics(period, tick)
                    self._emit(self._joint_state(tick))
                    if k % status_div == 0:
                        self._emit(self._posture_state(tick))
                        self._emit(self._exchange_state(tick))
                        self._emit(self._system_status(tick))
                k += 1
                tick += period
            if tick <= now:                     # 落后超过 100 个 tick，放弃追赶
                tick = now + period
            self._stop.wait(max(0.0, tick - time.monotonic()))

    # ------------------------------------------------------------ 安全语义（与固件约定一致）
    def _safety(self, now: float) -> None:
        link_ok = now - self._last_host_rx <= self.LINK_TIMEOUT_S
        if link_ok != self._link_ok:
            if not link_ok:                     # 先清设定再置标志，外部读到"断开"时设定必已清零
                for j in self.joints:
                    j.host_tau = 0.0
                    j.host_deadline = 0.0
                self.host_closed_loop_mask = 0
                if self.arm_state == ArmState.HOST_IK:
                    self.arm_state = ArmState.HOST
            self._link_ok = link_ok
            self._emit(Event(self._us(), EventCode.HOST_LINK_RESTORED if link_ok else EventCode.HOST_LINK_LOST,
                             Severity.INFO if link_ok else Severity.WARN, 0, 0))
        for i, j in enumerate(self.joints):
            if j.host_deadline and now > j.host_deadline:
                j.host_tau = 0.0
                j.host_deadline = 0.0
                self._emit(Event(self._us(), EventCode.TORQUE_EXPIRED, Severity.INFO, i, 0))

    def _host_tau_active(self, j: SimJoint, now: float) -> bool:
        return j.host_deadline > now and not self.estop

    def _physics(self, dt: float, now: float) -> None:
        t = now - self._t0
        for i, j in enumerate(self.joints):
            applied = j.host_tau if (self.arm_state == ArmState.ENABLE and self.rc_online
                                     and self._host_tau_active(j, now)) else 0.0
            host_cmd = 0.0
            if self.motion:
                base = 30.0 * math.sin(2 * math.pi * 0.2 * t + i * 0.7)
                dbase = 30.0 * 2 * math.pi * 0.2 * math.cos(2 * math.pi * 0.2 * t + i * 0.7)
            else:
                base = dbase = 0.0
            j.dq += (applied * 200.0 - j.dq * 2.0) * dt     # 外加力矩带来的附加运动
            extra = j.q - getattr(j, "_base", 0.0)
            extra += j.dq * dt
            loop_active = bool(self.host_closed_loop_mask & (1 << i))
            if self.arm_state in (ArmState.HOST, ArmState.HOST_IK) and loop_active and self.rc_online:
                error = j.host_target - j.q
                extra += max(-60.0 * dt, min(60.0 * dt, error))
                host_cmd = max(-j.host_limit, min(j.host_limit, error * 0.02))
            j._base = base                                    # type: ignore[attr-defined]
            j.q = max(-179.9, min(180.0, base + extra))
            j.tau_fb = applied + host_cmd + 0.02 * math.sin(t * 7 + i)
            j._dq_total = dbase + j.dq                        # type: ignore[attr-defined]
            j._applied = applied + host_cmd                   # type: ignore[attr-defined]

    def _flags(self, now: float) -> StatusFlag:
        f = StatusFlag.NONE
        if self.rc_online:
            f |= StatusFlag.RC_ONLINE
        if self._link_ok:
            f |= StatusFlag.HOST_LINK_OK
        if self.estop:
            f |= StatusFlag.ESTOP_LATCHED
        if any(self._host_tau_active(j, now) for j in self.joints):
            f |= StatusFlag.HOST_TORQUE_ACTIVE
        if self.arm_state in (ArmState.HOST, ArmState.HOST_IK) and self.host_closed_loop_mask:
            f |= StatusFlag.HOST_CLOSED_LOOP
        return f

    def _joint_state(self, now: float) -> JointState:
        recs = []
        for i, j in enumerate(self.joints):
            jf = JointFlag.ONLINE
            if self._host_tau_active(j, now):
                jf |= JointFlag.HOST_TORQUE_VALID
            if j.clamped:
                jf |= JointFlag.TORQUE_CLAMPED
            if self.arm_state in (ArmState.HOST, ArmState.HOST_IK) and self.host_target_valid_mask & (1 << i):
                jf |= JointFlag.HOST_TARGET_VALID
            if self.arm_state in (ArmState.HOST, ArmState.HOST_IK) and self.host_closed_loop_mask & (1 << i):
                jf |= JointFlag.HOST_CLOSED_LOOP
            enc = (_ZERO[i % 7] + int(_DIR[i % 7] * j.q / 360.0 * 65536)) & 0xFFFF
            recs.append(JointRecord(round(j.q, 4), getattr(j, "_dq_total", 0.0), j.tau_fb, j.q,
                                    getattr(j, "_applied", 0.0), enc,
                                    MotorStatus.ENABLED if self.arm_state in (
                                        ArmState.ENABLE, ArmState.HOST, ArmState.HOST_IK)
                                    else MotorStatus.DISABLED,
                                    0, 35 + i, jf, int(now * self.state_rate_hz) & 0xFFFF))
        age = min(0xFFFF, max(0, int((now - self._last_cmd) * 1000))) if self._last_cmd else 0xFFFF
        return JointState(self._tick_us(now), self.arm_state, self._flags(now), age, recs)

    def _posture_state(self, now: float) -> ArmPosture:
        angles = [j.q + _DH[i][3] for i, j in enumerate(self.joints)]
        branch = ((1 if math.sin(math.radians(angles[1])) < 0 else 0) |
                  (2 if math.sin(math.radians(angles[3])) < 0 else 0) |
                  (4 if math.sin(math.radians(angles[5])) < 0 else 0))
        flags = PostureFlag.VALID
        if self.arm_state == ArmState.HOST_IK:
            flags |= PostureFlag.IK_ACTIVE
            if self.ik_auto_psi:
                flags |= PostureFlag.AUTO_PSI
            branch = self.ik_branch
            pose = self.ik_pose
            psi = self.ik_psi_deg
        else:
            try:
                from ..kinematics.dh import DHTable, DHRow, euler_zyx_deg, forward_frames
                table = DHTable([DHRow(*row) for row in _DH])
                tf = forward_frames(table, [j.q for j in self.joints])[-1]
                pose = (float(tf[0, 3]), float(tf[1, 3]), float(tf[2, 3]), *euler_zyx_deg(tf[:3, :3]))
            except Exception:
                pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            psi = 0.0
        return ArmPosture(branch, flags, psi, *pose)

    def _tick_us(self, t: float) -> int:
        return int((t - self._t0) * 1e6) & 0xFFFFFFFF

    def _system_status(self, now: float) -> SystemStatus:
        return SystemStatus(self._tick_us(now), int((now - self._t0) * 1000), self.arm_state, ArmRole.MASTER,
                            1 if not self.rc_online else 3, 3 if self.arm_state == ArmState.ENABLE else 1,
                            self._flags(now), self.rx_frames_ok, self._parser.stats.crc_errors,
                            self._parser.stats.header_errors + self._parser.stats.length_errors, 0,
                            [0, 0, 0], [CanBusStatus(int(now * 1000), 0, 0, 0, 0) for _ in range(2)])

    def _exchange_state(self, now: float) -> ExchangeState:
        flags = ExchangeStateFlag.AVAILABLE
        if self.exchange_phase in (ExchangePhase.MOVING_POSE, ExchangePhase.INSERTING,
                                   ExchangePhase.LIFTING, ExchangePhase.ROTATING_P,
                                   ExchangePhase.ROTATING_Q, ExchangePhase.RESETTING):
            flags |= ExchangeStateFlag.MOVING
        if self.exchange_phase not in (ExchangePhase.PARKED, ExchangePhase.RESETTING):
            flags |= ExchangeStateFlag.POSE_VALID
        if self.exchange_auto:
            flags |= ExchangeStateFlag.AUTO_DEMO
        if self.exchange_phase == ExchangePhase.COMPLETE:
            flags |= ExchangeStateFlag.COMPLETE
        if self.exchange_phase in (ExchangePhase.Q_READY, ExchangePhase.COMPLETE):
            flags |= ExchangeStateFlag.Q_READY
        if self.exchange_free:
            flags |= ExchangeStateFlag.FREE
        return ExchangeState(
            self.exchange_phase, self.exchange_difficulty, flags, now - self._t0,
            *self.exchange_pose, self.exchange_insert_mm, self.exchange_lift_mm,
            self.exchange_p_deg, self.exchange_q_deg, self.exchange_q_target,
            *self.exchange_target, self.exchange_seed)

    # ------------------------------------------------------------ 输出
    def _emit(self, msg) -> None:
        if self.mute:
            return
        frame = bytearray(encode_frame(msg.TYPE, self._seq, msg.encode()))
        self._seq = (self._seq + 1) & 0xFFFF
        self._push(frame)

    def _push(self, frame: bytes | bytearray) -> None:
        f = self.faults
        frame = bytearray(frame)
        if f.corrupt_prob and self.rng.random() < f.corrupt_prob:
            k = self.rng.randrange(len(frame))
            frame[k] ^= 1 << self.rng.randrange(8)
        if f.drop_prob and self.rng.random() < f.drop_prob:
            del frame[self.rng.randrange(len(frame))]
        if f.noise_prob and self.rng.random() < f.noise_prob:
            frame[0:0] = bytes(self.rng.randrange(256) for _ in range(self.rng.randint(1, 8)))
        with self._olock:
            self._out += frame

    def take_output(self, n: int) -> bytes:
        with self._olock:
            if self.faults.chunk_max and self._out:
                n = min(n, self.rng.randint(1, self.faults.chunk_max))
            data = bytes(self._out[:n])
            del self._out[:n]
            return data

    def pending_output(self) -> int:
        with self._olock:
            return len(self._out)

    # ------------------------------------------------------------ 输入
    def feed(self, data: bytes) -> None:
        for fr in self._parser.feed(data):
            with self._lock:
                self.rx_frames_ok += 1
                self._last_host_rx = time.monotonic()
                if fr.seq in self._acked:                  # 重发：回同样应答，不重复执行
                    self._push(self._acked[fr.seq])
                    continue
                try:
                    msg = decode_frame(fr)
                except Exception:
                    continue
                self.received.append(msg)
                if fr.msg_type in self.ignore_types:
                    continue
                reply = self._handle(msg, fr.seq)
                if reply is not None:
                    frame = encode_frame(reply.TYPE, self._seq, reply.encode())
                    self._seq = (self._seq + 1) & 0xFFFF
                    if len(self._acked) > 64:
                        self._acked.pop(next(iter(self._acked)))
                    self._acked[fr.seq] = frame
                    if not self.mute:
                        self._push(frame)

    def _handle(self, m, seq: int):
        now = time.monotonic()
        if isinstance(m, Heartbeat):
            return None
        if isinstance(m, DeviceInfoReq):
            return DeviceInfo(0x10, self.n, JointRecord.S.size, ArmRole.MASTER, 256,
                              self.state_rate_hz, self.status_rate_hz, int(self.LINK_TIMEOUT_S * 1000),
                              self.HOLD_MAX_MS, 1, "sim-device")
        if isinstance(m, SetJointTorque):
            if self.estop:
                return Ack(seq, m.TYPE, AckResult.REJECT_ESTOP, 0)
            if any(not 0 <= it.joint < self.n or not math.isfinite(it.torque_nm) for it in m.items):
                return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
            hold = m.hold_ms or self.HOLD_DEFAULT_MS
            hold = min(hold, self.HOLD_MAX_MS) / 1000.0
            mask = 0
            for it in m.items:
                j = self.joints[it.joint]
                lim = min(j.host_limit, self.HARD_LIMIT_NM)
                tau = max(-lim, min(lim, it.torque_nm))
                j.clamped = tau != it.torque_nm
                if j.clamped:
                    mask |= 1 << it.joint
                j.host_tau = tau
                j.host_deadline = now + hold
            self._last_cmd = now
            return Ack(seq, m.TYPE, AckResult.OK_CLAMPED if mask else AckResult.OK, mask)
        if isinstance(m, SetHostTarget):
            if m.action == HostTargetAction.STOP_ALL:
                self.host_closed_loop_mask = 0
                if self.arm_state == ArmState.HOST_IK:
                    self.arm_state = ArmState.HOST
                self._last_cmd = now
                return Ack(seq, m.TYPE, AckResult.OK, 0)
            if self.arm_state != ArmState.HOST or m.action not in (
                    HostTargetAction.SET_OPEN, HostTargetAction.SET_CLOSED):
                return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
            if not 0 <= m.joint < self.n or not math.isfinite(m.target_deg):
                return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
            joint = self.joints[m.joint]
            abs_deg = _wrap_deg(joint.q)
            limit_min_abs = self.limit[0x0102][m.joint]
            limit_max_abs = self.limit[0x0103][m.joint]
            limit_min_multi = joint.q + (limit_min_abs - abs_deg)
            limit_max_multi = joint.q + (limit_max_abs - abs_deg)
            if m.multi_turn:
                target_deg = float(m.target_deg)
                if not limit_min_multi <= target_deg <= limit_max_multi:
                    return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
            else:
                if not limit_min_abs <= m.target_deg <= limit_max_abs:
                    return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
                target_deg = joint.q + (m.target_deg - abs_deg)
                target_deg = max(limit_min_multi, min(limit_max_multi, target_deg))
            joint.host_target = target_deg
            self.host_target_valid_mask |= 1 << m.joint
            if m.action == HostTargetAction.SET_CLOSED:
                self.host_closed_loop_mask |= 1 << m.joint
            else:
                self.host_closed_loop_mask &= ~(1 << m.joint)
            self._last_cmd = now
            return Ack(seq, m.TYPE, AckResult.OK, 0)
        if isinstance(m, SetIkTarget):
            if m.action == HostIkAction.STOP:
                self.host_closed_loop_mask = 0
                if self.arm_state == ArmState.HOST_IK:
                    self.arm_state = ArmState.HOST
                self._last_cmd = now
                return Ack(seq, m.TYPE, AckResult.OK, 0)
            if self.arm_state not in (ArmState.HOST, ArmState.HOST_IK):
                return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
            values = (m.x, m.y, m.z, m.roll, m.pitch, m.yaw, m.psi_deg)
            if not 0 <= m.branch < 8 or not all(math.isfinite(v) for v in values):
                return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
            if not all(math.isfinite(v) and v >= 0.0 for v in m.cost_weights) \
                    or (m.auto_psi and sum(m.cost_weights) <= 0.0):
                return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
            if self.arm_state == ArmState.HOST:
                for i, joint in enumerate(self.joints):
                    joint.host_target = joint.q
                    self.host_target_valid_mask |= 1 << i
            self.arm_state = ArmState.HOST_IK
            self.host_closed_loop_mask = (1 << self.n) - 1
            self.ik_branch = m.branch
            self.ik_pose = values[:6]
            self.ik_psi_deg = m.psi_deg
            self.ik_auto_psi = bool(m.auto_psi)
            self.ik_cost_weights = tuple(m.cost_weights)
            self._last_cmd = now
            return Ack(seq, m.TYPE, AckResult.OK, 0)
        if isinstance(m, ClearTorque):
            for i, j in enumerate(self.joints):
                if m.joint_mask >> i & 1:
                    j.host_tau = 0.0
                    j.host_deadline = 0.0
            return Ack(seq, m.TYPE, AckResult.OK, 0)
        if isinstance(m, Estop):
            latch = m.action == EstopAction.LATCH
            if latch:
                for j in self.joints:
                    j.host_tau = 0.0
                    j.host_deadline = 0.0
            if latch != self.estop:
                self.estop = latch
                self._emit(Event(self._us(), EventCode.ESTOP_LATCHED if latch else EventCode.ESTOP_RELEASED,
                                 Severity.WARN if latch else Severity.INFO, 0, 0))
            return Ack(seq, m.TYPE, AckResult.OK, 0)
        if isinstance(m, ExchangeCommand):
            action = ExchangeAction(m.action)
            if action == ExchangeAction.RESET:
                self.exchange_free = False
                self.exchange_phase = ExchangePhase.PARKED
                self.exchange_pose = [-200.0, 200.0, 800.0, 0.0, 90.0, 0.0]
                self.exchange_target = list(self.exchange_pose)
                self.exchange_insert_mm = 0.0
                self.exchange_lift_mm = 0.0
                self.exchange_p_deg = 0.0
                self.exchange_q_deg = 0.0
                self.exchange_auto = False
            elif action in (ExchangeAction.RANDOMIZE, ExchangeAction.MOVE_TO_POSE):
                if action == ExchangeAction.MOVE_TO_POSE and m.use_pose:
                    self.exchange_target = [m.x_mm, m.y_mm, m.z_mm,
                                            m.theta_deg, m.phi_deg, m.alpha_deg]
                else:
                    self.exchange_seed = m.seed or 1
                    rng = random.Random(self.exchange_seed)
                    self.exchange_target = [rng.uniform(-100.0, 0.0), rng.uniform(100.0, 300.0),
                                            rng.uniform(500.0, 700.0),
                                            0.0 if m.difficulty == 1 else rng.uniform(-90.0, 90.0),
                                            90.0 if m.difficulty == 1 else rng.uniform(0.0, 90.0),
                                            0.0 if m.difficulty == 1 else rng.uniform(-45.0, 45.0)]
                self.exchange_difficulty = m.difficulty
                self.exchange_pose = list(self.exchange_target)
                self.exchange_insert_mm = self.exchange_lift_mm = 0.0
                self.exchange_p_deg = self.exchange_q_deg = 0.0
                self.exchange_q_target = self.exchange_pose[5]
                self.exchange_phase = ExchangePhase.READY
            elif action == ExchangeAction.INSERT and self.exchange_phase == ExchangePhase.READY:
                self.exchange_insert_mm = -100.0
                self.exchange_phase = ExchangePhase.INSERTED
            elif action == ExchangeAction.LIFT and self.exchange_phase == ExchangePhase.INSERTED:
                self.exchange_lift_mm = 100.0
                self.exchange_phase = ExchangePhase.LIFTED
            elif action == ExchangeAction.ROTATE_P and self.exchange_phase == ExchangePhase.LIFTED:
                self.exchange_p_deg = 90.0
                self.exchange_phase = ExchangePhase.P_ROTATED
            elif action == ExchangeAction.ROTATE_Q and self.exchange_phase == ExchangePhase.P_ROTATED:
                self.exchange_q_target = m.q_target_deg
                self.exchange_q_deg = m.q_target_deg
                self.exchange_phase = ExchangePhase.Q_READY
            elif action == ExchangeAction.CONFIRM and self.exchange_phase == (
                    ExchangePhase.INSERTED if self.exchange_difficulty == 1 else
                    ExchangePhase.LIFTED if self.exchange_difficulty == 2 else
                    ExchangePhase.Q_READY):   # 确认点随难度变化，与 ArmSim 一致
                self.exchange_phase = ExchangePhase.COMPLETE
                self.exchange_auto = False
            elif action == ExchangeAction.AUTO_DEMO:
                self.exchange_auto = True
                if self.exchange_phase == ExchangePhase.PARKED:
                    self.exchange_phase = ExchangePhase.READY
            elif action == ExchangeAction.STOP_AUTO:
                self.exchange_auto = False
            elif action == ExchangeAction.FREE:
                self.exchange_free = True
                self.exchange_auto = False
                self.exchange_phase = ExchangePhase.PARKED
            else:
                return Ack(seq, m.TYPE, AckResult.BAD_PARAM, 0)
            return Ack(seq, m.TYPE, AckResult.OK, 0)
        if isinstance(m, (ParamRead, ParamWrite)):
            return self._param(m, seq)
        return Ack(seq, getattr(m, "TYPE", 0), AckResult.UNSUPPORTED, 0)

    def _param(self, m, seq: int) -> ParamValue:
        d = BY_ID.get(m.param_id)
        if d is None or m.index >= d.count:
            return ParamValue(seq, m.param_id, m.index, ParamType.U32, AckResult.BAD_PARAM, 0, 0)
        store = self._param_store(m.param_id)
        if isinstance(m, ParamWrite):
            if not d.access & W:
                return ParamValue(seq, m.param_id, m.index, d.ptype, AckResult.READ_ONLY, d.access,
                                  store[m.index])
            store[m.index] = m.value
        return ParamValue(seq, m.param_id, m.index, d.ptype, AckResult.OK, d.access, store[m.index])

    def _param_store(self, pid: int) -> list:
        if pid == 0x0001:
            return _Proxy(self, "state_rate_hz")
        if pid == 0x0002:
            return _Proxy(self, "status_rate_hz")
        if pid == 0x0003:
            return [int(self.LINK_TIMEOUT_S * 1000)]
        if pid == 0x0004:
            return [self.HOLD_MAX_MS]
        if pid == 0x0100:
            return (_ZERO * 2)[: self.n]
        if pid == 0x0101:
            return (_DIR * 2)[: self.n]
        if pid in self.limit:
            return self.limit[pid]
        if pid in self.gain:
            return self.gain[pid]
        if pid == 0x0300:
            return [j.host_limit for j in self.joints]
        if pid == 0x0400:
            return [v for row in _DH for v in row]
        return [0] * 32


class _Proxy(list):
    def __init__(self, obj, attr):
        super().__init__([getattr(obj, attr)])
        self._o, self._a = obj, attr

    def __setitem__(self, i, v):
        setattr(self._o, self._a, v)
        super().__setitem__(i, v)


# ====================================================================== 虚拟串口
class MockSerial:
    """serial.Serial 的最小替身，连接到 SimDevice。"""

    def __init__(self, device: SimDevice, timeout: float = 0.02, **_: object) -> None:
        self.device = device
        self.timeout = timeout
        self.is_open = True
        self._broken = False

    @property
    def in_waiting(self) -> int:
        self._check()
        return self.device.pending_output()

    def _check(self) -> None:
        if self._broken or not self.is_open:
            raise SerialException("mock port disconnected")

    def read(self, size: int = 1) -> bytes:
        deadline = time.monotonic() + (self.timeout or 0)
        while True:
            self._check()
            data = self.device.take_output(size)
            if data or time.monotonic() >= deadline:
                return data
            time.sleep(0.001)

    def write(self, data: bytes) -> int:
        self._check()
        self.device.feed(bytes(data))
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False

    def break_link(self) -> None:
        self._broken = True


class MockSerialFactory:
    """注入给 SerialTransport 的 serial_factory。unplug()/plug() 模拟拔插线。"""

    def __init__(self, device: SimDevice) -> None:
        self.device = device
        self.available = True
        self.current: MockSerial | None = None
        self.open_count = 0

    def __call__(self, **kw) -> MockSerial:
        if not self.available:
            raise SerialException(f"could not open port {kw.get('port')}")
        self.open_count += 1
        self.current = MockSerial(self.device, timeout=kw.get("timeout", 0.02))
        return self.current

    def unplug(self) -> None:
        self.available = False
        if self.current:
            self.current.break_link()

    def plug(self) -> None:
        self.available = True


def make_mock_transport(device: SimDevice | None = None, **cfg):
    """便捷构造：返回 (transport, device, factory)。"""
    from .transport import SerialConfig, SerialTransport
    device = device or SimDevice()
    factory = MockSerialFactory(device)
    conf = SerialConfig(port="MOCK", reconnect_backoff_s=(0.05, 0.1), **cfg)
    return SerialTransport(conf, serial_factory=factory), device, factory
