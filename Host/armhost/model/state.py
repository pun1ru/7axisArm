"""统一的机器人状态模型与线程安全环形缓存。

时间有两条线，严格区分：
  mcu_time_us  —— MCU 采样时刻（JOINT_STATE 帧里的时间戳，已展开为 64 位）
  host_time_ns —— 上位机收到该帧的 time.monotonic_ns()
曲线默认按 MCU 时间画（反映真实采样间隔），接收频率、到达抖动按上位机时间统计。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field

import numpy as np

from ..protocol.enums import ArmState, StatusFlag
from ..protocol.msg_report import JointRecord


@dataclass(frozen=True, slots=True)
class RobotSample:
    mcu_time_us: int
    host_time_ns: int
    seq: int
    arm_state: ArmState | int
    status_flags: StatusFlag | int
    host_cmd_age_ms: int
    joints: tuple[JointRecord, ...]


@dataclass(frozen=True, slots=True)
class Quantity:
    key: str            # JointRecord 字段名
    label: str
    unit: str
    group: str          # 同 group 的画在同一个图里


QUANTITIES: tuple[Quantity, ...] = (
    Quantity("angle_deg", "角度", "deg", "angle"),
    Quantity("target_deg", "目标角", "deg", "angle"),
    Quantity("velocity_dps", "角速度", "deg/s", "velocity"),
    Quantity("torque_nm", "反馈力矩", "N·m", "torque"),
    Quantity("torque_cmd_nm", "指令力矩", "N·m", "torque"),
)
QKEYS = tuple(q.key for q in QUANTITIES)
GROUPS = ("angle", "velocity", "torque")
GROUP_TITLE = {"angle": "角度 (deg)", "velocity": "角速度 (deg/s)", "torque": "力矩 (N·m)"}

TIME_MCU = "mcu"
TIME_HOST = "host"


@dataclass
class BufferView:
    """某一时刻缓存的只读拷贝（按时间先后排列）"""
    mcu_s: np.ndarray           # MCU 时间 s（相对缓存起点）
    host_s: np.ndarray          # 上位机接收时间 s（相对缓存起点）
    data: np.ndarray            # shape (n, joint_count, len(QUANTITIES))
    joint_count: int

    def __len__(self) -> int:
        return len(self.mcu_s)

    def channel(self, joint: int, qkey: str) -> np.ndarray:
        return self.data[:, joint, QKEYS.index(qkey)]

    def time(self, base: str) -> np.ndarray:
        return self.mcu_s if base == TIME_MCU else self.host_s


class SampleBuffer:
    """固定容量的 numpy 环形缓存。写：串口读线程；读：GUI 定时器。

    MCU 时间倒退超过 0.5 s 视为设备重启，自动清空，避免曲线折回。
    """

    def __init__(self, capacity: int = 30000, joint_count: int = 7) -> None:
        self.capacity = capacity
        self._lock = threading.Lock()
        self._alloc(joint_count)
        self.restarts = 0
        self.total = 0

    def _alloc(self, joint_count: int) -> None:
        self.joint_count = joint_count
        self._mcu = np.zeros(self.capacity, np.float64)
        self._host = np.zeros(self.capacity, np.float64)
        self._data = np.zeros((self.capacity, joint_count, len(QKEYS)), np.float32)
        self._head = 0
        self._n = 0
        self._t0_mcu: int | None = None
        self._t0_host: int | None = None
        self._last_mcu: int | None = None

    def clear(self) -> None:
        with self._lock:
            self._alloc(self.joint_count)

    @property
    def size(self) -> int:
        return self._n

    def append(self, s: RobotSample) -> None:
        n = len(s.joints)
        row = np.fromiter((getattr(j, k) for j in s.joints for k in QKEYS),
                          np.float32, n * len(QKEYS)).reshape(n, len(QKEYS))
        with self._lock:
            if n != self.joint_count:
                self._alloc(n)
            if self._last_mcu is not None and s.mcu_time_us + 500_000 < self._last_mcu:
                self._alloc(n)
                self.restarts += 1
            if self._t0_mcu is None:
                self._t0_mcu = s.mcu_time_us
                self._t0_host = s.host_time_ns
            self._last_mcu = s.mcu_time_us
            i = self._head
            self._mcu[i] = (s.mcu_time_us - self._t0_mcu) * 1e-6
            self._host[i] = (s.host_time_ns - self._t0_host) * 1e-9
            self._data[i] = row
            self._head = (i + 1) % self.capacity
            self._n = min(self._n + 1, self.capacity)
            self.total += 1

    def view(self, last_s: float | None = None, base: str = TIME_MCU) -> BufferView:
        """拷贝出按时间排序的数据；last_s 只取最近若干秒（按 base 时间轴）。"""
        with self._lock:
            n, h = self._n, self._head
            if n < self.capacity:
                idx = slice(0, n)
                mcu, host, data = self._mcu[idx].copy(), self._host[idx].copy(), self._data[idx].copy()
            else:
                mcu = np.concatenate((self._mcu[h:], self._mcu[:h]))
                host = np.concatenate((self._host[h:], self._host[:h]))
                data = np.concatenate((self._data[h:], self._data[:h]))
            jc = self.joint_count
        if last_s is not None and n:
            t = mcu if base == TIME_MCU else host
            k = int(np.searchsorted(t, t[-1] - last_s, side="left"))
            mcu, host, data = mcu[k:], host[k:], data[k:]
        return BufferView(mcu, host, data, jc)


@dataclass
class TimingStats:
    """区分 MCU 采样节奏与上位机到达节奏"""
    samples: int = 0
    mcu_rate_hz: float = 0.0        # 由 MCU 时间戳间隔得到的采样频率
    mcu_dt_max_ms: float = 0.0      # 最大采样间隔（发现 MCU 侧丢样 / 卡顿）
    host_rate_hz: float = 0.0       # 上位机实际收到的频率
    host_gap_max_ms: float = 0.0    # 最长到达间隔（USB/串口成批到达时会变大）
    jitter_ms: float = 0.0          # (host - mcu) 的标准差：传输延迟的抖动，不是绝对延迟


def timing_stats(v: BufferView, window_s: float = 2.0) -> TimingStats:
    if len(v) < 3:
        return TimingStats(samples=len(v))
    k = int(np.searchsorted(v.mcu_s, v.mcu_s[-1] - window_s))
    m, h = v.mcu_s[k:], v.host_s[k:]
    if len(m) < 3:
        return TimingStats(samples=len(v))
    dm, dh = np.diff(m), np.diff(h)
    span_m, span_h = m[-1] - m[0], h[-1] - h[0]
    off = h - m
    return TimingStats(len(v), (len(m) - 1) / span_m if span_m > 0 else 0.0, float(dm.max() * 1e3),
                       (len(h) - 1) / span_h if span_h > 0 else 0.0, float(dh.max() * 1e3),
                       float(np.std(off) * 1e3))


@dataclass
class ChannelStats:
    name: str
    unit: str
    n: int
    min: float
    max: float
    mean: float
    std: float
    rms: float


def channel_stats(v: BufferView, channels: list[tuple[int, str]],
                  names: dict[str, Quantity] | None = None) -> list[ChannelStats]:
    names = names or {q.key: q for q in QUANTITIES}
    out = []
    for j, k in channels:
        x = v.channel(j, k).astype(np.float64)
        q = names[k]
        if len(x) == 0:
            out.append(ChannelStats(f"J{j} {q.label}", q.unit, 0, *(float("nan"),) * 5))
            continue
        out.append(ChannelStats(f"J{j} {q.label}", q.unit, len(x), float(x.min()), float(x.max()),
                                float(x.mean()), float(x.std()), float(np.sqrt(np.mean(x * x)))))
    return out
