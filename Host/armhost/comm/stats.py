"""接收 / 发送统计、序号跟踪、MCU 时间戳展开。全部线程安全。"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field


class SeqTracker:
    """跟踪对端某一类消息的 u16 序号，统计丢帧 / 重复 / 乱序。

    diff = (seq - last) mod 65536
      diff == 1            正常
      2 <= diff < 32768    丢了 diff-1 帧
      diff == 0            重复
      diff >= 32768        乱序或对端重启（视为重启，重新同步，不计丢帧）
    """

    def __init__(self) -> None:
        self.last: int | None = None
        self.lost = 0
        self.duplicate = 0
        self.resync = 0

    def update(self, seq: int) -> int:
        """返回本次判定丢失的帧数"""
        if self.last is None:
            self.last = seq
            return 0
        diff = (seq - self.last) & 0xFFFF
        if diff == 0:
            self.duplicate += 1
            return 0
        if diff >= 0x8000:
            self.resync += 1
            self.last = seq
            return 0
        self.last = seq
        self.lost += diff - 1
        return diff - 1

    def reset(self) -> None:
        self.last = None


class SeqCounter:
    def __init__(self, start: int = 0) -> None:
        self._v = start & 0xFFFF
        self._lock = threading.Lock()

    def next(self) -> int:
        with self._lock:
            v = self._v
            self._v = (v + 1) & 0xFFFF
            return v


class McuClock:
    """把 32 位 µs 回绕计数展开成 64 位，便于画图与记录。"""

    def __init__(self) -> None:
        self._last: int | None = None
        self._epoch = 0

    def unwrap(self, t32: int) -> int:
        if self._last is not None and t32 < self._last and self._last - t32 > 0x80000000:
            self._epoch += 1 << 32
        self._last = t32
        return self._epoch + t32

    def reset(self) -> None:
        self._last = None
        self._epoch = 0


class RateMeter:
    """滑动窗口频率估计"""

    def __init__(self, window_s: float = 1.0) -> None:
        self.window = window_s
        self._t: deque[float] = deque()

    def tick(self, now: float) -> None:
        self._t.append(now)
        lim = now - self.window
        while self._t and self._t[0] < lim:
            self._t.popleft()

    def rate(self, now: float) -> float:
        lim = now - self.window
        while self._t and self._t[0] < lim:
            self._t.popleft()
        return len(self._t) / self.window


@dataclass
class LinkStats:
    rx_bytes: int = 0
    tx_bytes: int = 0
    rx_frames: int = 0
    tx_frames: int = 0
    tx_dropped: int = 0           # 断线期间被丢弃的待发帧
    tx_errors: int = 0
    decode_errors: int = 0        # CRC 正确但负载长度不对
    lost_frames: int = 0          # 按序号推算的丢帧
    duplicate_frames: int = 0
    ack_timeouts: int = 0
    reconnects: int = 0
    rx_by_type: dict[int, int] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _rate: dict[int, RateMeter] = field(default_factory=dict, repr=False)

    def on_rx(self, msg_type: int, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self.lock:
            self.rx_frames += 1
            self.rx_by_type[msg_type] = self.rx_by_type.get(msg_type, 0) + 1
            self._rate.setdefault(msg_type, RateMeter()).tick(now)

    def add(self, **kw: int) -> None:
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, getattr(self, k) + v)

    def snapshot(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        with self.lock:
            d = {k: v for k, v in self.__dict__.items()
                 if not k.startswith("_") and k != "lock"}
            d["rx_by_type"] = dict(self.rx_by_type)
            d["rate_hz"] = {t: m.rate(now) for t, m in self._rate.items()}
        return d
