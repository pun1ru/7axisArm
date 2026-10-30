"""监视界面的控制器：持有后端、缓存、记录器，桥接后端线程与 GUI。不依赖 Qt。

后端回调（串口读线程）里只做三件事：写环形缓存、记最新样本、非阻塞入记录队列。
事件放进 deque，由 GUI 定时器取走。GUI 不直接接触串口对象。
"""

from __future__ import annotations

import logging
import threading
from collections import deque
from datetime import datetime
from typing import Any

from .model import RobotSample, SampleBuffer
from .model.backend import RobotBackend
from .model.recorder import CsvRecorder
from .protocol import Event, EventCode, Severity
from .protocol.enums import ArmState

log = logging.getLogger("armhost.controller")


class MonitorController:
    def __init__(self, buffer_capacity: int = 30000, joint_count: int = 7) -> None:
        self.buffer = SampleBuffer(buffer_capacity, joint_count)
        self.recorder = CsvRecorder()
        self.backend: RobotBackend | None = None
        self._latest: RobotSample | None = None
        self._lock = threading.Lock()
        self._events: deque[tuple[str, str, Any]] = deque(maxlen=1000)
        self._gen = 0                     # 后端代号：旧后端残留回调直接丢弃

    # ---------------------------------------------------------------- 后端
    def set_backend(self, backend: RobotBackend | None) -> None:
        old = self.backend
        self._gen += 1
        gen = self._gen
        self.backend = None
        if old is not None:
            try:
                old.stop()            # 串口后端在这里发急停 + 清零
            except Exception:
                log.exception("stop backend failed")
        self.buffer.clear()
        with self._lock:
            self._latest = None
        if backend is None:
            return
        backend.add_sample_listener(lambda s: self._on_sample(gen, s))
        backend.add_event_listener(lambda k, p: self._on_event(gen, k, p))
        self.backend = backend
        self._push("info", f"切换数据源：{backend.name}")
        backend.start()

    def shutdown(self) -> None:
        self.recorder.stop()
        self.set_backend(None)

    # ---------------------------------------------------------------- 后端线程回调
    def _on_sample(self, gen: int, s: RobotSample) -> None:
        if gen != self._gen:
            return
        self.buffer.append(s)
        with self._lock:
            self._latest = s
        self.recorder.put(s)

    def _on_event(self, gen: int, kind: str, payload: Any) -> None:
        if gen != self._gen or kind == "system":
            return
        if kind == "replay_seek":
            self.buffer.clear()
        self._push(kind, self._describe(kind, payload), payload)

    def _push(self, kind: str, text: str, payload: Any = None) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._events.append((kind, f"{ts}  {text}", payload))

    @staticmethod
    def _describe(kind: str, p: Any) -> str:
        if kind == "event" and isinstance(p, Event):
            try:
                code = EventCode(p.code).name
            except ValueError:
                code = f"0x{p.code:04X}"
            try:
                sev = Severity(p.severity).name
            except ValueError:
                sev = str(p.severity)
            extra = ""
            if p.code == EventCode.ARM_STATE_CHANGED:
                try:
                    extra = f" -> {ArmState(p.value).name}"
                except ValueError:
                    pass
            return f"[MCU {p.mcu_time_us / 1e6:10.3f}s] {sev:5s} {code} src={p.source} val={p.value}{extra}"
        if kind == "link":
            return "链路恢复" if p else "链路丢失：300 ms 未收到有效帧"
        if kind == "transport":
            return f"串口 {p[0]} {p[1]}"
        if kind == "device_info":
            return (f"设备 {p.fw_name} 协议 v{p.proto_version >> 4}.{p.proto_version & 15} "
                    f"{p.joint_count} 关节 记录 {p.joint_record_size} B 上报 {p.state_rate_hz} Hz")
        if kind == "replay_finished":
            return f"回放结束 {p}"
        if kind == "replay_seek":
            return f"回放跳转到第 {p} 帧"
        if kind == "sim_error":
            return f"仿真异常退出：{p}"
        if kind == "sim_stopped":
            return f"仿真已停止（{p}）"
        if kind in ("info", "warn"):
            return str(p)
        return f"{kind} {p}"

    # ---------------------------------------------------------------- GUI 取数
    def latest(self) -> RobotSample | None:
        with self._lock:
            return self._latest

    def drain_events(self) -> list[tuple[str, str, Any]]:
        out = []
        while self._events:
            out.append(self._events.popleft())
        return out

    def status(self) -> dict:
        b = self.backend
        return b.status() if b is not None else {"backend": "—", "state": "closed"}

    def estop(self, latch: bool = True) -> None:
        b = self.backend
        if b is None or not b.can_estop:
            self._push("warn", "当前数据源不支持急停")
            return
        fut = b.estop(latch)
        what = "急停" if latch else "解除急停"

        def done(f):
            e = f.exception()
            self._push("estop", f"{what}{'失败：' + str(e) if e else '已确认'}")
        fut.add_done_callback(done)
