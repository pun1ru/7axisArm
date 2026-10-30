"""可切换的数据后端：串口真机 / 模拟设备 / CSV 回放。都不依赖 Qt。

UI 只认 RobotBackend：
  add_sample_listener(cb)  cb(RobotSample)，在后端线程里调用，必须轻量
  add_event_listener(cb)   cb(kind: str, payload)，在后端线程里调用
  status() -> dict         线程安全快照，UI 定时拉取
"""

from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import Future
from pathlib import Path
from typing import Any, Callable

from ..comm import ArmLink, FaultConfig, SerialConfig, SerialTransport, SimDevice, make_mock_transport
from ..comm.stats import McuClock
from ..protocol import MsgType
from ..protocol.msg_report import JointState
from .recorder import load_csv
from .state import RobotSample

log = logging.getLogger("armhost.backend")

SampleListener = Callable[[RobotSample], None]
EventListener = Callable[[str, Any], None]


class RobotBackend:
    name = "base"
    is_hardware = False
    can_estop = False

    def __init__(self) -> None:
        self._samples: list[SampleListener] = []
        self._events: list[EventListener] = []

    def add_sample_listener(self, cb: SampleListener) -> None:
        self._samples.append(cb)

    def add_event_listener(self, cb: EventListener) -> None:
        self._events.append(cb)

    def _emit_sample(self, s: RobotSample) -> None:
        for cb in list(self._samples):
            try:
                cb(s)
            except Exception:
                log.exception("sample listener failed")

    def _emit_event(self, kind: str, payload: Any = None) -> None:
        for cb in list(self._events):
            try:
                cb(kind, payload)
            except Exception:
                log.exception("event listener failed")

    def start(self) -> None: ...
    def stop(self) -> None: ...

    def status(self) -> dict:
        return {"backend": self.name}

    def estop(self, latch: bool = True):
        raise NotImplementedError


class SerialBackend(RobotBackend):
    name = "serial"
    is_hardware = True
    can_estop = True

    def __init__(self, transport: SerialTransport, label: str | None = None) -> None:
        super().__init__()
        self.link = ArmLink(transport)
        self.label = label or transport.config.port
        self._clock = McuClock()
        self.link.subscribe(MsgType.JOINT_STATE, self._on_state)
        self.link.subscribe(MsgType.SYSTEM_STATUS, lambda r: self._emit_event("system", r.msg))
        self.link.subscribe(MsgType.EVENT, lambda r: self._emit_event("event", r.msg))
        self.link.add_link_listener(self._on_link)
        transport.add_state_listener(lambda st, why: self._emit_event("transport", (st.value, why)))

    @classmethod
    def open_port(cls, port: str, baudrate: int = 921600) -> "SerialBackend":
        return cls(SerialTransport(SerialConfig(port=port, baudrate=baudrate)))

    def _on_state(self, r) -> None:
        m: JointState = r.msg
        self._emit_sample(RobotSample(self._clock.unwrap(m.mcu_time_us), r.host_time_ns, r.frame.seq,
                                      m.arm_state, m.status_flags, m.host_cmd_age_ms, tuple(m.joints)))

    def _on_link(self, alive: bool) -> None:
        if not alive:
            self._clock.reset()
        self._emit_event("link", alive)
        if alive:   # 每次链路恢复都重新握手，取关节数与版本
            fut = self.link.request_device_info()
            fut.add_done_callback(lambda f: f.exception() is None and self._emit_event("device_info", f.result()))

    def start(self) -> None:
        self.link.start()

    def stop(self) -> None:
        self.link.stop()

    def estop(self, latch: bool = True):
        return self.link.estop(latch)

    def read_dh(self) -> Future:
        """PARAM_READ 0x0400 × (关节数×4)，组装成 DHTable。返回 Future，不阻塞调用方。"""
        from ..kinematics import DHRow, DHTable
        n = self.link.device_info.joint_count if self.link.device_info else 7
        futs = [self.link.read_param(0x0400, k) for k in range(n * 4)]
        out: Future = Future()
        left = [len(futs)]
        lock = threading.Lock()

        def done(_f):
            with lock:
                left[0] -= 1
                if left[0] or out.done():
                    return
            try:
                v = [f.result().value for f in futs]
                out.set_result(DHTable([DHRow(*v[4 * i:4 * i + 4]) for i in range(n)], f"设备 {self.label}"))
            except Exception as e:     # 任意一个参数失败就整体失败
                out.set_exception(e)
        for f in futs:
            f.add_done_callback(done)
        return out

    def status(self) -> dict:
        d = self.link.snapshot_stats()
        d["backend"] = self.name
        d["port"] = self.label
        d["device_info"] = self.link.device_info
        d["system"] = self.link.mirror.snapshot().system
        return d


class MockBackend(SerialBackend):
    """模拟设备：走完整的协议栈（编码 -> 虚拟串口 -> 解析），不是直接造 RobotSample。"""
    name = "mock"
    is_hardware = False

    def __init__(self, rate_hz: int = 200, faults: FaultConfig | None = None) -> None:
        self.device = SimDevice(state_rate_hz=rate_hz, faults=faults)
        tr, _, self.factory = make_mock_transport(self.device)
        super().__init__(tr, label=f"模拟设备 {rate_hz} Hz")

    def start(self) -> None:
        self.device.start()
        super().start()

    def stop(self) -> None:
        super().stop()
        self.device.stop()


class ReplayBackend(RobotBackend):
    """按 MCU 时间戳节奏回放 CSV。回放出来的 host_time_ns 是回放时刻，原始接收时刻不重放。"""
    name = "replay"

    def __init__(self, path: str | Path, speed: float = 1.0, loop: bool = False) -> None:
        super().__init__()
        self.path = Path(path)
        self.samples = load_csv(self.path)
        if not self.samples:
            raise ValueError("CSV 里没有数据")
        self.speed = speed
        self.loop = loop
        self.index = 0
        self._paused = threading.Event()
        self._stop = threading.Event()
        self._seek: int | None = None
        self._thread: threading.Thread | None = None

    @property
    def duration_s(self) -> float:
        return (self.samples[-1].mcu_time_us - self.samples[0].mcu_time_us) * 1e-6

    def set_paused(self, paused: bool) -> None:
        (self._paused.set if paused else self._paused.clear)()

    def seek(self, fraction: float) -> None:
        self._seek = int(max(0.0, min(1.0, fraction)) * (len(self.samples) - 1))

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="replay", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(1.0)

    def _run(self) -> None:
        epoch = 0
        while not self._stop.is_set():
            wall0 = time.monotonic()
            t0 = self.samples[self.index].mcu_time_us
            while self.index < len(self.samples) and not self._stop.is_set():
                if self._seek is not None:
                    self.index, self._seek = self._seek, None
                    self._emit_event("replay_seek", self.index)   # UI 收到后清空缓存
                    break
                if self._paused.is_set():
                    time.sleep(0.05)
                    break
                s = self.samples[self.index]
                due = wall0 + (s.mcu_time_us - t0) * 1e-6 / max(self.speed, 1e-3)
                d = due - time.monotonic()
                if d > 0.002:
                    self._stop.wait(min(d, 0.05))
                    continue
                self._emit_sample(RobotSample(s.mcu_time_us + epoch, time.monotonic_ns(), s.seq, s.arm_state,
                                              s.status_flags, s.host_cmd_age_ms, s.joints))
                self.index += 1
            if self.index >= len(self.samples):
                if not self.loop:
                    self._emit_event("replay_finished", self.path)
                    return
                epoch += self.samples[-1].mcu_time_us - self.samples[0].mcu_time_us + 1000
                self.index = 0

    def status(self) -> dict:
        return {"backend": self.name, "port": self.path.name, "state": "replay",
                "link_alive": True, "progress": self.index / max(1, len(self.samples) - 1),
                "duration_s": self.duration_s, "speed": self.speed}
