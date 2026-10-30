"""SerialTransport：只负责字节收发，不理解协议。

* 读线程：阻塞 read（带超时），收到字节回调 on_bytes(data, host_time_ns)。
* 写线程：从有界队列取帧写出；断线期间队列里的帧直接丢弃并计数，
  **不会**在重连后补发——过时的运动命令绝不能延迟生效。
* 异常恢复：读写任一侧抛 SerialException / OSError -> 关闭端口 -> 按退避重连。
* serial_factory 可注入（虚拟串口 / 模拟设备），签名同 serial.Serial(**kwargs)。
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable

log = logging.getLogger("armhost.comm.transport")

try:
    import serial
    from serial.tools import list_ports
    SerialException: type[Exception] = serial.SerialException
except ImportError:          # 允许在没有 pyserial 的环境里用模拟设备
    serial = None
    list_ports = None
    SerialException = OSError


class LinkState(str, Enum):
    CLOSED = "closed"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"


@dataclass(frozen=True, slots=True)
class PortInfo:
    device: str
    description: str
    hwid: str
    vid: int | None
    pid: int | None
    serial_number: str | None


def list_serial_ports() -> list[PortInfo]:
    if list_ports is None:
        return []
    return [PortInfo(p.device, p.description or "", p.hwid or "", p.vid, p.pid, p.serial_number)
            for p in sorted(list_ports.comports(), key=lambda p: p.device)]


@dataclass
class SerialConfig:
    port: str = ""
    baudrate: int = 921600
    bytesize: int = 8
    parity: str = "N"
    stopbits: float = 1
    read_timeout_s: float = 0.02
    write_timeout_s: float = 0.1
    read_chunk: int = 4096
    tx_queue_size: int = 256
    auto_reconnect: bool = True
    reconnect_backoff_s: tuple[float, ...] = (0.5, 1.0, 2.0, 5.0)

    def serial_kwargs(self) -> dict[str, Any]:
        return dict(port=self.port, baudrate=self.baudrate, bytesize=self.bytesize,
                    parity=self.parity, stopbits=self.stopbits, timeout=self.read_timeout_s,
                    write_timeout=self.write_timeout_s)


@dataclass
class TransportStats:
    rx_bytes: int = 0
    tx_bytes: int = 0
    tx_frames: int = 0
    tx_dropped: int = 0
    tx_errors: int = 0
    rx_errors: int = 0
    reconnects: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def add(self, **kw: int) -> None:
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, getattr(self, k) + v)

    def snapshot(self) -> dict[str, int]:
        with self.lock:
            return {k: v for k, v in self.__dict__.items() if k != "lock"}


BytesCallback = Callable[[bytes, int], None]
StateCallback = Callable[[LinkState, str], None]


class SerialTransport:
    def __init__(self, config: SerialConfig | None = None,
                 serial_factory: Callable[..., Any] | None = None) -> None:
        self.config = config or SerialConfig()
        self._factory = serial_factory or (serial.Serial if serial else None)
        self._port: Any = None
        self._port_lock = threading.Lock()
        self._txq: queue.Queue[bytes] = queue.Queue(self.config.tx_queue_size)
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._reader: threading.Thread | None = None
        self._writer: threading.Thread | None = None
        self._state = LinkState.CLOSED
        self._on_bytes: list[BytesCallback] = []
        self._on_tx: list[Callable[[bytes], None]] = []
        self._on_state: list[StateCallback] = []
        self.stats = TransportStats()

    # ------------------------------------------------------------ 订阅
    def add_rx_listener(self, cb: BytesCallback) -> None:
        self._on_bytes.append(cb)

    def add_tx_listener(self, cb: Callable[[bytes], None]) -> None:
        self._on_tx.append(cb)

    def add_state_listener(self, cb: StateCallback) -> None:
        self._on_state.append(cb)

    @property
    def state(self) -> LinkState:
        return self._state

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    def _set_state(self, st: LinkState, reason: str = "") -> None:
        if st == self._state:
            return
        self._state = st
        log.info("transport %s %s", st.value, reason)
        for cb in list(self._on_state):
            try:
                cb(st, reason)
            except Exception:
                log.exception("state listener failed")

    # ------------------------------------------------------------ 生命周期
    def configure(self, **kw: Any) -> None:
        """修改参数；若已打开，下次重连时生效（调用 reopen() 立即生效）。"""
        for k, v in kw.items():
            if not hasattr(self.config, k):
                raise AttributeError(k)
            setattr(self.config, k, v)

    def open(self) -> None:
        if self._factory is None:
            raise RuntimeError("pyserial 未安装且未注入 serial_factory")
        if self._reader and self._reader.is_alive():
            return
        self._stop.clear()
        self._set_state(LinkState.CONNECTING)
        self._reader = threading.Thread(target=self._read_loop, name="serial-rx", daemon=True)
        self._writer = threading.Thread(target=self._write_loop, name="serial-tx", daemon=True)
        self._reader.start()
        self._writer.start()

    def close(self, timeout: float = 1.0) -> None:
        self._stop.set()
        self._close_port()
        for t in (self._reader, self._writer):
            if t and t.is_alive() and t is not threading.current_thread():
                t.join(timeout)
        self._drain_txq()
        self._set_state(LinkState.CLOSED)

    def reopen(self) -> None:
        self._close_port()          # 读线程会发现并按新配置重连

    def wait_connected(self, timeout: float) -> bool:
        return self._connected.wait(timeout)

    # ------------------------------------------------------------ 发送
    def write(self, frame: bytes) -> bool:
        """非阻塞入队。未连接或队列满返回 False 并计入 tx_dropped。"""
        if not self._connected.is_set():
            self.stats.add(tx_dropped=1)
            return False
        try:
            self._txq.put_nowait(bytes(frame))
            return True
        except queue.Full:
            self.stats.add(tx_dropped=1)
            return False

    # ------------------------------------------------------------ 内部
    def _try_open(self) -> bool:
        try:
            port = self._factory(**self.config.serial_kwargs())
            if hasattr(port, "reset_input_buffer"):
                port.reset_input_buffer()
        except (SerialException, OSError, ValueError) as e:
            log.warning("open %s failed: %s", self.config.port, e)
            return False
        with self._port_lock:
            self._port = port
        self._connected.set()
        self._set_state(LinkState.CONNECTED, self.config.port)
        return True

    def _close_port(self) -> None:
        self._connected.clear()
        with self._port_lock:
            port, self._port = self._port, None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass

    def _drain_txq(self) -> None:
        n = 0
        while True:
            try:
                self._txq.get_nowait()
                n += 1
            except queue.Empty:
                break
        if n:
            self.stats.add(tx_dropped=n)

    def _fail(self, reason: str) -> None:
        self._close_port()
        self._drain_txq()
        if not self._stop.is_set():
            self._set_state(LinkState.RECONNECTING, reason)

    def _read_loop(self) -> None:
        attempt = 0
        while not self._stop.is_set():
            port = self._port
            if port is None:
                if attempt and not self.config.auto_reconnect:
                    break
                if self._try_open():
                    if attempt:
                        self.stats.add(reconnects=1)
                    attempt = 0
                    continue
                bo = self.config.reconnect_backoff_s
                delay = bo[min(attempt, len(bo) - 1)]
                attempt += 1
                self._set_state(LinkState.RECONNECTING, "open failed")
                self._stop.wait(delay)
                continue
            try:
                n = getattr(port, "in_waiting", 0) or 1
                data = port.read(min(max(n, 1), self.config.read_chunk))
            except (SerialException, OSError, TypeError, AttributeError) as e:
                if self._stop.is_set():
                    break
                self.stats.add(rx_errors=1)
                log.warning("read error: %s", e)
                attempt = 1
                self._fail(f"read error: {e}")
                continue
            if not data:
                continue
            now = time.monotonic_ns()
            self.stats.add(rx_bytes=len(data))
            for cb in list(self._on_bytes):
                try:
                    cb(data, now)
                except Exception:
                    log.exception("rx listener failed")
        self._close_port()

    def _write_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._txq.get(timeout=0.1)
            except queue.Empty:
                continue
            port = self._port
            if port is None or not self._connected.is_set():
                self.stats.add(tx_dropped=1)
                continue
            try:
                n = port.write(frame)
                if hasattr(port, "flush"):
                    port.flush()
            except (SerialException, OSError) as e:
                if self._stop.is_set():
                    break
                self.stats.add(tx_errors=1)
                log.warning("write error: %s", e)
                self._fail(f"write error: {e}")
                continue
            if n is not None and n != len(frame):
                self.stats.add(tx_errors=1)
                continue
            self.stats.add(tx_bytes=len(frame), tx_frames=1)
            for cb in list(self._on_tx):
                try:
                    cb(frame)
                except Exception:
                    log.exception("tx listener failed")
