"""ArmLink：在 SerialTransport 之上实现协议会话。

* 解析：transport 读线程回调 -> FrameParser -> 解码 -> 更新镜像结构体 -> 分发订阅者
* 发送：分配 SEQ -> 编码 -> transport.write（非阻塞）
* 应答匹配：ACK.ack_seq / PARAM_VALUE.req_seq / DEVICE_INFO 按类型取最早的 pending
* 超时与重试：只有幂等命令可重试（同 SEQ 重发，固件按 SEQ 去重）；力矩命令绝不重试
* 心跳：每 heartbeat_period_s 发一次；超过 link_timeout_s 没收到任何有效帧判链路丢失
* 断线：所有 pending 以 LinkLostError 失败，解析器和序号跟踪复位

订阅回调在串口读线程中执行：只能做轻量工作（入队 / 写缓存），不能操作 Qt 控件。
"""

from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass
from typing import Any, Callable

from ..protocol import (Ack, AckResult, ClearTorque, DecodeError, DeviceInfo, DeviceInfoReq,
                        Estop, EstopAction, Frame, FrameFlag, FrameParser, Heartbeat, MsgType,
                        ParamRead, ParamValue, ParamWrite, SetJointTorque, TorqueItem,
                        decode_frame, encode_frame)
from ..protocol.enums import ParamType
from ..protocol.params import BY_ID
from .hexlog import HexLog
from .mirror import ArmMirror
from .stats import LinkStats, SeqCounter, SeqTracker
from .transport import LinkState, SerialTransport

log = logging.getLogger("armhost.comm.link")


class LinkLostError(ConnectionError):
    pass


class AckTimeoutError(TimeoutError):
    pass


class CommandRejected(RuntimeError):
    def __init__(self, result: int, reply: Any) -> None:
        super().__init__(f"rejected: {AckResult(result).name if result in AckResult._value2member_map_ else result}")
        self.result = result
        self.reply = reply


_RETRYABLE = {MsgType.DEVICE_INFO_REQ, MsgType.PARAM_READ, MsgType.PARAM_WRITE,
              MsgType.ESTOP, MsgType.CLEAR_TORQUE}


@dataclass
class _Pending:
    seq: int
    msg_type: int
    frame: bytes
    future: Future
    deadline: float
    retries_left: int
    timeout_s: float


@dataclass(frozen=True, slots=True)
class Received:
    msg: Any
    frame: Frame
    host_time_ns: int


Listener = Callable[[Received], None]


class ArmLink:
    def __init__(self, transport: SerialTransport, *, ack_timeout_s: float = 0.15,
                 retries: int = 2, heartbeat_period_s: float = 0.1,
                 link_timeout_s: float = 0.3, tick_s: float = 0.01) -> None:
        self.transport = transport
        self.ack_timeout_s = ack_timeout_s
        self.retries = retries
        self.heartbeat_period_s = heartbeat_period_s
        self.link_timeout_s = link_timeout_s
        self._tick_s = tick_s

        self.parser = FrameParser()
        self.stats = LinkStats()
        self.hexlog = HexLog()
        self.mirror = ArmMirror()
        self.device_info: DeviceInfo | None = None

        self._seq = SeqCounter()
        self._rx_seq = SeqTracker()
        self._pending: dict[int, _Pending] = {}
        self._plock = threading.Lock()
        self._listeners: dict[int | None, list[Listener]] = {}
        self._link_listeners: list[Callable[[bool], None]] = []
        self._last_rx = 0.0
        self._alive = False
        self._stop = threading.Event()
        self._timer: threading.Thread | None = None

        transport.add_rx_listener(self._on_bytes)
        transport.add_tx_listener(lambda b: self.hexlog.record("TX", b))
        transport.add_state_listener(self._on_transport_state)

    # ================================================================ 生命周期
    def start(self) -> None:
        self._stop.clear()
        self.transport.open()
        self._timer = threading.Thread(target=self._timer_loop, name="link-timer", daemon=True)
        self._timer.start()

    def stop(self, timeout: float = 1.0) -> None:
        """退出前尽力发急停 + 清零，再关端口。"""
        if self.transport.is_connected:
            self._send(Estop(EstopAction.LATCH))
            self._send(ClearTorque())
            time.sleep(0.05)
        self._stop.set()
        if self._timer and self._timer is not threading.current_thread():
            self._timer.join(timeout)
        self.transport.close(timeout)
        self._fail_all(LinkLostError("link stopped"))

    @property
    def link_alive(self) -> bool:
        return self._alive

    # ================================================================ 订阅
    def subscribe(self, msg_type: int | None, cb: Listener) -> Callable[[], None]:
        """msg_type=None 订阅全部。返回取消订阅函数。"""
        key = None if msg_type is None else int(msg_type)
        self._listeners.setdefault(key, []).append(cb)
        return lambda: self._listeners.get(key, []).remove(cb)

    def add_link_listener(self, cb: Callable[[bool], None]) -> None:
        self._link_listeners.append(cb)

    # ================================================================ 高层命令
    def request_device_info(self, timeout_s: float | None = None) -> Future:
        return self._request(DeviceInfoReq(), timeout_s)

    def set_joint_torque(self, torques: dict[int, float], hold_ms: int = 100,
                         want_ack: bool = True) -> Future | None:
        """设定关节力矩（N*m，关节 +q 方向）。不重试：过时的力矩命令不应补发。"""
        msg = SetJointTorque(hold_ms, [TorqueItem(j, float(t)) for j, t in sorted(torques.items())])
        if want_ack:
            return self._request(msg, retries=0)
        self._send(msg)
        return None

    def clear_torque(self, joint_mask: int = 0xFFFFFFFF) -> Future:
        return self._request(ClearTorque(joint_mask))

    def estop(self, latch: bool = True) -> Future:
        return self._request(Estop(EstopAction.LATCH if latch else EstopAction.RELEASE))

    def read_param(self, param_id: int, index: int = 0) -> Future:
        return self._request(ParamRead(param_id, index))

    def write_param(self, param_id: int, index: int, value, ptype: ParamType | None = None) -> Future:
        if ptype is None:
            if param_id not in BY_ID:
                raise ValueError(f"unknown param 0x{param_id:04X}, ptype required")
            ptype = BY_ID[param_id].ptype
        return self._request(ParamWrite(param_id, index, ptype, value))

    # ================================================================ 发送
    def _send(self, msg, flags: int = 0, seq: int | None = None) -> tuple[int, bytes, bool]:
        seq = self._seq.next() if seq is None else seq
        frame = encode_frame(msg.TYPE, seq, msg.encode(), flags)
        ok = self.transport.write(frame)
        return seq, frame, ok

    def _request(self, msg, timeout_s: float | None = None, retries: int | None = None) -> Future:
        fut: Future = Future()
        if not self.transport.is_connected:
            fut.set_exception(LinkLostError("not connected"))
            return fut
        timeout_s = self.ack_timeout_s if timeout_s is None else timeout_s
        if retries is None:
            retries = self.retries if msg.TYPE in _RETRYABLE else 0
        seq = self._seq.next()
        frame = encode_frame(msg.TYPE, seq, msg.encode(), FrameFlag.ACK_REQ)
        with self._plock:
            self._pending[seq] = _Pending(seq, int(msg.TYPE), frame, fut,
                                          time.monotonic() + timeout_s, retries, timeout_s)
        if not self.transport.write(frame):
            self._resolve(seq, exc=LinkLostError("tx queue rejected"))
        return fut

    def _resolve(self, seq: int, result: Any = None, exc: BaseException | None = None) -> bool:
        with self._plock:
            p = self._pending.pop(seq, None)
        if p is None or p.future.done():
            return False
        if exc is not None:
            p.future.set_exception(exc)
        else:
            p.future.set_result(result)
        return True

    def _fail_all(self, exc: BaseException) -> None:
        with self._plock:
            items = list(self._pending.values())
            self._pending.clear()
        for p in items:
            if not p.future.done():
                p.future.set_exception(exc)

    # ================================================================ 接收
    def _on_bytes(self, data: bytes, host_time_ns: int) -> None:
        self.hexlog.record("RX", data)
        for fr in self.parser.feed(data):
            self._on_frame(fr, host_time_ns)

    def _on_frame(self, fr: Frame, host_time_ns: int) -> None:
        lost = self._rx_seq.update(fr.seq)
        if lost:
            self.stats.add(lost_frames=lost)
        try:
            msg = decode_frame(fr)
        except (DecodeError, Exception) as e:   # 负载异常不能杀死读线程
            self.stats.add(decode_errors=1)
            log.debug("decode type=0x%02X failed: %s", fr.msg_type, e)
            return
        self.stats.on_rx(fr.msg_type)
        self._last_rx = time.monotonic()
        if not self._alive:
            self._set_alive(True)

        self.mirror.update(msg, host_time_ns)
        self._match_reply(msg)

        rec = Received(msg, fr, host_time_ns)
        for key in (fr.msg_type, None):
            for cb in list(self._listeners.get(key, ())):
                try:
                    cb(rec)
                except Exception:
                    log.exception("listener failed")

    def _match_reply(self, msg) -> None:
        if isinstance(msg, Ack):
            with self._plock:
                p = self._pending.get(msg.ack_seq)
            if p is None or p.msg_type != msg.ack_type:
                return
            if msg.ok:
                self._resolve(msg.ack_seq, msg)
            else:
                self._resolve(msg.ack_seq, exc=CommandRejected(msg.result, msg))
        elif isinstance(msg, ParamValue):
            if msg.result in (AckResult.OK, AckResult.OK_CLAMPED):
                self._resolve(msg.req_seq, msg)
            else:
                self._resolve(msg.req_seq, exc=CommandRejected(msg.result, msg))
        elif isinstance(msg, DeviceInfo):
            self.device_info = msg
            with self._plock:
                cands = sorted((p for p in self._pending.values()
                                if p.msg_type == MsgType.DEVICE_INFO_REQ), key=lambda p: p.deadline)
            if cands:
                self._resolve(cands[0].seq, msg)

    # ================================================================ 定时
    def _timer_loop(self) -> None:
        next_hb = 0.0
        while not self._stop.wait(self._tick_s):
            now = time.monotonic()
            if self.transport.is_connected and now >= next_hb:
                next_hb = now + self.heartbeat_period_s
                self._send(Heartbeat(int(now * 1000) & 0xFFFFFFFF, self.parser.stats.frames_ok))
            if self._alive and now - self._last_rx > self.link_timeout_s:
                self._set_alive(False)
            self._check_timeouts(now)

    def _check_timeouts(self, now: float) -> None:
        with self._plock:
            expired = [p for p in self._pending.values() if now >= p.deadline]
        for p in expired:
            if p.retries_left > 0 and self.transport.is_connected:
                p.retries_left -= 1
                p.deadline = now + p.timeout_s
                self.transport.write(p.frame)          # 同 SEQ 重发
                log.debug("retry seq=%d type=0x%02X", p.seq, p.msg_type)
            else:
                self.stats.add(ack_timeouts=1)
                self._resolve(p.seq, exc=AckTimeoutError(f"seq={p.seq} type=0x{p.msg_type:02X}"))

    def _set_alive(self, alive: bool) -> None:
        self._alive = alive
        log.info("link %s", "alive" if alive else "lost (no valid frame)")
        for cb in list(self._link_listeners):
            try:
                cb(alive)
            except Exception:
                log.exception("link listener failed")

    def _on_transport_state(self, st: LinkState, reason: str) -> None:
        if st in (LinkState.RECONNECTING, LinkState.CLOSED):
            self._fail_all(LinkLostError(reason or st.value))
            self.parser.reset()
            self._rx_seq.reset()
            self.mirror.reset_clock()
            if self._alive:
                self._set_alive(False)
        if st == LinkState.RECONNECTING:
            self.stats.add(reconnects=1)

    # ================================================================ 诊断
    def snapshot_stats(self) -> dict:
        d = self.stats.snapshot()
        d["parser"] = self.parser.stats.as_dict()
        d["transport"] = self.transport.stats.snapshot()
        d["state"] = self.transport.state.value
        d["link_alive"] = self._alive
        with self._plock:
            d["pending"] = len(self._pending)
        return d
