"""CSV 记录与读取。写盘在独立线程，采样线程只做非阻塞入队。

列：mcu_time_us, host_time_ns, seq, arm_state, status_flags, host_cmd_age_ms,
    然后每个关节 J{i}_angle_deg, J{i}_velocity_dps, J{i}_torque_nm, J{i}_target_deg,
    J{i}_torque_cmd_nm, J{i}_encoder, J{i}_motor_status, J{i}_error_id, J{i}_temperature_c,
    J{i}_joint_flags, J{i}_feedback_count
mcu_time_us 是展开后的 MCU 采样时刻；host_time_ns 是上位机 monotonic 接收时刻，两者不可互换。
"""

from __future__ import annotations

import csv
import logging
import queue
import re
import threading
import time
from pathlib import Path

from ..protocol.msg_report import JointRecord
from .state import RobotSample

log = logging.getLogger("armhost.model.recorder")

HEAD = ("mcu_time_us", "host_time_ns", "seq", "arm_state", "status_flags", "host_cmd_age_ms")
JFIELDS = ("angle_deg", "velocity_dps", "torque_nm", "target_deg", "torque_cmd_nm", "encoder",
           "motor_status", "error_id", "temperature_c", "joint_flags", "feedback_count")
_FLOAT = {"angle_deg", "velocity_dps", "torque_nm", "target_deg", "torque_cmd_nm"}


def header(joint_count: int) -> list[str]:
    return list(HEAD) + [f"J{i}_{f}" for i in range(joint_count) for f in JFIELDS]


def row(s: RobotSample) -> list:
    out: list = [s.mcu_time_us, s.host_time_ns, s.seq, int(s.arm_state), int(s.status_flags),
                 s.host_cmd_age_ms]
    for j in s.joints:
        for f in JFIELDS:
            v = getattr(j, f)
            out.append(f"{v:.6g}" if f in _FLOAT else int(v))
    return out


class CsvRecorder:
    def __init__(self, max_queue: int = 20000) -> None:
        self._q: queue.Queue = queue.Queue(max_queue)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.path: Path | None = None
        self.rows = 0
        self.dropped = 0
        self.error: str | None = None
        self.started_at = 0.0

    @property
    def active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, path: str | Path) -> None:
        if self.active:
            self.stop()
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.rows = self.dropped = 0
        self.error = None
        self._q = queue.Queue(self._q.maxsize)
        self._stop.clear()
        self.started_at = time.monotonic()
        self._thread = threading.Thread(target=self._run, name="csv-writer", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)
        self._thread = None

    def put(self, s: RobotSample) -> None:
        """采样线程调用，不阻塞；队列满（磁盘跟不上）就丢并计数。"""
        if not self.active:
            return
        try:
            self._q.put_nowait(s)
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        writer = None
        jc = None
        last_flush = time.monotonic()
        try:
            with open(self.path, "w", newline="", encoding="utf-8") as fp:
                w = csv.writer(fp)
                while not (self._stop.is_set() and self._q.empty()):
                    try:
                        s = self._q.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if jc is None:
                        jc = len(s.joints)
                        w.writerow(header(jc))
                        writer = w
                    if len(s.joints) != jc:
                        continue
                    writer.writerow(row(s))
                    self.rows += 1
                    if time.monotonic() - last_flush > 0.5:
                        fp.flush()
                        last_flush = time.monotonic()
        except OSError as e:
            self.error = str(e)
            log.error("CSV 写入失败: %s", e)


class SnapshotRecorder:
    """One CSV row per explicit capture, for gravity-identification poses.

    The file is created on the first capture and stays open until stop() or a
    new dataset is started. This is deliberately separate from CsvRecorder so
    continuous recording remains unchanged.
    """

    def __init__(self) -> None:
        self.path: Path | None = None
        self.rows = 0
        self.error: str | None = None
        self._fp = None
        self._writer = None
        self._joint_count: int | None = None

    @property
    def active(self) -> bool:
        return self._fp is not None

    def start(self, path: str | Path) -> None:
        self.stop()
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.error = None
        self.rows = 0
        self._joint_count = None
        try:
            self._fp = open(self.path, "w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._fp)
        except OSError as e:
            self.error = str(e)
            self._fp = self._writer = None
            raise

    def capture(self, s: RobotSample) -> bool:
        if self._fp is None or self._writer is None:
            raise RuntimeError("snapshot recorder is not active")
        try:
            if self._joint_count is None:
                self._joint_count = len(s.joints)
                self._writer.writerow(header(self._joint_count))
            if len(s.joints) != self._joint_count:
                raise ValueError(f"joint count changed from {self._joint_count} to {len(s.joints)}")
            self._writer.writerow(row(s))
            self._fp.flush()
            self.rows += 1
            return True
        except (OSError, ValueError) as e:
            self.error = str(e)
            log.error("snapshot write failed: %s", e)
            return False

    def stop(self) -> None:
        if self._fp is not None:
            try:
                self._fp.close()
            except OSError as e:
                self.error = str(e)
        self._fp = None
        self._writer = None
        self._joint_count = None


def load_csv(path: str | Path) -> list[RobotSample]:
    with open(path, newline="", encoding="utf-8") as fp:
        r = csv.reader(fp)
        cols = next(r)
        jc = max((int(m.group(1)) + 1 for c in cols if (m := re.match(r"J(\d+)_", c))), default=0)
        idx = {c: i for i, c in enumerate(cols)}
        missing = [c for c in header(jc) if c not in idx]
        if missing:
            raise ValueError(f"CSV 缺少列: {missing[:5]}")
        out = []
        for line in r:
            if not line:
                continue
            joints = tuple(JointRecord(**{f: (float if f in _FLOAT else int)(line[idx[f"J{i}_{f}"]])
                                          for f in JFIELDS}) for i in range(jc))
            out.append(RobotSample(int(line[idx["mcu_time_us"]]), int(line[idx["host_time_ns"]]),
                                   int(line[idx["seq"]]), int(line[idx["arm_state"]]),
                                   int(line[idx["status_flags"]]), int(line[idx["host_cmd_age_ms"]]),
                                   joints))
        return out
