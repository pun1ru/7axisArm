"""关节状态表：每关节一行，单元格对象一次建好，10 Hz 原地改文字。"""

from __future__ import annotations

import time
from collections.abc import Callable

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QFont
from PySide6.QtWidgets import QAbstractItemView, QHeaderView, QTableWidget, QTableWidgetItem

from ..kinematics import DHTable, euler_zyx_deg, forward_tcp
from ..model import RobotSample
from ..protocol.enums import JointFlag, MotorStatus
from . import theme

COLUMNS = (("角度\ndeg", "angle_deg", "{:9.3f}"), ("目标\ndeg", "target_deg", "{:9.3f}"),
           ("角速度\ndeg/s", "velocity_dps", "{:9.2f}"), ("反馈力矩\nN·m", "torque_nm", "{:7.3f}"),
           ("指令力矩\nN·m", "torque_cmd_nm", "{:7.3f}"), ("温度\n°C", "temperature_c", "{:d}"),
           ("电机状态", "motor_status", None), ("错误码", "error_id", "{:d}"),
           ("编码器", "encoder", "{:d}"), ("反馈计数", "feedback_count", "{:d}"), ("标志", "joint_flags", None))

_STATUS_OK = {MotorStatus.ENABLED}
_STATUS_IDLE = {MotorStatus.DISABLED}
POSE_ROWS = ("实际位姿", "目标位姿")


def status_text(v) -> tuple[str, str]:
    try:
        s = MotorStatus(v)
    except ValueError:
        return f"未知({v})", theme.ERR
    color = theme.OK if s in _STATUS_OK else theme.MUTED if s in _STATUS_IDLE else theme.ERR
    return s.name, color


def flags_text(v: int) -> str:
    f = JointFlag(v)
    parts = []
    if JointFlag.ONLINE in f:
        parts.append("在线")
    else:
        parts.append("离线")
    if JointFlag.AT_LIMIT in f:
        parts.append("限位")
    if JointFlag.HOST_TORQUE_VALID in f:
        parts.append("上位机τ")
    if JointFlag.TORQUE_CLAMPED in f:
        parts.append("限幅")
    return " ".join(parts)


class JointTable(QTableWidget):
    STALE_S = 0.5

    def __init__(self, joint_count: int = 7,
                 table_provider: Callable[[], DHTable | None] | None = None,
                 parent=None) -> None:
        super().__init__(joint_count + len(POSE_ROWS), len(COLUMNS), parent)
        self.joint_count = joint_count
        self._table_provider = table_provider
        self.setHorizontalHeaderLabels([c[0] for c in COLUMNS])
        self.setVerticalHeaderLabels([f"J{i}" for i in range(joint_count)] + list(POSE_ROWS))
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setAlternatingRowColors(True)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        self.verticalHeader().setDefaultSectionSize(22)
        mono = QFont("Consolas")
        mono.setStyleHint(QFont.Monospace)
        for r in range(joint_count):
            vh = self.verticalHeaderItem(r)
            vh.setForeground(QBrush(QColor(theme.JOINT_COLORS[r % 7])))
            for c in range(len(COLUMNS)):
                it = QTableWidgetItem("—")
                it.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                it.setFont(mono)
                self.setItem(r, c, it)
        for r in range(joint_count, joint_count + len(POSE_ROWS)):
            it = QTableWidgetItem("—")
            it.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            it.setFont(mono)
            self.setItem(r, 0, it)
            self.setSpan(r, 0, 1, len(COLUMNS))
            vh = self.verticalHeaderItem(r)
            vh.setForeground(QBrush(QColor(theme.OK if r == joint_count else theme.WARN)))
        self._stale = True
        self.setMinimumHeight(70)           # 放不下时表格自己出滚动条；写死 7 行高度会把窗口最小高度撑到 1000+

    def update_sample(self, s: RobotSample | None) -> None:
        now = time.monotonic()
        if s is None:
            self._set_stale(True)
            self._set_pose_text("—", "—")
            return
        # host_time_ns 来自 time.monotonic_ns()，与这里的 now 同一时钟
        self._set_stale(now - s.host_time_ns * 1e-9 > self.STALE_S)
        if self.joint_count != len(s.joints):
            return
        for r, j in enumerate(s.joints):
            for c, (_, key, fmt) in enumerate(COLUMNS):
                v = getattr(j, key)
                it = self.item(r, c)
                if key == "motor_status":
                    txt, color = status_text(v)
                    it.setForeground(QBrush(QColor(color)))
                elif key == "joint_flags":
                    txt = flags_text(v)
                    it.setForeground(QBrush(QColor(theme.OK if JointFlag.ONLINE in JointFlag(v) else theme.ERR)))
                elif key == "error_id":
                    txt = fmt.format(int(v))
                    it.setForeground(QBrush(QColor(theme.ERR if v else theme.TEXT)))
                else:
                    txt = fmt.format(int(v) if "d}" in fmt else float(v))
                if it.text() != txt:
                    it.setText(txt)
        self._update_pose_rows(s)

    def _update_pose_rows(self, s: RobotSample) -> None:
        table = self._table_provider() if self._table_provider is not None else None
        if table is None or len(s.joints) != table.n:
            self._set_pose_text("无可用 DH", "无可用 DH")
            return
        actual = self._pose_text(table, [j.angle_deg for j in s.joints])
        target = self._pose_text(table, [j.target_deg for j in s.joints])
        self._set_pose_text(actual, target)

    @staticmethod
    def _pose_text(table: DHTable, q_deg: list[float]) -> str:
        q = np.asarray(q_deg, dtype=float)
        if q.shape != (table.n,) or not np.all(np.isfinite(q)):
            return "位姿无效"
        pose = forward_tcp(table, q)
        xyz = pose[:3, 3]
        roll, pitch, yaw = euler_zyx_deg(pose[:3, :3])
        return (f"X={xyz[0]:+.4f} m   Y={xyz[1]:+.4f} m   Z={xyz[2]:+.4f} m"
                f"    |    Roll={roll:+.3f}°   Pitch={pitch:+.3f}°   Yaw={yaw:+.3f}°")

    def _set_pose_text(self, actual: str, target: str) -> None:
        for r, txt in zip(range(self.joint_count, self.joint_count + len(POSE_ROWS)),
                          (actual, target)):
            it = self.item(r, 0)
            if it.text() != txt:
                it.setText(txt)

    def _set_stale(self, stale: bool) -> None:
        if stale == self._stale:
            return
        self._stale = stale
        self.setStyleSheet("QTableWidget { color: #5a606b; }" if stale else "")
        self.setToolTip("数据超过 0.5 s 未更新" if stale else "")
