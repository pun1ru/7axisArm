"""关节状态表：每关节一行，单元格对象一次建好，10 Hz 原地改文字。"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QFont
from PySide6.QtWidgets import QAbstractItemView, QHeaderView, QTableWidget, QTableWidgetItem

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

    def __init__(self, joint_count: int = 7, parent=None) -> None:
        super().__init__(joint_count, len(COLUMNS), parent)
        self.setHorizontalHeaderLabels([c[0] for c in COLUMNS])
        self.setVerticalHeaderLabels([f"J{i}" for i in range(joint_count)])
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
        self._stale = True
        self.setMinimumHeight(70)           # 放不下时表格自己出滚动条；写死 7 行高度会把窗口最小高度撑到 1000+

    def update_sample(self, s: RobotSample | None) -> None:
        now = time.monotonic()
        if s is None:
            self._set_stale(True)
            return
        # host_time_ns 来自 time.monotonic_ns()，与这里的 now 同一时钟
        self._set_stale(now - s.host_time_ns * 1e-9 > self.STALE_S)
        if self.rowCount() != len(s.joints):
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

    def _set_stale(self, stale: bool) -> None:
        if stale == self._stale:
            return
        self._stale = stale
        self.setStyleSheet("QTableWidget { color: #5a606b; }" if stale else "")
        self.setToolTip("数据超过 0.5 s 未更新" if stale else "")
