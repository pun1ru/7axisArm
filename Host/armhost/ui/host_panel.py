"""Host mode target panel: firmware limits and one-joint target commands."""

from __future__ import annotations

import time

from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
                               QMessageBox, QPushButton, QVBoxLayout, QWidget)

from ..protocol.enums import ArmState, JointFlag


class HostPanel(QWidget):
    def __init__(self, ctrl, joint_count: int = 7, parent=None) -> None:
        super().__init__(parent)
        self.ctrl = ctrl
        self.joint_count = joint_count
        self._connected = False
        self._limits_ready = False
        self._limits_future = None
        self._limits_started = 0.0
        self._send_future = None
        self._next_limits_try = 0.0
        self._last_sample = None
        self._feedback = ""
        self._feedback_until = 0.0

        self.state_label = QLabel("未连接")
        self.state_label.setObjectName("value")
        self.state_label.setWordWrap(True)

        self.spins: list[QDoubleSpinBox] = []
        grid = QGridLayout()
        for i in range(joint_count):
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(-180.0, 180.0)
            spin.setSuffix(" deg")
            spin.setSingleStep(0.5)
            spin.setEnabled(False)
            grid.addWidget(QLabel(f"J{i}"), i // 2, (i % 2) * 2)
            grid.addWidget(spin, i // 2, (i % 2) * 2 + 1)
            self.spins.append(spin)

        self.joint_combo = QComboBox()
        for i in range(joint_count):
            self.joint_combo.addItem(f"J{i}", i)
        self.read_btn = QPushButton("读取限位")
        self.current_btn = QPushButton("载入当前角度")
        self.send_closed_btn = QPushButton("发送并闭环")
        self.send_open_btn = QPushButton("仅发送目标")
        self.stop_btn = QPushButton("全部退出闭环")
        self.send_closed_btn.setToolTip("只更新选中关节的目标，并让该关节投入角度闭环")
        self.send_open_btn.setToolTip("只更新选中关节的目标；该关节输出零力矩，其他关节状态不变")
        self.read_btn.clicked.connect(self.request_limits)
        self.current_btn.clicked.connect(self.load_current)
        self.send_closed_btn.clicked.connect(lambda _=False: self.send_target(True))
        self.send_open_btn.clicked.connect(lambda _=False: self.send_target(False))
        self.stop_btn.clicked.connect(self.stop_loops)

        buttons = QHBoxLayout()
        buttons.addWidget(self.read_btn)
        buttons.addWidget(self.current_btn)
        buttons.addStretch(1)
        send_row = QHBoxLayout()
        send_row.addWidget(QLabel("发送关节"))
        send_row.addWidget(self.joint_combo)
        send_row.addWidget(self.send_closed_btn)
        send_row.addWidget(self.send_open_btn)
        send_row.addWidget(self.stop_btn)
        send_row.addStretch(1)
        note = QLabel("一次只发送一个关节；未选中的关节保持原目标和原闭环状态。"
                      "目标必须位于固件限位内，拨杆左上 + 右下时才接受。")
        note.setObjectName("title")
        note.setWordWrap(True)
        lay = QVBoxLayout(self)
        lay.addWidget(self.state_label)
        lay.addLayout(grid)
        lay.addLayout(buttons)
        lay.addLayout(send_row)
        lay.addWidget(note)
        lay.addStretch(1)
        self._update_buttons()

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        if not connected:
            self._limits_ready = False
            self._limits_future = None
            self._limits_started = 0.0
            self._send_future = None
            for spin in self.spins:
                spin.setEnabled(False)
            self.state_label.setText("未连接")
        else:
            self._next_limits_try = 0.0
        self._update_buttons()

    def request_limits(self) -> None:
        if not self._connected or self._limits_future is not None:
            return
        try:
            future = self.ctrl.read_limits()
        except Exception as e:
            self.state_label.setText(f"读取限位失败：{e}")
            self._next_limits_try = time.monotonic() + 2.0
            return
        if future is None:
            self.state_label.setText("当前数据源不支持限位读取")
            self._next_limits_try = time.monotonic() + 2.0
            return
        self._limits_future = future
        self._limits_started = time.monotonic()
        self.state_label.setText("正在读取固件限位…")

    def load_current(self) -> None:
        sample = self._last_sample
        if sample is None:
            QMessageBox.information(self, "载入当前角度", "还没有收到关节数据")
            return
        for spin, joint in zip(self.spins, sample.joints):
            value = min(max(float(joint.angle_deg), spin.minimum()), spin.maximum())
            spin.setValue(value)

    def send_target(self, closed_loop: bool) -> None:
        if not self._is_host_state():
            QMessageBox.warning(self, "Host 目标", "当前不是 Host 模式，请先把拨杆切到左上 + 右下")
            return
        if not self._limits_ready:
            QMessageBox.warning(self, "Host 目标", "固件限位尚未读取，不能发送目标")
            return
        joint = int(self.joint_combo.currentData())
        try:
            self._send_future = self.ctrl.set_host_target(joint, self.spins[joint].value(), closed_loop)
        except Exception as e:
            QMessageBox.warning(self, "Host 目标", str(e))
            return
        if self._send_future is None:
            return
        action = "闭环目标" if closed_loop else "不闭环目标"
        self._set_feedback(f"J{joint} {action}已发送，等待固件确认…")
        self._update_buttons()

    def stop_loops(self) -> None:
        if not self._is_host_state():
            return
        self._send_future = self.ctrl.stop_host_loops()
        if self._send_future is not None:
            self._set_feedback("正在退出全部关节闭环…")
            self._update_buttons()

    def _set_feedback(self, text: str, seconds: float = 2.0) -> None:
        self._feedback = text
        self._feedback_until = time.monotonic() + seconds
        self.state_label.setText(text)

    def _is_host_state(self) -> bool:
        return self._last_sample is not None and self._last_sample.arm_state == ArmState.HOST

    def _update_buttons(self) -> None:
        host = self._is_host_state()
        can_send = self._connected and self._limits_ready and host and self._send_future is None
        self.send_closed_btn.setEnabled(can_send)
        self.send_open_btn.setEnabled(can_send)
        self.stop_btn.setEnabled(self._connected and host and self._send_future is None)
        self.read_btn.setEnabled(self._connected and self._limits_future is None)
        self.current_btn.setEnabled(self._connected and self._last_sample is not None)

    def _poll_futures(self) -> None:
        now = time.monotonic()
        if self._limits_future is not None:
            future = self._limits_future
            if future.done():
                self._limits_future = None
                try:
                    mins, maxs = future.result()
                    if len(mins) != self.joint_count or len(maxs) != self.joint_count:
                        raise ValueError("关节数量不匹配")
                    for i, (spin, lo, hi) in enumerate(zip(self.spins, mins, maxs)):
                        if not lo < hi:
                            raise ValueError(f"J{i} 限位无效：{lo} .. {hi}")
                        spin.setRange(float(lo), float(hi))
                        spin.setValue(min(max(spin.value(), float(lo)), float(hi)))
                        spin.setEnabled(True)
                    self._limits_ready = True
                    self._set_feedback("固件限位已载入")
                except Exception as e:
                    self._limits_ready = False
                    for spin in self.spins:
                        spin.setEnabled(False)
                    self._set_feedback(f"读取限位失败：{e}", 3.0)
                    self._next_limits_try = now + 2.0
            elif now - self._limits_started > 5.0:
                self._limits_future = None
                future.add_done_callback(lambda f: f.exception())
                self._limits_ready = False
                for spin in self.spins:
                    spin.setEnabled(False)
                self._set_feedback("限位读取超时，请重试", 3.0)
                self._next_limits_try = now + 2.0

        if self._send_future is not None and self._send_future.done():
            future = self._send_future
            self._send_future = None
            try:
                future.result()
                self._set_feedback("固件已确认目标")
            except Exception as e:
                self._set_feedback(f"目标被拒绝：{e}")
        self._update_buttons()

    def tick(self, sample) -> None:
        self._last_sample = sample
        self._poll_futures()
        if self._connected and sample is not None and not self._limits_ready \
                and self._limits_future is None and time.monotonic() >= self._next_limits_try:
            self.request_limits()

        if self._feedback and time.monotonic() < self._feedback_until:
            self.state_label.setText(self._feedback)
        elif sample is None:
            if self._connected:
                self.state_label.setText("已连接，等待关节数据")
        elif sample.arm_state == ArmState.HOST:
            if not self._limits_ready:
                if self._limits_future is not None:
                    self.state_label.setText("Host 模式：正在读取固件限位，发送暂不可用")
                else:
                    self.state_label.setText("Host 模式：限位未载入，发送暂不可用")
            else:
                closed = sum(bool(j.joint_flags & JointFlag.HOST_CLOSED_LOOP) for j in sample.joints)
                valid = sum(bool(j.joint_flags & JointFlag.HOST_TARGET_VALID) for j in sample.joints)
                self.state_label.setText(f"Host 模式：{closed}/{self.joint_count} 关节闭环，{valid} 个已设目标")
        else:
            state = ArmState(sample.arm_state).name if sample.arm_state in ArmState._value2member_map_ else str(sample.arm_state)
            self.state_label.setText(f"当前 {state}，切到左上 + 右下后可用")
        self._update_buttons()
