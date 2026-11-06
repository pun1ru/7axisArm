"""Host 控制面板：固件限位、单关节调试和 7 轴关节角闭环。"""

from __future__ import annotations

import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
                               QMessageBox, QPushButton, QSlider, QVBoxLayout, QWidget)

from ..protocol.enums import ArmState, JointFlag


class HostPanel(QWidget):
    def __init__(self, ctrl, joint_count: int = 7, parent=None) -> None:
        super().__init__(parent)
        self.ctrl = ctrl
        self.joint_count = joint_count
        self._connected = False
        self._limits_ready = False
        self._limits_abs: tuple[tuple[float, float], ...] | None = None
        self._limits_future = None
        self._limits_started = 0.0
        self._send_future = None
        self._send_joint: int | None = None
        self._send_kind = ""
        self._pending_targets: dict[int, float] = {}
        self._mode_active = False
        self._stop_after_send = False
        self._updating = False
        self._next_limits_try = 0.0
        self._last_sample = None
        self._feedback = ""
        self._feedback_until = 0.0

        self.state_label = QLabel("未连接")
        self.state_label.setObjectName("value")
        self.state_label.setWordWrap(True)

        self.spins: list[QDoubleSpinBox] = []
        self.sliders: list[QSlider] = []
        grid = QGridLayout()
        for i in range(joint_count):
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(-720.0, 720.0)
            spin.setSuffix(" deg")
            spin.setSingleStep(0.5)
            spin.setEnabled(False)
            slider = QSlider()
            slider.setRange(-7200, 7200)
            slider.setSingleStep(5)
            slider.setPageStep(50)
            slider.setEnabled(False)
            slider.valueChanged.connect(lambda value, joint=i: self._on_slider_changed(joint, value))
            spin.valueChanged.connect(lambda value, joint=i: self._on_spin_changed(joint, value))
            grid.addWidget(QLabel(f"J{i}"), i, 0)
            grid.addWidget(slider, i, 1)
            grid.addWidget(spin, i, 2)
            self.spins.append(spin)
            self.sliders.append(slider)

        self.joint_combo = QComboBox()
        for i in range(joint_count):
            self.joint_combo.addItem(f"J{i}", i)
        self.read_btn = QPushButton("读取限位")
        self.current_btn = QPushButton("载入当前角度")
        self.mode_btn = QPushButton("进入关节角闭环")
        self.mode_btn.setCheckable(True)
        self.mode_btn.setToolTip("一次性载入当前 7 轴角度并全部投入闭环；单独拖动滑条也会直接闭环对应关节")
        self.resend_btn = QPushButton("重发全部 7 轴")
        self.send_closed_btn = QPushButton("发送并闭环")
        self.send_open_btn = QPushButton("仅发送目标")
        self.stop_btn = QPushButton("全部退出闭环")
        self.send_closed_btn.setToolTip("只更新选中关节的目标，并让该关节投入角度闭环")
        self.send_open_btn.setToolTip("只更新选中关节的目标；该关节输出零力矩，其他关节状态不变")
        self.read_btn.clicked.connect(self.request_limits)
        self.current_btn.clicked.connect(lambda: self.load_current(queue=self._mode_active))
        self.mode_btn.toggled.connect(self._on_mode_toggled)
        self.resend_btn.clicked.connect(self.resend_all)
        self.send_closed_btn.clicked.connect(lambda _=False: self.send_target(True))
        self.send_open_btn.clicked.connect(lambda _=False: self.send_target(False))
        self.stop_btn.clicked.connect(self.stop_loops)

        buttons = QHBoxLayout()
        buttons.addWidget(self.read_btn)
        buttons.addWidget(self.current_btn)
        buttons.addStretch(1)
        mode_row = QHBoxLayout()
        mode_row.addWidget(self.mode_btn)
        mode_row.addWidget(self.resend_btn)
        mode_row.addWidget(self.stop_btn)
        mode_row.addStretch(1)
        send_row = QHBoxLayout()
        send_row.addWidget(QLabel("发送关节"))
        send_row.addWidget(self.joint_combo)
        send_row.addWidget(self.send_closed_btn)
        send_row.addWidget(self.send_open_btn)
        send_row.addStretch(1)
        note = QLabel("拖动滑条会直接向对应关节发送闭环目标，不需要先进入模式。"
                      "“进入关节角闭环”会载入当前角度并把 7 轴一次性全部投入闭环。"
                      "下方单关节按钮用于调试，未选中的关节保持原目标和原闭环状态。"
                      "目标以多圈角发送，必须位于当前圈锁存的固件限位内；拨杆左上 + 右下时才接受。")
        note.setObjectName("title")
        note.setWordWrap(True)
        lay = QVBoxLayout(self)
        lay.addWidget(self.state_label)
        lay.addLayout(buttons)
        lay.addLayout(mode_row)
        lay.addLayout(grid)
        lay.addLayout(send_row)
        lay.addWidget(note)
        lay.addStretch(1)

        self._send_timer = QTimer(self)
        self._send_timer.setInterval(10)
        self._send_timer.timeout.connect(self._pump_send_queue)
        self._update_buttons()

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        if not connected:
            self._deactivate_mode()
            self._stop_after_send = False
            self._limits_ready = False
            self._limits_abs = None
            self._limits_future = None
            self._limits_started = 0.0
            self._send_future = None
            self._send_joint = None
            self._send_kind = ""
            for spin in self.spins:
                spin.setEnabled(False)
            for slider in self.sliders:
                slider.setEnabled(False)
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

    def load_current(self, queue: bool = False) -> None:
        sample = self._last_sample
        if sample is None:
            QMessageBox.information(self, "载入当前角度", "还没有收到关节数据")
            return
        self._updating = True
        try:
            self._refresh_spin_ranges()
            for joint, (spin, joint_state) in enumerate(zip(self.spins, sample.joints)):
                spin.setValue(float(joint_state.angle_deg))
                self.sliders[joint].setValue(round(spin.value() * 10.0))
        finally:
            self._updating = False
        if queue and self._mode_active:
            self._queue_all_targets()

    def resend_all(self) -> None:
        if not self._mode_active:
            return
        self._queue_all_targets()
        self._set_feedback("正在重发全部 7 轴闭环目标…")

    def _on_mode_toggled(self, checked: bool) -> None:
        if not checked:
            was_active = self._mode_active
            self._deactivate_mode()
            if was_active and self._connected and self._is_host_state():
                self._poll_send_future()
                if self._send_future is None:
                    self.stop_loops()
                else:
                    self._stop_after_send = True
                    self._send_timer.start()
            return
        if not self._connected:
            self._reject_mode("未连接，不能进入关节角闭环")
            return
        if not self._limits_ready:
            self._reject_mode("固件限位尚未载入，不能进入关节角闭环")
            return
        if not self._is_host_state():
            self._reject_mode("请先退出 IK，并在 HOST 状态进入关节角闭环")
            return
        if self._send_future is not None:
            self._reject_mode("还有目标在发送，请稍后再进入")
            return

        self.load_current()
        self._mode_active = True
        self._set_mode_checked(True)
        self._queue_all_targets()
        self._set_feedback("关节角闭环：正在载入当前角度并闭环全部 7 轴…")
        self._update_buttons()

    def _reject_mode(self, text: str) -> None:
        self._mode_active = False
        self._set_mode_checked(False)
        self._set_feedback(text, 3.0)
        self._update_buttons()

    def _deactivate_mode(self) -> None:
        self._mode_active = False
        self._pending_targets.clear()
        self._send_timer.stop()
        self._set_mode_checked(False)

    def _set_mode_checked(self, checked: bool) -> None:
        self.mode_btn.blockSignals(True)
        self.mode_btn.setChecked(checked)
        self.mode_btn.blockSignals(False)
        self.mode_btn.setText("退出关节角闭环" if checked else "进入关节角闭环")

    def _on_spin_changed(self, joint: int, value: float) -> None:
        if self._updating:
            return
        self._updating = True
        try:
            self.sliders[joint].setValue(round(value * 10.0))
        finally:
            self._updating = False
        if self._mode_active:
            self._queue_joint(joint)

    def _on_slider_changed(self, joint: int, value: int) -> None:
        if self._updating:
            return
        self._updating = True
        try:
            self.spins[joint].setValue(value / 10.0)
        finally:
            self._updating = False
        self._queue_joint(joint)

    def _queue_joint(self, joint: int) -> None:
        if not self._connected or not self._limits_ready or not self._is_host_state():
            return
        self._pending_targets[joint] = float(self.spins[joint].value())
        if self._send_future is None:
            self._pump_send_queue()
        else:
            self._send_timer.start()

    def _queue_all_targets(self) -> None:
        if not self._connected or not self._limits_ready or not self._is_host_state():
            return
        for joint, spin in enumerate(self.spins):
            self._pending_targets[joint] = float(spin.value())
        if self._send_future is None:
            self._pump_send_queue()
        else:
            self._send_timer.start()

    def send_target(self, closed_loop: bool) -> None:
        if not self._is_host_state():
            QMessageBox.warning(self, "Host 目标", "当前不是 Host 模式，请先把拨杆切到左上 + 右下")
            return
        if not self._limits_ready:
            QMessageBox.warning(self, "Host 目标", "固件限位尚未读取，不能发送目标")
            return
        if self._mode_active:
            self._set_feedback("关节角闭环模式下请直接拖动滑条，或先退出该模式")
            return
        joint = int(self.joint_combo.currentData())
        self._send_future = self._start_target(joint, self.spins[joint].value(), closed_loop)
        if self._send_future is None:
            return
        action = "闭环目标" if closed_loop else "不闭环目标"
        self._set_feedback(f"J{joint} {action}已发送，等待固件确认…")
        self._update_buttons()

    def stop_loops(self) -> None:
        if not self._is_host_state():
            return
        self._deactivate_mode()
        self._stop_after_send = False
        self._send_future = self.ctrl.stop_host_loops()
        if self._send_future is not None:
            self._send_joint = None
            self._send_kind = "stop"
            self._set_feedback("正在退出全部关节闭环…")
            self._update_buttons()

    def _start_target(self, joint: int, target_deg: float, closed_loop: bool = True,
                      kind: str = "target"):
        try:
            future = self.ctrl.set_host_target(joint, target_deg, closed_loop)
        except Exception as exc:
            self._set_feedback(f"J{joint} 目标发送失败：{exc}", 3.0)
            return None
        if future is not None:
            self._send_joint = joint
            self._send_kind = kind
        return future

    def _pump_send_queue(self) -> None:
        self._poll_send_future()
        if self._send_future is not None:
            return
        if not self._pending_targets:
            self._send_timer.stop()
            return
        if not self._connected or not self._limits_ready or not self._is_host_state():
            self._pending_targets.clear()
            self._send_timer.stop()
            if self._mode_active:
                self._deactivate_mode()
                self._set_feedback("关节角闭环已退出：当前不是 HOST", 3.0)
            return

        joint = next(iter(self._pending_targets))
        target_deg = self._pending_targets.pop(joint)
        self._send_future = self._start_target(joint, target_deg, True, "direct")
        if self._send_future is None:
            self._deactivate_mode()
            if self._is_host_state():
                self.stop_loops()
            return
        self._send_timer.start()
        self._update_buttons()

    def _poll_send_future(self) -> None:
        if self._send_future is None or not self._send_future.done():
            return
        future = self._send_future
        joint = self._send_joint
        kind = self._send_kind
        self._send_future = None
        self._send_joint = None
        self._send_kind = ""
        try:
            future.result()
        except Exception as exc:
            if kind == "target" and self._mode_active:
                self._deactivate_mode()
                if self._is_host_state():
                    self.stop_loops()
            label = f"J{joint} " if joint is not None else ""
            self._set_feedback(f"{label}目标被拒绝：{exc}", 3.0)
        else:
            if kind == "stop":
                self._set_feedback("全部关节已退出闭环")
            elif kind == "target" and not self._mode_active:
                self._set_feedback("固件已确认目标")
        if self._stop_after_send and self._send_future is None:
            if self._is_host_state():
                self.stop_loops()
            else:
                self._stop_after_send = False
        self._update_buttons()

    def _set_feedback(self, text: str, seconds: float = 2.0) -> None:
        self._feedback = text
        self._feedback_until = time.monotonic() + seconds
        self.state_label.setText(text)

    @staticmethod
    def _wrap_single_turn(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    def _joint_multi_limits(self, joint: int) -> tuple[float, float] | None:
        if self._limits_abs is None:
            return None
        sample = self._last_sample
        if sample is not None and joint < len(sample.joints):
            multi_deg = float(sample.joints[joint].angle_deg)
        else:
            multi_deg = float(self.spins[joint].value())
        abs_deg = self._wrap_single_turn(multi_deg)
        limit_min_abs, limit_max_abs = self._limits_abs[joint]
        return (multi_deg + (limit_min_abs - abs_deg),
                multi_deg + (limit_max_abs - abs_deg))

    def _refresh_spin_ranges(self) -> None:
        if self._limits_abs is None:
            return
        for joint, spin in enumerate(self.spins):
            limits = self._joint_multi_limits(joint)
            if limits is None or not limits[0] < limits[1]:
                continue
            if (abs(spin.minimum() - limits[0]) > 1e-6 or
                    abs(spin.maximum() - limits[1]) > 1e-6):
                was_updating = self._updating
                self._updating = True
                try:
                    spin.setRange(limits[0], limits[1])
                    self.sliders[joint].setRange(round(limits[0] * 10.0), round(limits[1] * 10.0))
                    self.sliders[joint].setValue(round(spin.value() * 10.0))
                finally:
                    self._updating = was_updating

    def _is_host_state(self) -> bool:
        return self._last_sample is not None and self._last_sample.arm_state == ArmState.HOST

    def _is_host_mode(self) -> bool:
        return self._last_sample is not None and self._last_sample.arm_state in (
            ArmState.HOST, ArmState.HOST_IK)

    def _update_buttons(self) -> None:
        host = self._is_host_state()
        busy = self._send_future is not None
        can_mode = self._connected and self._limits_ready and host and not busy
        self.mode_btn.setEnabled(self._mode_active or can_mode)
        self.resend_btn.setEnabled(self._mode_active and host and not busy)
        self.send_closed_btn.setEnabled(can_mode and not self._mode_active)
        self.send_open_btn.setEnabled(can_mode and not self._mode_active)
        self.stop_btn.setEnabled(self._connected and self._is_host_mode() and not busy)
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
                    for i, (lo, hi) in enumerate(zip(mins, maxs)):
                        if not lo < hi:
                            raise ValueError(f"J{i} 限位无效：{lo} .. {hi}")
                    self._limits_abs = tuple((float(lo), float(hi)) for lo, hi in zip(mins, maxs))
                    self._refresh_spin_ranges()
                    for spin in self.spins:
                        spin.setEnabled(True)
                    for slider in self.sliders:
                        slider.setEnabled(True)
                    self._limits_ready = True
                    if self._is_host_state():
                        self.load_current()
                    self._set_feedback("固件限位已载入")
                except Exception as e:
                    self._limits_ready = False
                    self._limits_abs = None
                    for spin in self.spins:
                        spin.setEnabled(False)
                    for slider in self.sliders:
                        slider.setEnabled(False)
                    self._set_feedback(f"读取限位失败：{e}", 3.0)
                    self._next_limits_try = now + 2.0
            elif now - self._limits_started > 5.0:
                self._limits_future = None
                future.add_done_callback(lambda f: f.exception())
                self._limits_ready = False
                self._limits_abs = None
                for spin in self.spins:
                    spin.setEnabled(False)
                for slider in self.sliders:
                    slider.setEnabled(False)
                self._set_feedback("限位读取超时，请重试", 3.0)
                self._next_limits_try = now + 2.0

        self._poll_send_future()
        self._update_buttons()

    def tick(self, sample) -> None:
        previous = self._last_sample
        self._last_sample = sample
        if self._limits_ready and sample is not None and sample.arm_state == ArmState.HOST \
                and (previous is None or previous.arm_state != ArmState.HOST):
            self.load_current()
        if self._mode_active and (sample is None or sample.arm_state != ArmState.HOST):
            self._deactivate_mode()
            reason = "IK 已接管" if sample is not None and sample.arm_state == ArmState.HOST_IK else "当前不是 HOST"
            self._set_feedback(f"关节角闭环已退出：{reason}", 3.0)
        if self._limits_ready:
            self._refresh_spin_ranges()
        self._poll_futures()
        if self._connected and sample is not None and not self._limits_ready \
                and self._limits_future is None and time.monotonic() >= self._next_limits_try:
            self.request_limits()

        if self._feedback and time.monotonic() < self._feedback_until:
            self.state_label.setText(self._feedback)
        elif sample is None:
            if self._connected:
                self.state_label.setText("已连接，等待关节数据")
        elif sample.arm_state in (ArmState.HOST, ArmState.HOST_IK):
            if not self._limits_ready:
                if self._limits_future is not None:
                    self.state_label.setText("Host 模式：正在读取固件限位，发送暂不可用")
                else:
                    self.state_label.setText("Host 模式：限位未载入，发送暂不可用")
            else:
                closed = sum(bool(j.joint_flags & JointFlag.HOST_CLOSED_LOOP) for j in sample.joints)
                valid = sum(bool(j.joint_flags & JointFlag.HOST_TARGET_VALID) for j in sample.joints)
                mode = "Host IK" if sample.arm_state == ArmState.HOST_IK else "Host"
                if self._mode_active:
                    mode = "关节角闭环"
                self.state_label.setText(f"{mode} 模式：{closed}/{self.joint_count} 关节闭环，{valid} 个已设目标")
        else:
            state = ArmState(sample.arm_state).name if sample.arm_state in ArmState._value2member_map_ else str(sample.arm_state)
            self.state_label.setText(f"当前 {state}，切到左上 + 右下后可用")
        self._update_buttons()
