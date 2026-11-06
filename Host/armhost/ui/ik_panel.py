"""HOST_IK 控制面板：滑条编辑末端位姿 / ψ，并显示当前分支的可达 ψ 区间。"""

from __future__ import annotations

import time
from concurrent.futures import Future

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (QCheckBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)

from ..kinematics import (IkStatus, feasible_psi_intervals, make_srs_param, solve_ik,
                          transform_from_xyz_rpy)
from ..model import branch_text
from ..protocol.enums import ArmState, PostureFlag

_PSI_RANGE = "ψ 可达区间（关节限位 + 非奇异）"


class WrappedSlider(QSlider):
    """水平滑条：拖动、滚轮或方向键越过端点时，按 [-180°, 180°] 周期环绕。"""

    def __init__(self, lo: int, hi: int, parent=None) -> None:
        super().__init__(Qt.Horizontal, parent)
        self._lo = lo
        self._hi = hi
        self._period = hi - lo
        self.setRange(lo, hi)
        self._dragging = False
        self._last_x = 0.0
        self._drag_accum = 0.0

    def _wrapped(self, value: int) -> int:
        if self._lo <= value <= self._hi:
            return value
        return self._lo + (value - self._lo) % self._period

    def shift_steps(self, steps: int) -> None:
        if steps:
            self.setValue(self._wrapped(self.value() + steps))

    def mousePressEvent(self, event) -> None:
        super().mousePressEvent(event)
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._last_x = event.globalPosition().x()
            self._drag_accum = 0.0
            event.accept()

    def mouseMoveEvent(self, event) -> None:
        if not self._dragging or not (event.buttons() & Qt.LeftButton):
            super().mouseMoveEvent(event)
            return
        x = event.globalPosition().x()
        dx = x - self._last_x
        self._last_x = x
        if self.width() > 0:
            self._drag_accum += dx * self._period / self.width()
            steps = int(self._drag_accum)
            if steps:
                self._drag_accum -= steps
                self.shift_steps(steps)
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        self._dragging = False
        self._drag_accum = 0.0
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        if not delta:
            super().wheelEvent(event)
            return
        steps = max(1, abs(delta) // 120) * self.singleStep()
        self.shift_steps(steps if delta > 0 else -steps)
        event.accept()

    def keyPressEvent(self, event) -> None:
        step = self.singleStep()
        if event.key() in (Qt.Key_Right, Qt.Key_Up):
            self.shift_steps(step)
        elif event.key() in (Qt.Key_Left, Qt.Key_Down):
            self.shift_steps(-step)
        elif event.key() == Qt.Key_PageUp:
            self.shift_steps(self.pageStep())
        elif event.key() == Qt.Key_PageDown:
            self.shift_steps(-self.pageStep())
        else:
            super().keyPressEvent(event)
            return
        event.accept()


class ScaledSlider(QWidget):
    """浮点量 -> QSlider 整数刻度，右侧显示当前值。"""
    valueChanged = Signal(float)

    def __init__(self, lo: float, hi: float, scale: int, step: float, decimals: int,
                 suffix: str, parent=None, wrap: bool = False) -> None:
        super().__init__(parent)
        self._scale = scale
        self._decimals = decimals
        self._suffix = suffix
        if wrap:
            self.slider = WrappedSlider(round(lo * scale), round(hi * scale))
        else:
            self.slider = QSlider(Qt.Horizontal)
            self.slider.setRange(round(lo * scale), round(hi * scale))
        single = max(1, round(step * scale))
        self.slider.setSingleStep(single)
        self.slider.setPageStep(max(single, round(step * scale * 10)))
        self.value_label = QLabel()
        self.value_label.setObjectName("value")
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value_label.setMinimumWidth(78)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.slider, 1)
        lay.addWidget(self.value_label)
        self.slider.valueChanged.connect(self._changed)
        self._changed(self.slider.value())

    def _changed(self, _value: int) -> None:
        self.value_label.setText(f"{self.value():+.{self._decimals}f}{self._suffix}")
        self.valueChanged.emit(self.value())

    def value(self) -> float:
        return self.slider.value() / self._scale

    def setValue(self, value: float) -> None:
        self.slider.setValue(round(float(value) * self._scale))


class IkPanel(QWidget):
    def __init__(self, ctrl, parent=None) -> None:
        super().__init__(parent)
        self.ctrl = ctrl
        self._connected = False
        self._supported = False
        self._future = None
        self._future_started = 0.0
        self._future_quiet = False
        self._auto_dirty = False
        self._last_sample = None
        self._branch: int | None = None
        self._feedback = ""
        self._feedback_until = 0.0

        self._generation = 0
        self._param_token = 0
        self._dh = None
        self._limits: tuple[tuple[float, float], ...] | None = None
        self._param_future: Future | None = None
        self._param_started = 0.0
        self._next_param_try = 0.0
        self._param_error = ""
        self._updating = False
        self._initialized = False
        self._last_range_key = None

        self.state_label = QLabel("未连接")
        self.state_label.setObjectName("value")
        self.state_label.setWordWrap(True)
        self.branch_label = QLabel("—")
        self.branch_label.setObjectName("value")

        posture = QGroupBox("当前构型")
        pg = QFormLayout(posture)
        pg.addRow("分支", self.branch_label)
        self.ik_status = QLabel("—")
        self.ik_status.setObjectName("value")
        pg.addRow("逆解状态", self.ik_status)

        target = QGroupBox("目标（腰部连杆坐标系）")
        grid = QGridLayout(target)
        self.fields: dict[str, ScaledSlider] = {}
        self.psi_range_label = QLabel(f"{_PSI_RANGE}：等待设备 DH / 关节限位")
        self.psi_range_label.setObjectName("value")
        self.psi_range_label.setWordWrap(True)
        grid.addWidget(self.psi_range_label, 0, 0, 1, 2)
        rows = (
            ("x", "X", -2.0, 2.0, 1000, 0.005, 3, " m"),
            ("y", "Y", -2.0, 2.0, 1000, 0.005, 3, " m"),
            ("z", "Z", -2.0, 2.0, 1000, 0.005, 3, " m"),
            ("roll", "Roll", -180.0, 180.0, 10, 0.5, 1, "°"),
            ("pitch", "Pitch", -180.0, 180.0, 10, 0.5, 1, "°"),
            ("yaw", "Yaw", -180.0, 180.0, 10, 0.5, 1, "°"),
            ("psi", "臂角 ψ", -180.0, 180.0, 10, 0.5, 1, "°"),
        )
        for i, (key, text, lo, hi, scale, step, decimals, suffix) in enumerate(rows):
            widget = ScaledSlider(lo, hi, scale, step, decimals, suffix, wrap=key == "psi")
            if key == "psi":
                widget.setToolTip("可连续拖动；越过 +180° 或 -180° 后自动从另一端继续")
            widget.setEnabled(False)
            widget.valueChanged.connect(self._on_pose_changed)
            grid.addWidget(QLabel(text), i + 1, 0)
            grid.addWidget(widget, i + 1, 1)
            self.fields[key] = widget

        self.auto_psi = QCheckBox("自动选择 ψ（最小加权关节位移）")
        self.auto_psi.setToolTip(
            "保持当前分支，在目标位姿下满足关节限位和非奇异条件的 ψ 采样点中，"
            "按 J0~J6 加权代价选择")
        self.auto_psi.toggled.connect(self._on_auto_psi_changed)
        grid.addWidget(self.auto_psi, len(rows) + 1, 0, 1, 2)

        weights_box = QGroupBox("自动 ψ 代价权重（deg²）")
        weights_grid = QGridLayout(weights_box)
        self.weight_spins: list[QDoubleSpinBox] = []
        for i in range(7):
            spin = QDoubleSpinBox()
            spin.setRange(0.0, 100.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            spin.setValue(1.0)
            spin.setPrefix(f"J{i} ")
            spin.setMaximumWidth(96)
            spin.valueChanged.connect(self._on_cost_weights_changed)
            weights_grid.addWidget(spin, i // 4, i % 4)
            self.weight_spins.append(spin)

        self.load_btn = QPushButton("载入当前位姿 / ψ")
        self.apply_btn = QPushButton("立即重发当前目标")
        self.toggle_btn = QPushButton("进入 IK 闭环")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setToolTip("从 HOST 进入 HOST_IK；再次点击退出并回到 HOST")
        self.load_btn.clicked.connect(self.load_current)
        self.apply_btn.clicked.connect(self.send_target)
        self.toggle_btn.clicked.connect(self._on_toggle)

        buttons = QHBoxLayout()
        buttons.addWidget(self.load_btn)
        buttons.addWidget(self.apply_btn)
        buttons.addWidget(self.toggle_btn)
        buttons.addStretch(1)

        note = QLabel("IK 只保持进入时的当前分支；改变分支需先退出 IK、让实际构型进入新分支后再进入。"
                      "位姿和 ψ 会一起送入 S-R-S 解析逆解，解出的 7 个关节目标投入闭环。"
                      "自动 ψ 模式在当前分支的可行 ψ 中按 J0~J6 加权关节位移代价选择。"
                      "进入 IK 后拖动任意滑条会自动合并发送，不需要再点发送按钮。"
                      "ψ 可达区间由 Host 使用设备实际 DH、关节限位和肩/腕非奇异条件预览，"
                      "最终仍以固件校验为准。")
        note.setObjectName("title")
        note.setWordWrap(True)

        lay = QVBoxLayout(self)
        lay.addWidget(self.state_label)
        lay.addWidget(posture)
        lay.addWidget(target)
        lay.addWidget(weights_box)
        lay.addLayout(buttons)
        lay.addWidget(note)
        lay.addStretch(1)

        self._range_timer = QTimer(self)
        self._range_timer.setSingleShot(True)
        self._range_timer.setInterval(60)
        self._range_timer.timeout.connect(self._update_psi_range)
        self._auto_timer = QTimer(self)
        self._auto_timer.setSingleShot(True)
        self._auto_timer.setInterval(50)
        self._auto_timer.timeout.connect(self._auto_send)
        self._update_enabled()

    # ------------------------------------------------------------ 连接 / 命令
    def set_connected(self, connected: bool, supported: bool = True) -> None:
        now = time.monotonic()
        self._connected = connected and supported
        self._supported = connected and supported
        self._generation += 1
        self._param_token += 1
        self._future = None
        self._future_quiet = False
        self._auto_dirty = False
        self._auto_timer.stop()
        self._param_future = None
        self._dh = None
        self._limits = None
        self._param_error = ""
        self._initialized = False
        self._last_range_key = None
        if not self._connected:
            self._last_sample = None
            self._branch = None
            self.toggle_btn.setChecked(False)
            self.state_label.setText("未连接" if not connected else "当前数据源不支持 IK 闭环")
            self.branch_label.setText("—")
            self.ik_status.setText("—")
            self.psi_range_label.setText(f"{_PSI_RANGE}：等待连接")
            for widget in self.fields.values():
                widget.setEnabled(False)
        else:
            self._next_param_try = now
            self.psi_range_label.setText(f"{_PSI_RANGE}：正在读取设备 DH / 关节限位…")
            self._start_param_load()
        self._update_enabled()

    def load_current(self) -> bool:
        sample = self._last_sample
        posture = sample.posture if sample is not None else None
        if posture is None or not posture.valid:
            self._set_feedback("还没有有效的 ARM_POSTURE 或当前 ψ 无定义")
            return False
        values = {"x": posture.x, "y": posture.y, "z": posture.z, "roll": posture.roll,
                  "pitch": posture.pitch, "yaw": posture.yaw, "psi": posture.psi_deg}
        self._updating = True
        for key, value in values.items():
            self.fields[key].setValue(value)
        self._updating = False
        self._set_feedback("已载入当前位姿 / ψ")
        self._schedule_range()
        return True

    def send_target(self, allow_enter: bool = False, auto: bool = False) -> None:
        if auto and not self._is_host_ik():
            self._auto_dirty = False
            return
        branch = self._branch
        if branch is None:
            if not auto:
                self._set_feedback("没有有效分支，无法发送 IK 目标")
            return
        values = {k: widget.value() for k, widget in self.fields.items()}
        weights = tuple(spin.value() for spin in self.weight_spins)
        auto_psi = self.auto_psi.isChecked()
        if auto_psi and sum(weights) <= 0.0:
            self._set_feedback("自动 ψ 至少需要一个非零代价权重")
            return
        future = self.ctrl.set_ik_target(int(branch), values["x"], values["y"], values["z"],
                                         values["roll"], values["pitch"], values["yaw"], values["psi"],
                                         auto_psi, weights)
        self._set_future(future, "" if auto else "正在发送 IK 目标…", quiet=auto)

    def _on_auto_psi_changed(self, _checked: bool) -> None:
        self._schedule_range()
        self._queue_auto_send()

    def _on_cost_weights_changed(self, _value: float) -> None:
        if self.auto_psi.isChecked():
            self._queue_auto_send()

    def _queue_auto_send(self) -> None:
        if self._is_host_ik():
            self._auto_dirty = True
            if self._future is None and not self._auto_timer.isActive():
                self._auto_timer.start()
        self._update_enabled()

    def _auto_send(self) -> None:
        if not self._is_host_ik():
            self._auto_dirty = False
            return
        if self._future is not None or not self._auto_dirty:
            return
        self._auto_dirty = False
        self.send_target(auto=True)

    def _on_toggle(self, checked: bool) -> None:
        sample = self._last_sample
        if checked:
            if sample is None or sample.arm_state != ArmState.HOST:
                self._set_feedback("请先在 HOST 状态（拨杆左上 + 右下）进入 IK")
                self.toggle_btn.setChecked(False)
                return
            if not self.load_current():
                self.toggle_btn.setChecked(False)
                return
            self.send_target(allow_enter=True)
        else:
            self._auto_timer.stop()
            self._auto_dirty = False
            if sample is not None and sample.arm_state == ArmState.HOST_IK:
                self._set_future(self.ctrl.stop_ik(), "正在退出 IK 闭环…")

    # ------------------------------------------------------------ 设备 DH / 限位
    def _start_param_load(self) -> None:
        if not self._supported or self._param_future is not None:
            return
        gen = self._generation
        token = self._param_token + 1
        self._param_token = token
        future: Future = Future()
        self._param_future = future
        self._param_started = time.monotonic()

        def fail(exc: Exception) -> None:
            if gen == self._generation and token == self._param_token and not future.done():
                future.set_exception(exc)

        try:
            limits_future = self.ctrl.read_limits()
            if limits_future is None:
                fail(RuntimeError("当前数据源不支持读取关节限位"))
                return
        except Exception as exc:
            fail(exc)
            return

        def limits_done(f) -> None:
            if gen != self._generation or token != self._param_token or future.done():
                return
            try:
                mins, maxs = f.result()
                if len(mins) != 7 or len(maxs) != 7:
                    raise ValueError("关节数量不是 7")
                limits = tuple((float(lo), float(hi)) for lo, hi in zip(mins, maxs))
            except Exception as exc:
                fail(exc)
                return
            try:
                dh_future = self.ctrl.read_dh()
                if dh_future is None:
                    raise RuntimeError("当前数据源不支持读取 DH")
            except Exception as exc:
                fail(exc)
                return

            def dh_done(g) -> None:
                if gen != self._generation or token != self._param_token or future.done():
                    return
                try:
                    dh = g.result()
                except Exception as exc:
                    fail(exc)
                    return
                future.set_result((dh, limits))

            dh_future.add_done_callback(dh_done)

        limits_future.add_done_callback(limits_done)

    def _poll_param_load(self) -> None:
        future = self._param_future
        if future is None:
            return
        now = time.monotonic()
        if not future.done():
            if now - self._param_started > 12.0:
                self._param_future = None
                self._param_token += 1
                self._param_error = "读取设备 DH / 限位超时，稍后重试"
                self._next_param_try = now + 3.0
                self.psi_range_label.setText(f"{_PSI_RANGE}：{self._param_error}")
            return
        self._param_future = None
        try:
            self._dh, self._limits = future.result()
            self._param_error = ""
            self._schedule_range()
        except Exception as exc:
            self._dh = None
            self._limits = None
            self._param_error = f"读取设备 DH / 限位失败：{exc}"
            self._next_param_try = now + 3.0
            self.psi_range_label.setText(f"{_PSI_RANGE}：{self._param_error}")

    # ------------------------------------------------------------ 状态刷新
    def tick(self, sample) -> None:
        self._last_sample = sample
        self._poll_future()
        self._poll_param_load()
        now = time.monotonic()
        if self._supported and self._limits is None and self._param_future is None \
                and now >= self._next_param_try:
            self._start_param_load()
        if not self._supported:
            self._update_enabled()
            return

        state = sample.arm_state if sample is not None else None
        if state != ArmState.HOST_IK:
            self._auto_timer.stop()
            self._auto_dirty = False
        posture = sample.posture if sample is not None else None
        branch = posture.branch if posture is not None and posture.valid else None
        if posture is not None and posture.valid and not self._initialized:
            self._initialized = True
            self.load_current()
        if branch != self._branch:
            self._branch = branch
            self._last_range_key = None
            self._schedule_range()
        self.branch_label.setText(branch_text(branch))
        if state == ArmState.HOST_IK:
            self.toggle_btn.setChecked(True)
            self.toggle_btn.setText("退出 IK 闭环")
        elif state == ArmState.HOST:
            self.toggle_btn.setChecked(False)
            self.toggle_btn.setText("进入 IK 闭环")
        else:
            self.toggle_btn.setChecked(False)
            self.toggle_btn.setText("进入 IK 闭环")

        if posture is not None:
            flags = PostureFlag(posture.flags)
            singular = []
            if PostureFlag.SHOULDER_SINGULAR in flags:
                singular.append("肩奇异")
            if PostureFlag.WRIST_SINGULAR in flags:
                singular.append("腕奇异")
            status = ("IK 闭环" if PostureFlag.IK_ACTIVE in flags else "关节目标") + \
                     ("，自动 ψ" if PostureFlag.AUTO_PSI in flags else "") + \
                     (f"，{'/'.join(singular)}" if singular else "")
            self.ik_status.setText(status)
        else:
            self.ik_status.setText("—")

        if self._feedback and time.monotonic() < self._feedback_until:
            self.state_label.setText(self._feedback)
        elif sample is None:
            self.state_label.setText("已连接，等待 ARM_POSTURE")
        elif state == ArmState.HOST_IK:
            mode = "自动 ψ" if self.auto_psi.isChecked() else "手动 ψ"
            self.state_label.setText(f"HOST_IK：{mode}，拖动位姿滑条会自动发送，逆解结果保持当前分支")
        elif state == ArmState.HOST:
            self.state_label.setText("HOST：可载入当前位姿后进入 IK 闭环")
        else:
            name = ArmState(state).name if state in ArmState._value2member_map_ else str(state)
            self.state_label.setText(f"当前 {name}；请先切到 HOST")
        self._update_enabled()

    # ------------------------------------------------------------ ψ 可达域
    def _on_pose_changed(self, _value: float) -> None:
        if not self._updating:
            self._schedule_range()
            if self._is_host_ik():
                self._auto_dirty = True
                if self._future is None and not self._auto_timer.isActive():
                    self._auto_timer.start()

    def _schedule_range(self) -> None:
        if self._supported and not self._range_timer.isActive():
            self._range_timer.start()

    def _range_key(self):
        return (self._branch, tuple(round(widget.value(), 4) for widget in self.fields.values()))

    def _update_psi_range(self) -> None:
        if not self._supported:
            return
        if self._dh is None or self._limits is None:
            if self._param_error:
                self.psi_range_label.setText(f"{_PSI_RANGE}：{self._param_error}")
            return
        sample = self._last_sample
        posture = sample.posture if sample is not None else None
        branch = posture.branch if posture is not None else None
        if branch is None or not 0 <= int(branch) < 8:
            self.psi_range_label.setText(f"{_PSI_RANGE}：没有有效分支")
            return
        param = make_srs_param(self._dh)
        if param is None:
            self.psi_range_label.setText(
                f"{_PSI_RANGE}：当前 DH 不满足 S-R-S 构型或大/小臂长度无效")
            return
        if sample is None or len(sample.joints) != 7:
            self.psi_range_label.setText(f"{_PSI_RANGE}：等待 7 轴关节数据")
            return

        values = {k: widget.value() for k, widget in self.fields.items()}
        target = transform_from_xyz_rpy(values["x"], values["y"], values["z"],
                                        values["roll"], values["pitch"], values["yaw"])
        key = self._range_key()
        intervals = feasible_psi_intervals(param, target, int(branch), self._limits,
                                           tuple(j.angle_deg for j in sample.joints))
        self._last_range_key = key
        prefix = f"分支 {branch_text(branch)}，"
        if not intervals:
            probe = solve_ik(param, target, int(branch), values["psi"])
            if probe.status == IkStatus.OUT_OF_REACH:
                text = "无解：腕心超出当前大/小臂可达范围"
            elif probe.status == IkStatus.REF_DEGENERATE:
                text = "ψ 无定义：S→W 与基座 z0 近似平行"
            else:
                text = "当前位姿有 IK 解，但没有同时满足关节限位和非奇异条件的 ψ"
            self.psi_range_label.setText(f"{_PSI_RANGE}：{prefix}{text}")
            return
        parts = " ∪ ".join(f"[{lo:+.1f}°, {hi:+.1f}°]" for lo, hi in intervals)
        self.psi_range_label.setText(f"{_PSI_RANGE}：{prefix}{parts}")

    # ------------------------------------------------------------ 异步命令
    def _set_future(self, future, message: str, quiet: bool = False) -> None:
        if future is None:
            return
        self._future = future
        self._future_started = time.monotonic()
        self._future_quiet = quiet
        if not quiet:
            self._set_feedback(message, 10.0)
        self._update_enabled()

    def _poll_future(self) -> None:
        if self._future is None:
            return
        if not self._future.done():
            if time.monotonic() - self._future_started > 2.0:
                self._set_feedback("IK 命令等待应答…", 2.0)
            return
        future, self._future = self._future, None
        quiet, self._future_quiet = self._future_quiet, False
        try:
            future.result()
            if not quiet:
                self._set_feedback("固件已确认 IK 目标")
        except Exception as exc:
            self._set_feedback(f"IK 目标被拒绝：{exc}", 3.0)
        self._update_enabled()
        if self._auto_dirty and self._is_host_ik() and not self._auto_timer.isActive():
            self._auto_timer.start()

    def _is_host_ik(self) -> bool:
        return self._last_sample is not None and self._last_sample.arm_state == ArmState.HOST_IK

    def _set_feedback(self, text: str, seconds: float = 2.0) -> None:
        self._feedback = text
        self._feedback_until = time.monotonic() + seconds
        self.state_label.setText(text)

    def _update_enabled(self) -> None:
        sample = self._last_sample
        state = sample.arm_state if sample is not None else None
        host = state in (ArmState.HOST, ArmState.HOST_IK)
        posture_ok = sample is not None and sample.posture is not None and sample.posture.valid
        busy = self._future is not None
        self.load_btn.setEnabled(self._supported and posture_ok and not busy)
        self.toggle_btn.setEnabled(self._supported and host and not busy)
        self.apply_btn.setEnabled(self._supported and state == ArmState.HOST_IK and posture_ok and not busy)
        self.auto_psi.setEnabled(self._supported and host)
        auto_psi = self.auto_psi.isChecked()
        for key, widget in self.fields.items():
            widget.setEnabled(self._supported and state == ArmState.HOST_IK
                              and not (key == "psi" and auto_psi))
        for spin in self.weight_spins:
            spin.setEnabled(self._supported and host and auto_psi)
