"""HOST_IK 控制面板：滑条编辑 TCP 位姿 / ψ，并显示当前分支的可达 ψ 区间。"""

from __future__ import annotations

import math
import time
from concurrent.futures import Future

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)

from ..kinematics import (IkStatus, feasible_psi_intervals, make_srs_param, solve_ik,
                          transform_from_xyz_rpy)
from ..model import branch_text
from ..protocol.enums import ArmState, IkMethod, PostureFlag
from . import theme

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
        self._numeric_plan_since_ns: int | None = None
        self._target_dirty = False
        self._last_plan_sequence = -1
        self._plan_base_sequence = -1

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

        actual = QGroupBox("实际 TCP 位姿 / 跟踪误差（腰部连杆坐标系）")
        ag = QFormLayout(actual)
        self.actual_pose_label = QLabel("—")
        self.actual_pose_label.setObjectName("value")
        self.actual_pose_label.setWordWrap(True)
        self.actual_pose_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.track_error_label = QLabel("—")
        self.track_error_label.setObjectName("value")
        self.track_error_label.setWordWrap(True)
        self.track_error_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.plan_label = QLabel("—")
        self.plan_label.setObjectName("value")
        self.plan_label.setWordWrap(True)
        self.plan_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.target_state_label = QLabel("—")
        self.target_state_label.setObjectName("value")
        self.target_state_label.setWordWrap(True)
        ag.addRow("实际", self.actual_pose_label)
        ag.addRow("误差", self.track_error_label)
        ag.addRow("数值规划", self.plan_label)
        ag.addRow("发送状态", self.target_state_label)
        error_hint = QLabel("误差 = 实际 TCP − 当前滑条目标；姿态误差取两姿态旋转矩阵的夹角。")
        error_hint.setObjectName("title")
        error_hint.setWordWrap(True)
        ag.addRow(error_hint)

        target = QGroupBox("TCP 目标（工具坐标 / 腰部连杆坐标系）")
        grid = QGridLayout(target)
        self.fields: dict[str, ScaledSlider] = {}
        self.psi_range_label = QLabel(f"{_PSI_RANGE}：等待设备 DH / 关节限位")
        self.psi_range_label.setObjectName("value")
        self.psi_range_label.setWordWrap(True)
        grid.addWidget(self.psi_range_label, 0, 0, 1, 2)
        self.method_box = QComboBox()
        self.method_box.addItem("解析解（S-R-S，ψ 可调）", int(IkMethod.ANALYTIC))
        self.method_box.addItem("数值解（加权 DLS + 零空间）", int(IkMethod.NUMERIC))
        self.method_box.setToolTip(
            "解析解：一次解出 7 个关节目标，臂角 ψ 手动给定或自动选择。\n"
            "数值解：先预计算完整关节轨迹，计算期间机械臂保持不动；完成后按轨迹执行。"
            "规划使用加权阻尼最小二乘、零空间和关节限位约束。\n"
            "数值解不受分支约束，ψ 滑条不起作用，关节权重表示该关节动多少的代价。")
        self.method_box.currentIndexChanged.connect(self._on_method_changed)
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
        grid.addWidget(QLabel("求解方式"), len(rows) + 2, 0)
        grid.addWidget(self.method_box, len(rows) + 2, 1)

        self.weights_box = weights_box = QGroupBox("自动 ψ 代价权重（deg²）")
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
        self.apply_btn = QPushButton("发送目标")
        self.toggle_btn = QPushButton("进入 IK 闭环")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setToolTip("从 HOST 进入 HOST_IK；再次点击退出并回到 HOST")
        self.load_btn.clicked.connect(self.load_current)
        self.apply_btn.clicked.connect(lambda: self.send_target())
        self.toggle_btn.clicked.connect(self._on_toggle)

        buttons = QHBoxLayout()
        buttons.addWidget(self.load_btn)
        buttons.addWidget(self.apply_btn)
        buttons.addWidget(self.toggle_btn)
        buttons.addStretch(1)

        note = QLabel("IK 只保持进入时的当前分支；改变分支需先退出 IK、让实际构型进入新分支后再进入。"
                      "x/y/z 是工具 TCP，坐标轴由当前 DH 与固定工具变换共同确定；"
                      "位姿和 ψ 会一起送入 S-R-S 解析逆解，解出的 7 个关节目标投入闭环。"
                      "自动 ψ 模式在当前分支的可行 ψ 中按 J0~J6 加权关节位移代价选择。"
                      "数值解会先计算整条关节轨迹，计算中机械臂保持不动，完成后自动执行。"
                      "修改滑条、ψ 或权重后不会自动发送；数值解点击“发送并计算”，"
                      "解析解点击“发送目标”后才更新设备目标。"
                      "ψ 可达区间由 Host 使用设备实际 DH、关节限位和肩/腕非奇异条件预览，"
                      "最终仍以固件校验为准。")
        note.setObjectName("title")
        note.setWordWrap(True)

        lay = QVBoxLayout(self)
        lay.addWidget(self.state_label)
        lay.addWidget(posture)
        lay.addWidget(actual)
        lay.addLayout(buttons)
        lay.addWidget(target)
        lay.addWidget(weights_box)
        lay.addWidget(note)
        lay.addStretch(1)

        self._range_timer = QTimer(self)
        self._range_timer.setSingleShot(True)
        self._range_timer.setInterval(60)
        self._range_timer.timeout.connect(self._update_psi_range)
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
        self._param_future = None
        self._dh = None
        self._limits = None
        self._param_error = ""
        self._initialized = False
        self._last_range_key = None
        self._numeric_plan_since_ns = None
        self._target_dirty = False
        self._last_plan_sequence = -1
        self._plan_base_sequence = -1
        if not self._connected:
            self._last_sample = None
            self._branch = None
            self.toggle_btn.setChecked(False)
            self.state_label.setText("未连接" if not connected else "当前数据源不支持 IK 闭环")
            self.branch_label.setText("—")
            self.ik_status.setText("—")
            self._clear_pose_display()
            self.plan_label.setText("—")
            self.plan_label.setStyleSheet("")
            self.target_state_label.setText("—")
            self.target_state_label.setStyleSheet("")
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
        self._set_target_dirty(False, "已载入当前位姿，尚未发送")
        self._set_feedback("已载入当前位姿 / ψ")
        self._schedule_range()
        return True

    def send_target(self, allow_enter: bool = False) -> None:
        method = self._method()
        numeric = method == IkMethod.NUMERIC
        branch = self._branch
        if branch is None and not numeric:   # 数值解从当前构型出发，不依赖分支
            self._set_feedback("没有有效分支，无法发送 IK 目标")
            return
        values = {k: widget.value() for k, widget in self.fields.items()}
        weights = tuple(spin.value() for spin in self.weight_spins)
        auto_psi = self.auto_psi.isChecked() and not numeric
        if (auto_psi or numeric) and sum(weights) <= 0.0:
            self._set_feedback("至少需要一个非零权重")
            return
        future = self.ctrl.set_ik_target(int(branch or 0), values["x"], values["y"], values["z"],
                                         values["roll"], values["pitch"], values["yaw"], values["psi"],
                                         auto_psi, weights, int(method))
        self._set_target_dirty(False, "目标已发送，等待设备应答")
        if numeric:
            self._numeric_plan_since_ns = time.monotonic_ns()
            self._plan_base_sequence = self._last_plan_sequence
            self.state_label.setText("HOST_IK：正在预计算数值轨迹，机械臂保持不动")
            self.ik_status.setText("数值轨迹计算中，机械臂保持不动")
            self.plan_label.setText("计算中…  等待设备完成数值轨迹规划")
            self.plan_label.setStyleSheet(f"color: {theme.WARN};")
        self._set_future(future, "正在发送 IK 目标…")

    def _set_target_dirty(self, dirty: bool, text: str) -> None:
        self._target_dirty = dirty
        self.target_state_label.setText(text)
        color = theme.WARN if dirty else theme.MUTED
        self.target_state_label.setStyleSheet(f"color: {color};")

    def _mark_target_dirty(self) -> None:
        if self._supported:
            self._set_target_dirty(True, "位姿或参数已修改，尚未发送")

    def _method(self) -> IkMethod:
        return IkMethod(int(self.method_box.currentData()))

    def _is_numeric(self) -> bool:
        return self._method() == IkMethod.NUMERIC

    def _on_method_changed(self, _index: int) -> None:
        self._numeric_plan_since_ns = None
        self.weights_box.setTitle("关节权重 W（越大该关节动得越少）" if self._is_numeric()
                                  else "自动 ψ 代价权重（deg²）")
        self.apply_btn.setText("发送并计算" if self._is_numeric() else "发送目标")
        self._mark_target_dirty()
        self._schedule_range()
        self._update_enabled()

    def _on_auto_psi_changed(self, _checked: bool) -> None:
        self._mark_target_dirty()
        self._schedule_range()
        self._update_enabled()

    def _on_cost_weights_changed(self, _value: float) -> None:
        if self.auto_psi.isChecked() or self._is_numeric():
            self._mark_target_dirty()
            self._update_enabled()

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
            self._numeric_plan_since_ns = None
            self._set_target_dirty(False, "IK 闭环已退出")
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
        posture = sample.posture if sample is not None else None
        branch = posture.branch if posture is not None and posture.valid else None
        if posture is not None and posture.valid and not self._initialized:
            self._initialized = True
            self.load_current()
        tracking_error = self._update_pose_display(posture)
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
            numeric = PostureFlag.NUMERIC_IK in flags
            ik_status_bits = posture.ik_status
            plan_sequence_new = posture.plan_sequence != self._plan_base_sequence
            self._last_plan_sequence = posture.plan_sequence
            plan_pending = (numeric and self._numeric_plan_since_ns is not None
                            and not plan_sequence_new)
            plan_active = numeric and (posture.planning or plan_pending)
            singular = []
            if PostureFlag.SHOULDER_SINGULAR in flags:
                singular.append("肩奇异")
            if PostureFlag.WRIST_SINGULAR in flags:
                singular.append("腕奇异")
            if plan_active:
                status = "数值轨迹计算中，机械臂保持不动"
            elif numeric and ik_status_bits == 1:
                status = "数值规划失败：目标不可达、受限位阻挡或停在局部极小"
            elif numeric and ik_status_bits == 2:
                status = "数值规划失败：参考平面退化"
            elif numeric and not posture.plan_meta_valid:
                status = "数值解：设备未上报轨迹长度/执行进度（旧版报文）"
            else:
                status = ("IK 闭环" if PostureFlag.IK_ACTIVE in flags else "关节目标") + \
                         ("，数值解" if numeric else "") + \
                         ("，自动 ψ" if PostureFlag.AUTO_PSI in flags else "") + \
                         (f"，{'/'.join(singular)}" if singular else "")
            self.ik_status.setText(status)
            if plan_active:
                if not self._target_dirty:
                    self._set_target_dirty(False, "目标已发送，数值轨迹计算中")
                self.plan_label.setText(
                    f"计算中…  已完成 {posture.plan_time_ms:.1f} ms CPU")
                self.plan_label.setStyleSheet(f"color: {theme.WARN};")
            elif numeric and ik_status_bits in (1, 2):
                self._numeric_plan_since_ns = None
                reasons = {
                    1: "数值线性方程/DLS 求解失败",
                    2: "卡在关节限位、奇异或局部极小",
                    3: "超过最大规划迭代次数",
                }
                reason = reasons.get(posture.plan_failure, "目标不可达或规划受阻") \
                    if posture.plan_detail_valid else "目标不可达或规划受阻"
                limited = [f"J{i}" for i in range(7)
                           if posture.plan_detail_valid and posture.plan_limit_mask & (1 << i)]
                if limited:
                    reason += "；受限关节 " + "/".join(limited)
                if not self._target_dirty:
                    self._set_target_dirty(True, "规划失败，机械臂不会运动")
                self.plan_label.setText(f"计算失败：{reason}，未执行")
                self.plan_label.setStyleSheet(f"color: {theme.ERR};")
            elif numeric and not posture.plan_meta_valid:
                self._numeric_plan_since_ns = None
                reached = tracking_error is not None and \
                    tracking_error[0] <= 1.0 and tracking_error[1] <= 0.5
                if not self._target_dirty:
                    self._set_target_dirty(
                        not reached,
                        "设备未上报轨迹进度；实际已到位" if reached
                        else "设备未上报轨迹进度；实际未到位，请重启更新后的 ArmSim")
                self.plan_label.setText(
                    f"计算耗时 {posture.plan_time_ms:.1f} ms；"
                    "旧版32B报文未包含轨迹长度和执行进度"
                )
                self.plan_label.setStyleSheet(f"color: {theme.WARN};")
            elif numeric and plan_sequence_new and not posture.planning:
                self._numeric_plan_since_ns = None
                if posture.plan_steps == 0:
                    already_reached = tracking_error is not None and \
                        tracking_error[0] <= 1.0 and tracking_error[1] <= 0.5
                    if already_reached:
                        if not self._target_dirty:
                            self._set_target_dirty(False, "当前已在目标容差内，无需运动")
                        self.plan_label.setText(
                            f"计算完成：{posture.plan_time_ms:.1f} ms CPU；"
                            "当前位置已在目标容差内，无需运动")
                        self.plan_label.setStyleSheet(f"color: {theme.MUTED};")
                    else:
                        dp = tracking_error[0] if tracking_error is not None else float("nan")
                        dr = tracking_error[1] if tracking_error is not None else float("nan")
                        self._set_target_dirty(
                            True, "计算返回 0 个轨迹点，但实际 TCP 仍有明显误差")
                        self.plan_label.setText(
                            f"规划异常：0 个轨迹点，但实际误差 {dp:.1f} mm / {dr:.3f}°；"
                            "未执行，请重新发送")
                        self.plan_label.setStyleSheet(f"color: {theme.ERR};")
                elif posture.plan_exec_index < posture.plan_steps:
                    if not self._target_dirty:
                        self._set_target_dirty(False, "轨迹执行中")
                    self.plan_label.setText(
                        f"计算完成：{posture.plan_time_ms:.1f} ms CPU；"
                        f"执行轨迹 {posture.plan_exec_index}/{posture.plan_steps}")
                    self.plan_label.setStyleSheet(f"color: {theme.OK};")
                else:
                    reached = tracking_error is not None and \
                        tracking_error[0] <= 1.0 and tracking_error[1] <= 0.5
                    if not self._target_dirty:
                        self._set_target_dirty(
                            not reached,
                            "轨迹执行完成，已到位" if reached
                            else "轨迹执行完成，但实际位姿仍未到位")
                    self.plan_label.setText(
                        f"计算完成：{posture.plan_time_ms:.1f} ms CPU；轨迹已执行完"
                        + ("，实际已到位" if reached else "，实际未到位"))
                    self.plan_label.setStyleSheet(
                        f"color: {theme.OK if reached else theme.ERR};")
            elif self._numeric_plan_since_ns is not None:
                self.plan_label.setText("计算中…  等待规划结果")
                self.plan_label.setStyleSheet(f"color: {theme.WARN};")
            else:
                self.plan_label.setText("—")
                self.plan_label.setStyleSheet("")
        else:
            self.ik_status.setText("—")
            self.plan_label.setText("—")
            self.plan_label.setStyleSheet("")

        if self._feedback and time.monotonic() < self._feedback_until:
            self.state_label.setText(self._feedback)
        elif sample is None:
            self.state_label.setText("已连接，等待 ARM_POSTURE")
        elif state == ArmState.HOST_IK:
            if posture is not None and plan_active:
                self.state_label.setText("HOST_IK：正在预计算数值轨迹，机械臂保持不动")
            elif self._is_numeric():
                if posture is not None and posture.ik_status in (1, 2):
                    self.state_label.setText("HOST_IK：数值规划失败，机械臂保持不动")
                elif posture is not None and not posture.plan_meta_valid:
                    self.state_label.setText(
                        "HOST_IK：旧版设备未上报轨迹执行进度，请重启更新后的 ArmSim")
                elif posture is not None and posture.plan_steps == 0:
                    self.state_label.setText("HOST_IK：当前位置已在目标容差内，无需运动")
                elif posture is not None and posture.plan_exec_index < posture.plan_steps:
                    self.state_label.setText(
                        f"HOST_IK：轨迹执行中 {posture.plan_exec_index}/{posture.plan_steps}")
                else:
                    self.state_label.setText(
                        "HOST_IK：轨迹已执行完；修改目标后点击“发送并计算”")
            else:
                mode = "自动 ψ" if self.auto_psi.isChecked() else "手动 ψ"
                self.state_label.setText(f"HOST_IK：{mode}，修改目标后点击“发送目标”")
        elif state == ArmState.HOST:
            self.state_label.setText("HOST：可载入当前位姿后进入 IK 闭环")
        else:
            name = ArmState(state).name if state in ArmState._value2member_map_ else str(state)
            self.state_label.setText(f"当前 {name}；请先切到 HOST")
        self._update_enabled()

    def _clear_pose_display(self) -> None:
        self.actual_pose_label.setText("—")
        self.track_error_label.setText("—")
        self.track_error_label.setStyleSheet("")

    def _update_pose_display(self, posture) -> tuple[float, float] | None:
        if posture is None or not posture.valid:
            self._clear_pose_display()
            return None
        values = {k: widget.value() for k, widget in self.fields.items()}
        dx = (posture.x - values["x"]) * 1000.0
        dy = (posture.y - values["y"]) * 1000.0
        dz = (posture.z - values["z"]) * 1000.0
        dp = math.sqrt(dx * dx + dy * dy + dz * dz)
        target_rot = transform_from_xyz_rpy(
            values["x"], values["y"], values["z"],
            values["roll"], values["pitch"], values["yaw"])[:3, :3]
        actual_rot = transform_from_xyz_rpy(
            posture.x, posture.y, posture.z,
            posture.roll, posture.pitch, posture.yaw)[:3, :3]
        relative = target_rot.T @ actual_rot
        cosine = max(-1.0, min(1.0, (float(relative.trace()) - 1.0) / 2.0))
        dr = math.degrees(math.acos(cosine))

        self.actual_pose_label.setText(
            f"XYZ  X {posture.x * 1000:+9.3f}  Y {posture.y * 1000:+9.3f}  "
            f"Z {posture.z * 1000:+9.3f} mm\n"
            f"RPY  {posture.roll:+8.3f}°  {posture.pitch:+8.3f}°  {posture.yaw:+8.3f}°"
            f"    ψ {posture.psi_deg:+8.3f}°")
        self.track_error_label.setText(
            f"ΔXYZ  {dx:+8.3f}  {dy:+8.3f}  {dz:+8.3f} mm\n"
            f"|Δp| {dp:8.3f} mm      姿态夹角 {dr:7.3f}°")
        color = theme.OK if dp <= 1.0 and dr <= 0.5 else theme.WARN if dp <= 5.0 and dr <= 2.0 else theme.ERR
        self.track_error_label.setStyleSheet(f"color: {color};")
        return dp, dr

    # ------------------------------------------------------------ ψ 可达域
    def _on_pose_changed(self, _value: float) -> None:
        if not self._updating:
            self._mark_target_dirty()
            self._schedule_range()
            self._update_enabled()

    def _schedule_range(self) -> None:
        if self._supported and not self._range_timer.isActive():
            self._range_timer.start()

    def _range_key(self):
        return (self._branch, tuple(round(widget.value(), 4) for widget in self.fields.values()))

    def _update_psi_range(self) -> None:
        if not self._supported:
            return
        if self._is_numeric():
            self.psi_range_label.setText("数值解：不使用 ψ，冗余度由零空间项拉向进入 IK 时的构型")
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
            if self._is_numeric() and self._numeric_plan_since_ns is not None:
                self._set_target_dirty(False, "设备已接受目标，等待数值轨迹计算")
            else:
                self._set_target_dirty(False, "设备已接受目标")
            if not quiet:
                self._set_feedback(
                    "设备已接收目标，等待数值轨迹规划" if self._is_numeric()
                    else "设备已接受 IK 目标")
        except Exception as exc:
            self._numeric_plan_since_ns = None
            self._set_target_dirty(True, "发送失败，目标仍未发送")
            self._set_feedback(f"IK 目标被拒绝：{exc}", 3.0)
        self._update_enabled()

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
        numeric = self._is_numeric()
        self.apply_btn.setText("发送并计算" if numeric else "发送目标")
        self.auto_psi.setEnabled(self._supported and host and not numeric)
        auto_psi = self.auto_psi.isChecked()
        for key, widget in self.fields.items():
            widget.setEnabled(self._supported and state == ArmState.HOST_IK
                              and not (key == "psi" and (auto_psi or numeric)))
        for spin in self.weight_spins:
            spin.setEnabled(self._supported and host and (auto_psi or numeric))
        self.method_box.setEnabled(self._supported and host)
