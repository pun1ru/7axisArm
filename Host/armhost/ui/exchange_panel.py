"""兑换站（科技核心）控制页。

布局自上而下：状态与步骤进度、难度与随机位姿、装配步骤、实时读数，手动位姿折叠在“高级”里。
装配步骤按规则 5.4.3 随难度显示：一级插入后确认，二级加上移，三/四级再加 P 轴与 Q 轴。
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QPushButton, QVBoxLayout, QWidget)

from ..protocol.enums import ExchangeAction, ExchangePhase, ExchangeStateFlag


PHASE_TEXT = {
    ExchangePhase.PARKED: "停放位，选择难度后开始",
    ExchangePhase.MOVING_POSE: "科技核心运动到位中",
    ExchangePhase.READY: "已到位，可插入能量单元",
    ExchangePhase.INSERTING: "插入中",
    ExchangePhase.INSERTED: "已插入 100 mm",
    ExchangePhase.LIFTING: "上移中",
    ExchangePhase.LIFTED: "已上移 100 mm",
    ExchangePhase.ROTATING_P: "P 轴旋转中",
    ExchangePhase.P_ROTATED: "P 轴 90° 完成",
    ExchangePhase.ROTATING_Q: "Q 轴旋转中",
    ExchangePhase.Q_READY: "Q 轴到位",
    ExchangePhase.COMPLETE: "装配成功，能量单元回收中",
    ExchangePhase.RESETTING: "回到停放位中",
}

DIFFICULTY_TEXT = (
    (1, "一级  固定朝向，插入"),
    (2, "二级  随机朝向，插入 + 上移"),
    (3, "三级  随机朝向，插入 + 上移 + P/Q 旋转"),
    (4, "四级  同三级（双核心协同仅在实机）"),
)

# 完整流程的 6 步；每个难度只需要其中一部分。
STEPS = (("pose", "到位"), ("insert", "插入"), ("lift", "上移"),
         ("p", "P 轴"), ("q", "Q 轴"), ("confirm", "确认"))
REQUIRED = {1: ("pose", "insert", "confirm"),
            2: ("pose", "insert", "lift", "confirm"),
            3: tuple(k for k, _ in STEPS),
            4: tuple(k for k, _ in STEPS)}
# 各阶段已完成的步骤数（按 STEPS 顺序）。
RANK = {ExchangePhase.PARKED: 0, ExchangePhase.MOVING_POSE: 0, ExchangePhase.READY: 1,
        ExchangePhase.INSERTING: 1, ExchangePhase.INSERTED: 2, ExchangePhase.LIFTING: 2,
        ExchangePhase.LIFTED: 3, ExchangePhase.ROTATING_P: 3, ExchangePhase.P_ROTATED: 4,
        ExchangePhase.ROTATING_Q: 4, ExchangePhase.Q_READY: 5, ExchangePhase.COMPLETE: 6,
        ExchangePhase.RESETTING: 0}
# 可以开始/修改位姿的阶段。
IDLE = (None, ExchangePhase.PARKED, ExchangePhase.READY)
# 各难度确认时所处的阶段。
CONFIRM_AT = {1: ExchangePhase.INSERTED, 2: ExchangePhase.LIFTED,
              3: ExchangePhase.Q_READY, 4: ExchangePhase.Q_READY}


def _spin(lo: float, hi: float, suffix: str, value: float) -> QDoubleSpinBox:
    w = QDoubleSpinBox()
    w.setRange(lo, hi)
    w.setDecimals(1)
    w.setSingleStep(5.0)
    w.setSuffix(suffix)
    w.setValue(value)
    return w


def steps_html(difficulty: int, phase) -> str:
    """步骤进度：已完成 ✓，当前步加粗，其余变灰。只列出该难度需要的步骤。"""
    rank = RANK.get(phase, 0)
    need = REQUIRED.get(difficulty, REQUIRED[1])
    parts, current_marked = [], False
    for idx, (key, name) in enumerate(STEPS):
        if key not in need:
            continue
        if idx < rank:
            parts.append(f"<span style='color:#3fb950'>✓ {name}</span>")
        elif not current_marked and phase is not None:
            parts.append(f"<b>▶ {name}</b>")
            current_marked = True
        else:
            parts.append(f"<span style='color:#8b949e'>{name}</span>")
    return "　".join(parts)


class ExchangePanel(QWidget):
    """右侧停靠区内的一个页签；只通过 ctrl.exchange_command 发命令，不碰串口或 MuJoCo。"""

    def __init__(self, ctrl, parent=None) -> None:
        super().__init__(parent)
        self.ctrl = ctrl
        self._connected = False
        self._pending = None
        self._pending_action = None
        self._auto_requested = False
        self._auto_state_grace_until = 0.0
        self._feedback = ""
        self._feedback_until = 0.0
        self._state = None

        # ---------------------------------------------------------- 状态与步骤进度
        self.state_label = QLabel("未连接")
        self.state_label.setObjectName("value")
        self.state_label.setWordWrap(True)
        self.steps_label = QLabel()
        self.steps_label.setTextFormat(Qt.RichText)
        self.steps_label.setWordWrap(True)

        # ---------------------------------------------------------- 难度与开始
        self.difficulty = QComboBox()
        for level, text in DIFFICULTY_TEXT:
            self.difficulty.addItem(text, level)
        self.random_btn = QPushButton("随机位姿并开始")
        self.random_btn.setToolTip("按所选难度在规则表 5-10 范围内随机生成位姿，科技核心自动运动到位")
        self.auto_btn = QPushButton("自动演示")
        self.auto_btn.setCheckable(True)
        self.auto_btn.setToolTip("自动完成当前难度的全部步骤")
        self.reset_btn = QPushButton("复位")
        self.free_btn = QPushButton("释放关节（可在窗口里拖动）")
        self.free_btn.setCheckable(True)
        self.free_btn.setToolTip("撤掉兑换站所有驱动，可在 MuJoCo 窗口里 Ctrl+右键拖动各关节。\n"
                                 "再次点击或点“复位”重新接管并回到停放位。")

        start = QGroupBox("难度")
        sg = QGridLayout(start)
        sg.addWidget(self.difficulty, 0, 0, 1, 3)
        sg.addWidget(self.random_btn, 1, 0)
        sg.addWidget(self.auto_btn, 1, 1)
        sg.addWidget(self.reset_btn, 1, 2)
        sg.addWidget(self.free_btn, 2, 0, 1, 3)

        # ---------------------------------------------------------- 装配步骤
        self.q_target = _spin(-90.0, 90.0, "°", 0.0)
        self.q_target.setSingleStep(5.0)
        self.insert_btn = QPushButton("插入 100 mm")
        self.lift_btn = QPushButton("上移 100 mm")
        self.p_btn = QPushButton("P 轴 +90°")
        self.q_btn = QPushButton("Q 轴转到目标")
        self.confirm_btn = QPushButton("确认装配")
        self.confirm_btn.setObjectName("exchangeConfirm")
        self.q_label = QLabel("Q 目标")

        self.assembly = QGroupBox("装配步骤")
        ag = QGridLayout(self.assembly)
        ag.addWidget(self.insert_btn, 0, 0)
        ag.addWidget(self.lift_btn, 0, 1)
        ag.addWidget(self.p_btn, 1, 0)
        ag.addWidget(self.q_btn, 1, 1)
        qrow = QHBoxLayout()
        qrow.addWidget(self.q_label)
        qrow.addWidget(self.q_target, 1)
        ag.addLayout(qrow, 2, 0, 1, 2)
        ag.addWidget(self.confirm_btn, 3, 0, 1, 2)

        # ---------------------------------------------------------- 读数
        self.motion_label = QLabel("—")
        self.motion_label.setObjectName("value")
        self.motion_label.setWordWrap(True)
        self.seed_label = QLabel("")
        self.seed_label.setObjectName("title")
        self.seed_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        # ---------------------------------------------------------- 高级：手动位姿
        self.x = _spin(-100.0, 0.0, " mm", -50.0)
        self.y = _spin(100.0, 300.0, " mm", 200.0)
        self.z = _spin(500.0, 700.0, " mm", 600.0)
        self.theta = _spin(-90.0, 90.0, "°", 0.0)
        self.phi = _spin(0.0, 90.0, "°", 90.0)
        self.alpha = _spin(-45.0, 45.0, "°", 0.0)
        self.apply_btn = QPushButton("应用位姿")
        self.apply_btn.setToolTip("一级必须保持 θ=0°、φ=90°、α=0°")
        self.current_btn = QPushButton("载入当前")
        self.advanced = QGroupBox("手动位姿（高级）")
        self.advanced.setCheckable(True)
        self.advanced.setChecked(False)
        adv = QGridLayout(self.advanced)
        for i, (name, widget) in enumerate((("X", self.x), ("Y", self.y), ("Z", self.z),
                                            ("θ", self.theta), ("φ", self.phi), ("α", self.alpha))):
            row, col = divmod(i, 3)
            adv.addWidget(QLabel(name), row * 2, col)
            adv.addWidget(widget, row * 2 + 1, col)
        adv.addWidget(self.apply_btn, 4, 0, 1, 2)
        adv.addWidget(self.current_btn, 4, 2)
        self._advanced_widgets = (self.x, self.y, self.z, self.theta, self.phi, self.alpha,
                                  self.apply_btn, self.current_btn)
        self.advanced.toggled.connect(self._on_advanced_toggled)
        self._on_advanced_toggled(False)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.addWidget(self.state_label)
        lay.addWidget(self.steps_label)
        lay.addWidget(start)
        lay.addWidget(self.assembly)
        lay.addWidget(self.motion_label)
        lay.addWidget(self.seed_label)
        lay.addWidget(self.advanced)
        lay.addStretch(1)

        self.random_btn.clicked.connect(lambda: self._send(ExchangeAction.RANDOMIZE))
        self.apply_btn.clicked.connect(lambda: self._send(ExchangeAction.MOVE_TO_POSE, use_pose=True))
        self.current_btn.clicked.connect(self._load_state_pose)
        self.insert_btn.clicked.connect(lambda: self._send(ExchangeAction.INSERT))
        self.lift_btn.clicked.connect(lambda: self._send(ExchangeAction.LIFT))
        self.p_btn.clicked.connect(lambda: self._send(ExchangeAction.ROTATE_P))
        self.q_btn.clicked.connect(lambda: self._send(ExchangeAction.ROTATE_Q))
        self.confirm_btn.clicked.connect(lambda: self._send(ExchangeAction.CONFIRM))
        self.reset_btn.clicked.connect(lambda: self._send(ExchangeAction.RESET))
        self.auto_btn.toggled.connect(self._on_auto)
        self.free_btn.toggled.connect(self._on_free)
        self.difficulty.currentIndexChanged.connect(self._update_buttons)
        self.set_connected(False)

    def _on_advanced_toggled(self, on: bool) -> None:
        """折叠时隐藏内容，只留标题行。"""
        for w in self._advanced_widgets:
            w.setVisible(on)
        for lbl in self.advanced.findChildren(QLabel):
            lbl.setVisible(on)

    def _level(self) -> int:
        s = self._state
        if s is not None and s.phase not in IDLE:
            return int(s.difficulty)
        return int(self.difficulty.currentData() or 1)

    # ---------------------------------------------------------------- 连接 / 命令
    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        self._pending = None
        self._pending_action = None
        self._auto_requested = False
        self._auto_state_grace_until = 0.0
        self._state = None
        self._feedback = ""
        self._feedback_until = 0.0
        self.auto_btn.blockSignals(True)
        self.auto_btn.setChecked(False)
        self.auto_btn.blockSignals(False)
        self.free_btn.blockSignals(True)
        self.free_btn.setChecked(False)
        self.free_btn.blockSignals(False)
        self.state_label.setText("未连接：请选择 ArmSim 串口数据源" if not connected
                                 else "已连接，等待兑换站状态…")
        self._update_buttons()

    def _send(self, action: ExchangeAction, use_pose: bool = False) -> bool:
        if not self._connected or self._pending is not None:
            return False
        difficulty = int(self.difficulty.currentData() or 1)
        try:
            future = self.ctrl.exchange_command(
                action, difficulty, self.q_target.value(),
                self.x.value(), self.y.value(), self.z.value(),
                self.theta.value(), self.phi.value(), self.alpha.value(), 0, use_pose)
        except Exception as exc:
            self._set_feedback(f"发送失败：{exc}", 3.0)
            return False
        if future is None:
            self._set_feedback("当前数据源不支持兑换站", 3.0)
            return False
        self._pending = future
        self._pending_action = action
        self._set_feedback(f"已发送：{action.name}", 1.5)
        self._update_buttons()
        return True

    def _on_auto(self, checked: bool) -> None:
        self._auto_requested = checked
        action = ExchangeAction.AUTO_DEMO if checked else ExchangeAction.STOP_AUTO
        if not self._send(action):
            self._auto_requested = not checked
            self.auto_btn.blockSignals(True)
            self.auto_btn.setChecked(self._auto_requested)
            self.auto_btn.blockSignals(False)

    def _on_free(self, checked: bool) -> None:
        # 开：撤驱动；关：RESET 重新接管并平滑回到停放位
        if not self._send(ExchangeAction.FREE if checked else ExchangeAction.RESET):
            self.free_btn.blockSignals(True)
            self.free_btn.setChecked(not checked)
            self.free_btn.blockSignals(False)

    def _load_state_pose(self) -> None:
        s = self._state
        if s is None:
            return
        for spin, value in ((self.x, s.x_mm), (self.y, s.y_mm), (self.z, s.z_mm),
                            (self.theta, s.theta_deg), (self.phi, s.phi_deg),
                            (self.alpha, s.alpha_deg)):
            spin.setValue(float(value))

    # ---------------------------------------------------------------- 周期刷新
    def tick(self, sample) -> None:
        if self._pending is not None and self._pending.done():
            future, self._pending = self._pending, None
            action, self._pending_action = self._pending_action, None
            try:
                future.result()
            except Exception as exc:
                self._set_feedback(f"命令被拒绝：{exc}", 3.0)
                if action in (ExchangeAction.AUTO_DEMO, ExchangeAction.STOP_AUTO):
                    self._auto_requested = not self._auto_requested
                    self.auto_btn.blockSignals(True)
                    self.auto_btn.setChecked(self._auto_requested)
                    self.auto_btn.blockSignals(False)
            else:
                if action in (ExchangeAction.AUTO_DEMO, ExchangeAction.STOP_AUTO):
                    # ACK 先于下一帧 EXCHANGE_STATE 到达；短暂保持按钮状态，避免旧状态把它弹回
                    self._auto_state_grace_until = time.monotonic() + 0.5

        state = getattr(sample, "exchange", None) if sample is not None else None
        if state is not None:
            self._state = state

        if self._feedback and time.monotonic() < self._feedback_until:
            self.state_label.setText(self._feedback)
        elif not self._connected:
            self.state_label.setText("未连接：请选择 ArmSim 串口数据源")
        elif state is None:
            self.state_label.setText("已连接，但尚未收到兑换站状态（请确认已加载兑换站）")
        else:
            try:
                text = PHASE_TEXT.get(ExchangePhase(state.phase), str(state.phase))
            except ValueError:
                text = str(state.phase)
            flags = ExchangeStateFlag(state.flags)
            free = ExchangeStateFlag.FREE in flags
            self.state_label.setText("关节已释放：可在窗口里拖动" if free else text)
            if self._pending is None and self.free_btn.isChecked() != free:
                self.free_btn.blockSignals(True)
                self.free_btn.setChecked(free)
                self.free_btn.blockSignals(False)
            self.motion_label.setText(
                f"位置 [{state.x_mm:+.0f}, {state.y_mm:+.0f}, {state.z_mm:+.0f}] mm　"
                f"θ/φ/α [{state.theta_deg:+.0f}, {state.phi_deg:.0f}, {state.alpha_deg:+.0f}]°\n"
                f"插入 {state.insert_mm:.0f} mm　上移 {state.lift_mm:.0f} mm　"
                f"P {state.p_deg:.0f}°　Q {state.q_deg:+.0f}° / 目标 {state.q_target_deg:+.0f}°")
            self.seed_label.setText(f"随机种子 {state.seed}　t={state.sim_time_s:.1f} s")
            auto = ExchangeStateFlag.AUTO_DEMO in ExchangeStateFlag(state.flags)
            if self._pending is None and time.monotonic() >= self._auto_state_grace_until:
                self._auto_requested = auto
                self.auto_btn.blockSignals(True)
                self.auto_btn.setChecked(auto)
                self.auto_btn.blockSignals(False)
            # 流程进行中，下拉框跟随仿真侧的难度；空闲时以用户的选择为准
            if state.phase not in IDLE:
                idx = self.difficulty.findData(int(state.difficulty))
                if idx >= 0 and idx != self.difficulty.currentIndex():
                    self.difficulty.blockSignals(True)
                    self.difficulty.setCurrentIndex(idx)
                    self.difficulty.blockSignals(False)
                self.q_target.blockSignals(True)
                self.q_target.setValue(float(state.q_target_deg))
                self.q_target.blockSignals(False)

        self._update_buttons()

    def _set_feedback(self, text: str, seconds: float = 2.0) -> None:
        self._feedback = text
        self._feedback_until = time.monotonic() + seconds
        self.state_label.setText(text)

    def _update_buttons(self) -> None:
        s = self._state
        phase = ExchangePhase(s.phase) if s is not None and s.phase in set(ExchangePhase) else None
        level = self._level()
        ready = self._connected and self._pending is None
        busy = bool(s and ExchangeStateFlag.MOVING in ExchangeStateFlag(s.flags))
        free = bool(s and ExchangeStateFlag.FREE in ExchangeStateFlag(s.flags))
        idle = ready and not busy and phase in IDLE and not free

        self.steps_label.setText(steps_html(level, phase) if s is not None else "")
        # 该难度不需要的步骤直接隐藏，避免一堆灰按钮
        self.lift_btn.setVisible(level >= 2)
        for w in (self.p_btn, self.q_btn, self.q_target, self.q_label):
            w.setVisible(level >= 3)

        self.difficulty.setEnabled(idle)
        self.random_btn.setEnabled(idle)
        for w in self._advanced_widgets:
            if w is self.current_btn:
                w.setEnabled(self._connected and s is not None)
            else:
                w.setEnabled(idle)
        step_ok = ready and not busy and not free
        self.insert_btn.setEnabled(step_ok and phase == ExchangePhase.READY)
        self.lift_btn.setEnabled(step_ok and phase == ExchangePhase.INSERTED)
        self.p_btn.setEnabled(step_ok and phase == ExchangePhase.LIFTED)
        self.q_btn.setEnabled(step_ok and phase == ExchangePhase.P_ROTATED)
        self.q_target.setEnabled(step_ok and phase in (ExchangePhase.P_ROTATED,
                                                       ExchangePhase.Q_READY))
        self.confirm_btn.setEnabled(step_ok and phase == CONFIRM_AT[level])
        self.reset_btn.setEnabled(ready)
        self.auto_btn.setEnabled(ready and not free)
        self.free_btn.setEnabled(ready)
