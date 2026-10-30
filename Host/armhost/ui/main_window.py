"""监视主窗口：组装工具栏、波形、关节表、状态面板与底部标签页。"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QGuiApplication
from PySide6.QtWidgets import (QComboBox, QDockWidget, QLabel, QMainWindow, QMessageBox, QPushButton,
                               QSplitter, QTabWidget, QToolBar)

from ..comm import FaultConfig, list_serial_ports
from ..controller import MonitorController
from ..model import channel_stats, timing_stats
from ..model.backend import MockBackend, ReplayBackend, SerialBackend
from ..model.sim import SimBackend
from . import theme
from .joint_table import JointTable
from .panels import HexPanel, LogView, RecordPanel, StatsTable
from .pose_panel import PosePanel
from .scope import ScopeWidget
from .status_panel import StatusPanel

log = logging.getLogger("armhost.ui")

SRC_MOCK = "mock"
SRC_MOCK_NOISY = "mock_noisy"
SRC_SIM = "sim"
SRC_SERIAL = "serial"
BAUDS = (115200, 460800, 921600, 1000000, 2000000)


class MonitorWindow(QMainWindow):
    def __init__(self, ctrl: MonitorController | None = None, joint_count: int = 7,
                 config: dict | None = None) -> None:
        super().__init__()
        from ..config import DEFAULTS
        self.cfg = config or DEFAULTS
        self.setWindowTitle("七轴机械臂上位机 — 监视")
        self.resize(1500, 920)
        self.ctrl = ctrl or MonitorController(joint_count=joint_count)
        self._hex_last_ns = 0

        # ---------------------------------------------------------- 工具栏
        tb = QToolBar("连接")
        tb.setMovable(False)
        self.addToolBar(tb)
        tb.addWidget(QLabel(" 数据源 "))
        self.src = QComboBox()
        self.src.addItem("模拟设备", SRC_MOCK)
        self.src.addItem("模拟设备（含噪声/丢字节）", SRC_MOCK_NOISY)
        self.src.addItem("仿真（MuJoCo / 内置模型）", SRC_SIM)
        self.src.addItem("串口", SRC_SERIAL)
        self.src.currentIndexChanged.connect(self._on_src)
        tb.addWidget(self.src)
        tb.addWidget(QLabel(" 端口 "))
        self.port = QComboBox()
        self.port.setMinimumWidth(220)
        tb.addWidget(self.port)
        self.refresh_act = QAction("刷新", self)
        self.refresh_act.setToolTip("重新枚举串口")
        self.refresh_act.triggered.connect(self._refresh_ports)
        tb.addAction(self.refresh_act)
        tb.addWidget(QLabel(" 波特率 "))
        self.baud = QComboBox()
        for b in BAUDS:
            self.baud.addItem(str(b), b)
        b = int(self.cfg["serial"]["baudrate"])
        if b not in BAUDS:
            self.baud.addItem(str(b), b)
        self.baud.setCurrentIndex(self.baud.findData(b))
        tb.addWidget(self.baud)
        tb.addSeparator()
        self.conn_btn = QPushButton("连接")
        self.conn_btn.setCheckable(True)
        self.conn_btn.toggled.connect(self._on_connect)
        tb.addWidget(self.conn_btn)
        tb.addSeparator()
        self.estop_btn = QPushButton("急 停")
        self.estop_btn.setObjectName("estop")
        self.estop_btn.setToolTip("锁存急停：清除并拒绝上位机力矩设定（Ctrl+E）。遥控保护档仍是最高优先级")
        self.estop_btn.setShortcut("Ctrl+E")
        self.estop_btn.clicked.connect(lambda: self.ctrl.estop(True))
        tb.addWidget(self.estop_btn)
        self.release_btn = QPushButton("解除急停")
        self.release_btn.clicked.connect(self._release_estop)
        tb.addWidget(self.release_btn)

        # ---------------------------------------------------------- 中央：波形 / 三维 两个标签页，关节表常驻下方
        self.scope = ScopeWidget(self.ctrl.buffer, joint_count, fps=int(self.cfg["ui"]["scope_fps"]))
        i = self.scope.win_combo.findData(int(self.cfg["ui"]["default_window_s"]))
        if i >= 0:
            self.scope.win_combo.setCurrentIndex(i)
        self.pose = PosePanel(self.ctrl.latest, self._read_device_dh, fps=int(self.cfg["ui"]["view3d_fps"]))
        self.center_tabs = QTabWidget()
        self.center_tabs.addTab(self.scope, "实时波形")
        self.center_tabs.addTab(self.pose, "三维姿态")
        self.table = JointTable(joint_count)
        split = QSplitter(Qt.Vertical)
        split.addWidget(self.center_tabs)
        split.addWidget(self.table)
        split.setStretchFactor(0, 4)
        split.setStretchFactor(1, 1)
        self.setCentralWidget(split)

        # ---------------------------------------------------------- 右侧状态
        self.status_panel = StatusPanel()
        dock = QDockWidget("状态", self)
        dock.setObjectName("status_dock")
        dock.setWidget(self.status_panel)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

        # ---------------------------------------------------------- 底部标签
        self.record = RecordPanel(self.ctrl.recorder)
        self.record.replay_requested.connect(self._start_replay)
        self.record.replay_pause.connect(self._replay_pause)
        self.record.replay_seek.connect(self._replay_seek)
        self.stats = StatsTable()
        self.events = LogView()
        self.hex = HexPanel()
        self.hex.enable.toggled.connect(self._hex_toggle)
        tabs = QTabWidget()
        tabs.addTab(self.record, "记录 / 回放")
        tabs.addTab(self.stats, "统计")
        tabs.addTab(self.events, "事件")
        tabs.addTab(self.hex, "原始数据")
        self.tabs = tabs
        bdock = QDockWidget("工具", self)
        bdock.setObjectName("tool_dock")
        bdock.setWidget(tabs)
        bdock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, bdock)
        self.resizeDocks([bdock], [200], Qt.Vertical)
        self.resizeDocks([dock], [330], Qt.Horizontal)

        # ---------------------------------------------------------- 状态栏
        self.sb_conn = QLabel()
        self.sb_rate = QLabel()
        self.sb_time = QLabel()
        self.sb_err = QLabel()
        for w in (self.sb_conn, self.sb_rate, self.sb_time, self.sb_err):
            w.setObjectName("value")
            self.statusBar().addPermanentWidget(w)

        # ---------------------------------------------------------- 定时器（批量刷新，与数据到达解耦）
        self._t_fast = QTimer(self, interval=100, timeout=self._tick_fast)      # 10 Hz
        self._t_slow = QTimer(self, interval=500, timeout=self._tick_slow)      # 2 Hz
        self._t_fast.start()
        self._t_slow.start()

        self._refresh_ports()
        self._on_src()
        self.estop_btn.setEnabled(False)      # 连上支持急停的数据源后才可用
        self.release_btn.setEnabled(False)

    # ================================================================ 数据源
    def _on_src(self) -> None:
        """未连接时：只有选了"串口"，端口 / 波特率才可选。"""
        serial = self.src.currentData() == SRC_SERIAL
        for w in (self.port, self.baud):
            w.setEnabled(serial)
            w.setToolTip("" if serial else "数据源为模拟设备时不需要串口参数")
        self.refresh_act.setEnabled(serial)
        self.src.setToolTip("")

    def _refresh_ports(self) -> None:
        cur = self.port.currentData()
        self.port.clear()
        for p in list_serial_ports():
            self.port.addItem(f"{p.device}  {p.description}", p.device)
        if self.port.count() == 0:
            self.port.addItem("（没有串口）", None)
        i = self.port.findData(cur)
        if i >= 0:
            self.port.setCurrentIndex(i)

    def _on_connect(self, on: bool) -> None:
        if not on:
            self._switch(None)
            return
        src = self.src.currentData()
        try:
            if src == SRC_SERIAL:
                port = self.port.currentData()
                if not port:
                    raise ValueError("没有可用串口")
                backend = SerialBackend.open_port(port, int(self.baud.currentData()))
            elif src == SRC_MOCK_NOISY:
                backend = MockBackend(faults=FaultConfig(noise_prob=0.03, corrupt_prob=0.02, drop_prob=0.02,
                                                         chunk_max=31))
            elif src == SRC_SIM:
                s = self.cfg["sim"]
                backend = SimBackend(s["rate_hz"], s["realtime"], s["prefer_mujoco"])
            else:
                backend = MockBackend(rate_hz=int(self.cfg["mock"]["rate_hz"]))
        except Exception as e:
            QMessageBox.warning(self, "连接失败", str(e))
            self.conn_btn.setChecked(False)
            return
        self._switch(backend)

    def _switch(self, backend) -> None:
        QGuiApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.ctrl.set_backend(backend)
        finally:
            QGuiApplication.restoreOverrideCursor()
        replay = isinstance(backend, ReplayBackend)
        # 下拉框跟随实际数据源（命令行 --sim / --mock 直接连接时也一致）
        src = {"sim": SRC_SIM, "mock": SRC_MOCK, "serial": SRC_SERIAL}.get(getattr(backend, "name", ""))
        if src is not None and isinstance(backend, MockBackend) and backend.device.faults.noise_prob:
            src = SRC_MOCK_NOISY
        if src is not None:
            self.src.blockSignals(True)
            self.src.setCurrentIndex(self.src.findData(src))
            self.src.blockSignals(False)
        self.record.set_replay_active(replay)
        if backend is None:
            self.src.setEnabled(True)
            self._on_src()
        else:
            # 连接期间锁定，避免在活动连接下改端口；断开后恢复
            for w in (self.src, self.port, self.baud):
                w.setEnabled(False)
                w.setToolTip("连接中不能修改，先点\"断开\"")
            self.refresh_act.setEnabled(False)
        self.conn_btn.blockSignals(True)
        self.conn_btn.setChecked(backend is not None)
        self.conn_btn.setText("断开" if backend is not None else "连接")
        self.conn_btn.blockSignals(False)
        can = backend is not None and backend.can_estop
        self.estop_btn.setEnabled(can)
        self.release_btn.setEnabled(can)
        self.scope.set_paused(False)
        self._hex_toggle(self.hex.enable.isChecked())

    def _start_replay(self, path: str, speed: float, loop: bool) -> None:
        try:
            backend = ReplayBackend(path, speed, loop)
        except Exception as e:
            QMessageBox.warning(self, "回放失败", f"{path}\n{e}")
            return
        self._switch(backend)

    def _replay_pause(self, p: bool) -> None:
        if isinstance(self.ctrl.backend, ReplayBackend):
            self.ctrl.backend.set_paused(p)

    def _replay_seek(self, f: float) -> None:
        if isinstance(self.ctrl.backend, ReplayBackend):
            self.ctrl.backend.seek(f)

    def _release_estop(self) -> None:
        if QMessageBox.question(self, "解除急停", "确认解除急停？解除后上位机力矩设定重新被接受。") \
                == QMessageBox.Yes:
            self.ctrl.estop(False)

    def _hex_toggle(self, on: bool) -> None:
        link = getattr(self.ctrl.backend, "link", None)
        if link is not None:
            link.hexlog.enabled = on

    def _read_device_dh(self):
        b = self.ctrl.backend
        return b.read_dh() if b is not None and hasattr(b, "read_dh") else None

    # ================================================================ 定时刷新
    def _tick_fast(self) -> None:
        s = self.ctrl.latest()
        self.table.update_sample(s)
        st = self.ctrl.status()
        view = self.ctrl.buffer.view(3.0)
        timing = timing_stats(view)
        self.status_panel.update_all(st, s, timing, self.ctrl.buffer.size, self.ctrl.buffer.capacity)
        state = st.get("state", "—")
        alive = st.get("link_alive", False)
        color = theme.OK if alive else theme.ERR if state != "closed" else theme.MUTED
        self.sb_conn.setText(f"<span style='color:{color}'>●</span> {st.get('backend', '—')} {st.get('port', '')} {state}")
        self.sb_rate.setText(f"  MCU {timing.mcu_rate_hz:5.0f} Hz | 接收 {timing.host_rate_hz:5.0f} Hz  ")
        if s is not None:
            self.sb_time.setText(f"  MCU t={s.mcu_time_us / 1e6:10.3f}s  上位机 t={s.host_time_ns / 1e9:10.3f}s  ")
        p = st.get("parser", {})
        self.sb_err.setText(f"  丢帧 {st.get('lost_frames', 0)}  校验错 {p.get('crc_errors', 0) + p.get('header_errors', 0)}  "
                            if p else "")

        evs = self.ctrl.drain_events()
        if evs:
            self.events.append_lines([t for _, t, _ in evs])
            for kind, text, payload in evs:
                if kind == "replay_finished":
                    self.record.set_replay_active(False)
                if kind == "sim_error":
                    QMessageBox.warning(self, "仿真停止", f"仿真线程异常退出：\n{payload}\n\n可以重新连接再试。")
                if kind == "device_info" and payload.joint_count != self.table.rowCount():
                    self.events.append_lines([f"⚠ 设备关节数 {payload.joint_count} 与界面 {self.table.rowCount()} 不一致"])
            self.statusBar().showMessage(evs[-1][1], 5000)

        if self.hex.enable.isChecked():
            link = getattr(self.ctrl.backend, "link", None)
            if link is not None:
                recs = [r for r in link.hexlog.snapshot(400) if r.host_time_ns > self._hex_last_ns]
                if recs:
                    self._hex_last_ns = recs[-1].host_time_ns
                    self.hex.view.append_lines([r.format() for r in recs])
        self.record.tick(st)

    def _tick_slow(self) -> None:
        if self.tabs.currentWidget() is self.stats:
            self.stats.update_stats(channel_stats(self.scope.current_view(), self.scope.visible_channels()))

    # ================================================================ 关闭
    def closeEvent(self, ev) -> None:
        if self.ctrl.recorder.active and QMessageBox.question(
                self, "退出", "正在记录，确认停止记录并退出？") != QMessageBox.Yes:
            ev.ignore()
            return
        for t in (self._t_fast, self._t_slow, self.scope.timer, self.pose.timer):
            t.stop()
        self.record.stop_recording()
        self.ctrl.shutdown()              # 串口后端：发急停 + 清零，再关线程
        super().closeEvent(ev)
