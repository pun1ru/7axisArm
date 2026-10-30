"""底部标签页：记录与回放、基本统计、事件日志、原始十六进制。"""

from __future__ import annotations

import math
import time
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QFileDialog, QHBoxLayout,
                               QHeaderView, QLabel, QPlainTextEdit, QPushButton, QSlider,
                               QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from ..model import ChannelStats
from ..model.recorder import CsvRecorder

LOG_DIR = Path(__file__).resolve().parents[2] / "logs"


class RecordPanel(QWidget):
    """CSV 记录 + 回放控制。回放通过信号交给主窗口切换后端。"""
    replay_requested = Signal(str, float, bool)    # path, speed, loop
    replay_pause = Signal(bool)
    replay_seek = Signal(float)

    def __init__(self, recorder: CsvRecorder, parent=None) -> None:
        super().__init__(parent)
        self.recorder = recorder
        # ---- 记录
        self.rec_btn = QPushButton("● 开始记录")
        self.rec_btn.setCheckable(True)
        self.rec_btn.toggled.connect(self._toggle_record)
        self.rec_info = QLabel("未记录")
        self.rec_info.setObjectName("value")
        self.open_dir_btn = QPushButton("另存为…")
        self.open_dir_btn.setToolTip("选择记录文件位置；不选则自动存到 Host/logs/")
        self.open_dir_btn.clicked.connect(self._choose_path)
        self._next_path: Path | None = None
        # ---- 回放
        self.load_btn = QPushButton("打开 CSV 回放…")
        self.load_btn.clicked.connect(self._choose_replay)
        self.speed = QComboBox()
        for s in (0.25, 0.5, 1.0, 2.0, 5.0):
            self.speed.addItem(f"{s:g}×", s)
        self.speed.setCurrentIndex(2)
        self.loop = QCheckBox("循环")
        self.play_btn = QPushButton("暂停回放")
        self.play_btn.setCheckable(True)
        self.play_btn.setEnabled(False)
        self.play_btn.toggled.connect(lambda p: (self.play_btn.setText("继续回放" if p else "暂停回放"),
                                                 self.replay_pause.emit(p)))
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setEnabled(False)
        self.slider.sliderReleased.connect(lambda: self.replay_seek.emit(self.slider.value() / 1000))
        self.replay_info = QLabel("")
        self.replay_info.setObjectName("value")

        r1 = QHBoxLayout()
        r1.addWidget(QLabel("记录"))
        r1.addWidget(self.rec_btn)
        r1.addWidget(self.open_dir_btn)
        r1.addWidget(self.rec_info, 1)
        r2 = QHBoxLayout()
        r2.addWidget(QLabel("回放"))
        r2.addWidget(self.load_btn)
        r2.addWidget(QLabel("速度"))
        r2.addWidget(self.speed)
        r2.addWidget(self.loop)
        r2.addWidget(self.play_btn)
        r2.addWidget(self.slider, 1)
        r2.addWidget(self.replay_info)
        lay = QVBoxLayout(self)
        lay.addLayout(r1)
        lay.addLayout(r2)
        note = QLabel("CSV 同时保存 MCU 采样时刻 mcu_time_us 与上位机接收时刻 host_time_ns；"
                      "回放按 MCU 时间节奏重放。")
        note.setObjectName("title")
        note.setWordWrap(True)
        lay.addWidget(note)
        lay.addStretch(1)

    def _choose_path(self) -> None:
        LOG_DIR.mkdir(exist_ok=True)
        p, _ = QFileDialog.getSaveFileName(self, "记录到", str(LOG_DIR / self._default_name()), "CSV (*.csv)")
        if p:
            self._next_path = Path(p)
            self.rec_info.setText(f"下次记录到 {self._next_path.name}")

    @staticmethod
    def _default_name() -> str:
        return datetime.now().strftime("arm_%Y%m%d_%H%M%S.csv")

    def _toggle_record(self, on: bool) -> None:
        if on:
            path = self._next_path or (LOG_DIR / self._default_name())
            self._next_path = None
            self.recorder.start(path)
            self.rec_btn.setText("■ 停止记录")
        else:
            self.recorder.stop()
            self.rec_btn.setText("● 开始记录")
            if self.recorder.path:
                self.rec_info.setText(f"已保存 {self.recorder.path}  {self.recorder.rows} 行"
                                      + (f"，丢弃 {self.recorder.dropped}" if self.recorder.dropped else ""))

    def stop_recording(self) -> None:
        if self.rec_btn.isChecked():
            self.rec_btn.setChecked(False)

    def _choose_replay(self) -> None:
        p, _ = QFileDialog.getOpenFileName(self, "选择记录文件", str(LOG_DIR), "CSV (*.csv)")
        if p:
            self.replay_requested.emit(p, float(self.speed.currentData()), self.loop.isChecked())

    def set_replay_active(self, active: bool) -> None:
        self.play_btn.setEnabled(active)
        self.slider.setEnabled(active)
        if not active:
            self.play_btn.setChecked(False)
            self.replay_info.setText("")

    def tick(self, status: dict) -> None:
        r = self.recorder
        if r.active:
            dur = time.monotonic() - r.started_at
            self.rec_info.setText(f"记录中 {r.path.name}  {r.rows} 行  {dur:.0f} s"
                                  + (f"  丢弃 {r.dropped}" if r.dropped else "")
                                  + (f"  错误: {r.error}" if r.error else ""))
        if status.get("backend") == "replay":
            prog = status.get("progress", 0.0)
            if not self.slider.isSliderDown():
                self.slider.setValue(int(prog * 1000))
            self.replay_info.setText(f"{prog * status.get('duration_s', 0):6.1f} / {status.get('duration_s', 0):.1f} s")


class StatsTable(QTableWidget):
    COLS = ("通道", "单位", "点数", "最小", "最大", "平均", "标准差", "RMS")

    def __init__(self, parent=None) -> None:
        super().__init__(0, len(self.COLS), parent)
        self.setHorizontalHeaderLabels(self.COLS)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setAlternatingRowColors(True)
        self.verticalHeader().setVisible(False)
        self.verticalHeader().setDefaultSectionSize(20)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.setToolTip("统计范围 = 波形当前显示的数据（暂停时为冻结的整段缓存）")

    def update_stats(self, stats: list[ChannelStats]) -> None:
        if self.rowCount() != len(stats):
            self.setRowCount(len(stats))
            for r in range(len(stats)):
                for c in range(len(self.COLS)):
                    if self.item(r, c) is None:
                        it = QTableWidgetItem()
                        it.setTextAlignment((Qt.AlignLeft if c < 2 else Qt.AlignRight) | Qt.AlignVCenter)
                        self.setItem(r, c, it)
        for r, s in enumerate(stats):
            vals = (s.name, s.unit, str(s.n)) + tuple("—" if math.isnan(x) else f"{x:.4g}"
                                                      for x in (s.min, s.max, s.mean, s.std, s.rms))
            for c, v in enumerate(vals):
                it = self.item(r, c)
                if it.text() != v:
                    it.setText(v)


class LogView(QPlainTextEdit):
    """事件 / 原始数据：只追加文本，行数有上限。"""

    def __init__(self, max_lines: int = 2000, parent=None) -> None:
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(max_lines)
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        f = QFont("Consolas")
        f.setStyleHint(QFont.Monospace)
        self.setFont(f)

    def append_lines(self, lines: list[str]) -> None:
        if not lines:
            return
        bar = self.verticalScrollBar()
        at_end = bar.value() >= bar.maximum() - 4
        self.appendPlainText("\n".join(lines))
        if at_end:
            bar.setValue(bar.maximum())


class HexPanel(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.enable = QCheckBox("抓取原始数据")
        self.enable.setToolTip("开启后记录最近 2000 块收发字节；会略增加 CPU，调试完关掉")
        self.clear_btn = QPushButton("清空")
        self.view = LogView(4000)
        top = QHBoxLayout()
        top.addWidget(self.enable)
        top.addWidget(self.clear_btn)
        top.addStretch(1)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.addLayout(top)
        lay.addWidget(self.view, 1)
        self.clear_btn.clicked.connect(self.view.clear)
