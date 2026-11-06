"""工作空间控制页：限位表 + 采样参数 + 计算按钮。

只负责生成 WorkspaceCloud 并发出信号；实际点云渲染在 View3D 中完成。
"""

from __future__ import annotations

import time
from concurrent.futures import Future
from typing import Callable

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout,
                               QGroupBox, QHBoxLayout, QHeaderView, QLabel, QPushButton, QSpinBox,
                               QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from ..kinematics import DHTable
from ..kinematics.workspace import sample_workspace
from ..model import branch_text


class WorkspacePanel(QWidget):
    cloud_ready = Signal(object)
    cleared = Signal()
    visibility_changed = Signal(bool)
    fit_requested = Signal()

    def __init__(self, table: DHTable,
                 read_device_limits: Callable[[], Future | None] | None = None, parent=None) -> None:
        super().__init__(parent)
        self._table = table
        self._read_device_limits = read_device_limits
        self._pending: Future | None = None
        self._pending_started = 0.0
        self._loading = False
        self._has_result = False
        self._source = "默认 ±180°"

        self.limit_tbl = QTableWidget(0, 2)
        self.limit_tbl.setHorizontalHeaderLabels(["最小 (°)", "最大 (°)"])
        self.limit_tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.limit_tbl.verticalHeader().setDefaultSectionSize(22)
        self.limit_tbl.setSelectionMode(QAbstractItemView.NoSelection)
        self.limit_tbl.setToolTip("单圈绝对角限位；手动修改后会清除旧点云，需要重新计算")
        self.limit_tbl.itemChanged.connect(self._on_limit_edit)
        self._fill_limits([-180.0] * table.n, [180.0] * table.n)

        self.read_btn = QPushButton("读取设备限位")
        self.read_btn.clicked.connect(self._read_limits)
        self.source_lbl = QLabel()
        self.source_lbl.setObjectName("title")
        self.source_lbl.setWordWrap(True)
        self._update_source()

        limit_box = QGroupBox("关节限位")
        ll = QVBoxLayout(limit_box)
        ll.addWidget(self.limit_tbl, 1)
        ll.addWidget(self.read_btn)
        ll.addWidget(self.source_lbl)

        self.samples = QSpinBox()
        self.samples.setRange(2_000, 500_000)
        self.samples.setSingleStep(10_000)
        self.samples.setValue(100_000)
        self.samples.setGroupSeparatorShown(True)
        self.branch = QComboBox()
        self.branch.addItem("全部 8 个分支", None)
        for b in range(8):
            self.branch.addItem(f"分支 {branch_text(b)}", b)
        self.branch.setToolTip("按 λ2 / λ4 / λ6 的符号过滤构型；全部=8 个分支汇总")
        self.voxel_mm = QDoubleSpinBox()
        self.voxel_mm.setRange(5.0, 50.0)
        self.voxel_mm.setDecimals(1)
        self.voxel_mm.setSingleStep(1.0)
        self.voxel_mm.setValue(15.0)
        self.voxel_mm.setSuffix(" mm")
        self.show_cb = QCheckBox("在三维视图中显示")
        self.show_cb.setChecked(True)
        self.show_cb.toggled.connect(self.visibility_changed)
        self.fit_cb = QCheckBox("计算后适配视角")
        self.fit_cb.setChecked(True)
        self.samples.valueChanged.connect(self._sample_changed)
        self.voxel_mm.valueChanged.connect(self._sample_changed)

        sample_box = QGroupBox("采样")
        sg = QGridLayout(sample_box)
        sg.addWidget(QLabel("构型数"), 0, 0)
        sg.addWidget(self.samples, 0, 1)
        sg.addWidget(QLabel("IK 分支"), 1, 0)
        sg.addWidget(self.branch, 1, 1)
        sg.addWidget(QLabel("体素尺寸"), 2, 0)
        sg.addWidget(self.voxel_mm, 2, 1)
        sg.addWidget(self.show_cb, 3, 0, 1, 2)
        sg.addWidget(self.fit_cb, 4, 0, 1, 2)

        self.calc_btn = QPushButton("计算并显示")
        self.calc_btn.clicked.connect(self.calculate)
        self.clear_btn = QPushButton("清除点云")
        self.clear_btn.clicked.connect(self.clear)
        self.fit_btn = QPushButton("适配视角")
        self.fit_btn.clicked.connect(self.fit_requested)
        actions = QHBoxLayout()
        actions.addWidget(self.calc_btn)
        actions.addWidget(self.clear_btn)
        actions.addWidget(self.fit_btn)

        self.status = QLabel("在关节限位内采样 100000 个构型，计算末端位置可达点云")
        self.status.setWordWrap(True)
        self.status.setObjectName("title")
        hint = QLabel("提示：点云表示末端位置；用上方“前视 / 侧视 / 俯视”"
                      "可看三个投影包络。")
        hint.setWordWrap(True)
        hint.setObjectName("title")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(limit_box)
        lay.addWidget(sample_box)
        lay.addLayout(actions)
        lay.addWidget(self.status)
        lay.addWidget(hint)
        lay.addStretch(1)

        self._poll_timer = QTimer(self, interval=50, timeout=self._poll_limits)
        self.branch.currentIndexChanged.connect(self._branch_changed)
        self._sample_changed()

    def set_table(self, table: DHTable) -> None:
        old_n = self._table.n
        self._table = table
        if old_n != table.n:
            self._fill_limits([-180.0] * table.n, [180.0] * table.n)
            self._source = "默认 ±180°"
            self._update_source()
        self.invalidate()

    def limits_deg(self) -> tuple[tuple[float, float], ...]:
        out = []
        for i in range(self._table.n):
            try:
                lo = float(self.limit_tbl.item(i, 0).text())
                hi = float(self.limit_tbl.item(i, 1).text())
            except (AttributeError, ValueError):
                raise ValueError(f"J{i} 限位不是有效数字")
            if not (lo < hi):
                raise ValueError(f"J{i} 限位必须满足 最小 < 最大")
            out.append((lo, hi))
        return tuple(out)

    def calculate(self) -> None:
        try:
            limits = self.limits_deg()
            QGuiApplication.setOverrideCursor(Qt.WaitCursor)
            try:
                cloud = sample_workspace(self._table, limits, samples=self.samples.value(),
                                         voxel_m=self.voxel_mm.value() / 1000.0,
                                         branch=self.branch.currentData())
            finally:
                QGuiApplication.restoreOverrideCursor()
        except Exception as exc:
            self.status.setText(f"计算失败：{exc}")
            return

        self._has_result = True
        voxel_note = "" if abs(cloud.voxel_m - self.voxel_mm.value() / 1000.0) < 1e-9 \
            else f"（自动放粗到 {cloud.voxel_m * 1000:.1f} mm）"
        branch_info = "全部 8 分支" if cloud.branch is None else f"分支 #{cloud.branch}"
        self.status.setText(f"{branch_info}，采样 {cloud.samples:,} → {len(cloud.points_m):,} 个体素"
                            f"{voxel_note}，耗时 {cloud.elapsed_ms:.0f} ms")
        self.cloud_ready.emit(cloud)
        if self.fit_cb.isChecked():
            self.fit_requested.emit()

    def clear(self) -> None:
        self._has_result = False
        self.status.setText("点云已清除；修改限位或 DH 后请重新计算")
        self.cleared.emit()

    def invalidate(self) -> None:
        if self._has_result:
            self._has_result = False
            self.cleared.emit()
        self.status.setText("DH 或关节限位已变化，请重新计算工作空间")

    def _sample_changed(self) -> None:
        if self._has_result:
            self.invalidate()
        branch = self.branch.currentData()
        branch_info = "全部 8 分支" if branch is None else f"分支 #{branch}"
        self.status.setText(f"{branch_info}：将采样 {self.samples.value():,} 个构型，体素 "
                            f"{self.voxel_mm.value():.1f} mm")

    def _branch_changed(self) -> None:
        self._sample_changed()

    def _fill_limits(self, mins, maxs) -> None:
        self._loading = True
        self.limit_tbl.setRowCount(len(mins))
        self.limit_tbl.setVerticalHeaderLabels([f"J{i}" for i in range(len(mins))])
        for i, (lo, hi) in enumerate(zip(mins, maxs)):
            for col, value in enumerate((lo, hi)):
                it = QTableWidgetItem(f"{float(value):.1f}")
                it.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.limit_tbl.setItem(i, col, it)
        self._loading = False

    def _on_limit_edit(self, _item) -> None:
        if self._loading:
            return
        self._source = "手动限位"
        self._update_source()
        self.invalidate()

    def _update_source(self) -> None:
        self.source_lbl.setText(f"当前来源：{self._source}")

    def _read_limits(self) -> None:
        if self._pending is not None:
            return
        if self._read_device_limits is None:
            self.status.setText("当前界面没有接入设备限位读取接口，可手动填写")
            return
        try:
            fut = self._read_device_limits()
        except Exception as exc:
            self.status.setText(f"读取限位失败：{exc}")
            return
        if fut is None:
            self.status.setText("当前数据源不支持读取关节限位，可手动填写")
            return
        self._pending = fut
        self._pending_started = time.monotonic()
        self.read_btn.setEnabled(False)
        self.status.setText("正在读取设备关节限位…")
        self._poll_timer.start()

    def _poll_limits(self) -> None:
        fut = self._pending
        if fut is None:
            self._poll_timer.stop()
            return
        if not fut.done():
            if time.monotonic() - self._pending_started > 12.0:
                self._pending = None
                self._poll_timer.stop()
                self.read_btn.setEnabled(True)
                self.status.setText("读取设备限位超时，可重试或手动填写")
            return
        self._pending = None
        self._poll_timer.stop()
        self.read_btn.setEnabled(True)
        try:
            mins, maxs = fut.result()
            if len(mins) != self._table.n or len(maxs) != self._table.n:
                raise ValueError(f"设备返回 {len(mins)} 轴限位，界面是 {self._table.n} 轴")
        except Exception as exc:
            self.status.setText(f"读取设备限位失败：{exc}")
            return
        self._fill_limits(mins, maxs)
        self._source = "设备固件"
        self._update_source()
        self.invalidate()
        self.status.setText("设备限位已载入，请重新计算工作空间")
