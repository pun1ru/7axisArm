"""实时波形：多通道曲线、变量选择、颜色、图例、缩放平移、暂停恢复、滚动时间窗口。

刷新策略（README #4 第 4、7 条）：
* 曲线对象在构造时一次建好（关节数 × 量），之后只 setData，不为数据点建 UI 对象；
* 定时器 30 Hz 从 SampleBuffer 批量取一次，不是每收到一帧刷新一次；
* 只给勾选的通道 setData，未勾选的隐藏；clipToView + 自动降采样控制每帧点数。
暂停只冻结显示，不影响接收与记录；暂停后可在整个缓存里自由平移缩放。
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from pyqtgraph.graphicsItems.PlotCurveItem import PlotCurveItem as _PlotCurveItem
from PySide6.QtCore import QEvent, Qt, QTimer, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QCheckBox, QColorDialog, QComboBox, QGridLayout, QHBoxLayout, QLabel,
                               QPushButton, QSizePolicy, QSplitter, QVBoxLayout, QWidget)

from ..model import GROUP_TITLE, GROUPS, QKEYS, QUANTITIES, TIME_HOST, TIME_MCU, BufferView, SampleBuffer
from . import theme
from .layout_util import hscrollable

WINDOWS_S = (2, 5, 10, 20, 30, 60)
DASHED = {"target_deg", "torque_cmd_nm"}
DEFAULT_ON = {"angle_deg", "velocity_dps", "torque_nm"}


# PySide6 6.11 + Python 3.14 can crash natively in pyqtgraph's
# PlotCurveItem.pixelPadding -> pixelVectors -> Point(QPointF) path.
# The padding is only a sub-pixel bounding-box margin for a 1 px cosmetic
# pen, so disabling it keeps rendering unchanged and avoids the crash.
if not getattr(_PlotCurveItem, "_armhost_safe_pixel_padding", False):
    _PlotCurveItem.pixelPadding = lambda self: 0.0
    _PlotCurveItem._armhost_safe_pixel_padding = True


def decimate_minmax(x: np.ndarray, ys: np.ndarray, bins: int) -> tuple[np.ndarray, np.ndarray]:
    """所有勾选通道一起做 min/max 抽稀：每个桶输出 (min, max) 两个点，峰值不丢。

    x: (n,)   ys: (n, c)  ->  x2: (2*bins,)  ys2: (2*bins, c)
    一次 numpy reshape + reduce，比 pyqtgraph 逐条曲线降采样快一个数量级。
    """
    n = len(x)
    if n <= 2 * bins:
        return x, ys
    k = n // bins
    m = k * bins
    off = n - m                        # 从头部丢掉余数，保证最新样本一定在里面
    xb = x[off:].reshape(bins, k)
    yb = ys[off:].reshape(bins, k, -1)
    x2 = np.repeat(xb[:, 0], 2)
    x2[1::2] = xb[:, -1]
    y2 = np.empty((2 * bins, ys.shape[1]), ys.dtype)
    y2[0::2] = yb.min(axis=1)
    y2[1::2] = yb.max(axis=1)
    return x2, y2


class ChannelSelector(QWidget):
    """行 = 关节，列 = 量；首列是颜色按钮。"""
    changed = Signal()
    color_changed = Signal(int, QColor)

    def __init__(self, joint_count: int, parent=None) -> None:
        super().__init__(parent)
        self.colors = [QColor(theme.JOINT_COLORS[i % len(theme.JOINT_COLORS)]) for i in range(joint_count)]
        self.boxes: dict[tuple[int, str], QCheckBox] = {}
        g = QGridLayout(self)
        g.setContentsMargins(4, 4, 4, 4)
        g.setHorizontalSpacing(6)
        g.setVerticalSpacing(2)
        g.addWidget(QLabel("关节"), 0, 0)
        for c, q in enumerate(QUANTITIES, start=1):
            lb = QLabel(q.label)
            lb.setObjectName("title")
            lb.setToolTip(f"{q.key} [{q.unit}]")
            g.addWidget(lb, 0, c, Qt.AlignCenter)
            # 列头按钮：整列全选 / 全不选
            btn = QPushButton("全")
            btn.setFixedWidth(34)
            btn.setToolTip(f"切换整列：{q.label}")
            btn.clicked.connect(lambda _=False, k=q.key: self.toggle_column(k))
            g.addWidget(btn, joint_count + 1, c, Qt.AlignCenter)
        self.color_btns: list[QPushButton] = []
        for j in range(joint_count):
            b = QPushButton(f"J{j}")
            b.setFixedWidth(40)
            b.setToolTip("点击修改颜色")
            b.clicked.connect(lambda _=False, jj=j: self._pick_color(jj))
            self.color_btns.append(b)
            g.addWidget(b, j + 1, 0)
            for c, q in enumerate(QUANTITIES, start=1):
                cb = QCheckBox()
                cb.setChecked(q.key in DEFAULT_ON)
                cb.toggled.connect(self.changed)
                self.boxes[(j, q.key)] = cb
                g.addWidget(cb, j + 1, c, Qt.AlignCenter)
            self._paint(j)
        g.setRowStretch(joint_count + 2, 1)
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)

    def _paint(self, j: int) -> None:
        c = self.colors[j]
        fg = "black" if c.lightness() > 140 else "white"
        self.color_btns[j].setStyleSheet(f"background:{c.name()}; color:{fg}; border:0; font-weight:bold;")

    def _pick_color(self, j: int) -> None:
        c = QColorDialog.getColor(self.colors[j], self, f"J{j} 曲线颜色")
        if c.isValid():
            self.colors[j] = c
            self._paint(j)
            self.color_changed.emit(j, c)

    def toggle_column(self, key: str) -> None:
        col = [cb for (j, k), cb in self.boxes.items() if k == key]
        target = not all(cb.isChecked() for cb in col)
        for cb in col:
            cb.blockSignals(True)
            cb.setChecked(target)
            cb.blockSignals(False)
        self.changed.emit()

    def selected(self) -> list[tuple[int, str]]:
        return [k for k, cb in self.boxes.items() if cb.isChecked()]


class ScopeWidget(QWidget):
    paused_changed = Signal(bool)

    def __init__(self, buffer: SampleBuffer, joint_count: int = 7, fps: int = 30, parent=None) -> None:
        super().__init__(parent)
        self.buffer = buffer
        self.joint_count = joint_count
        self.window_s = 10.0
        self.time_base = TIME_MCU
        self.paused = False
        self._frozen: BufferView | None = None
        self.frames = 0

        # ---------- 工具条
        bar = QHBoxLayout()
        bar.setContentsMargins(0, 0, 0, 0)
        bar.addWidget(QLabel("时间窗口"))
        self.win_combo = QComboBox()
        for s in WINDOWS_S:
            self.win_combo.addItem(f"{s} s", s)
        self.win_combo.setCurrentIndex(WINDOWS_S.index(10))
        self.win_combo.currentIndexChanged.connect(self._on_window)
        bar.addWidget(self.win_combo)
        bar.addSpacing(8)
        bar.addWidget(QLabel("时间轴"))
        self.base_combo = QComboBox()
        self.base_combo.addItem("MCU 采样时间", TIME_MCU)
        self.base_combo.addItem("上位机接收时间", TIME_HOST)
        self.base_combo.setToolTip("MCU 采样时间：帧内时间戳，反映真实采样间隔\n"
                                   "上位机接收时间：到达本机的时刻，含传输延迟与成批到达")
        self.base_combo.currentIndexChanged.connect(self._on_base)
        bar.addWidget(self.base_combo)
        bar.addSpacing(8)
        self.pause_btn = QPushButton("暂停")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setShortcut("Space")
        self.pause_btn.setToolTip("暂停 / 恢复显示（空格）。暂停不影响接收和记录")
        self.pause_btn.toggled.connect(self.set_paused)
        bar.addWidget(self.pause_btn)
        self.follow_btn = QPushButton("跟随最新")
        self.follow_btn.setToolTip("恢复滚动显示")
        self.follow_btn.clicked.connect(lambda: self.set_paused(False))
        bar.addWidget(self.follow_btn)
        self.yauto_btn = QPushButton("Y 自动")
        self.yauto_btn.setToolTip("所有图的 Y 轴恢复自动缩放")
        self.yauto_btn.clicked.connect(self.auto_y)
        bar.addWidget(self.yauto_btn)
        self.legend_btn = QPushButton("图例")
        self.legend_btn.setCheckable(True)
        self.legend_btn.setChecked(True)
        self.legend_btn.setToolTip("显示 / 隐藏图例（通道多时图例会挡住曲线）")
        self.legend_btn.toggled.connect(lambda on: [lg.setVisible(on) for lg in self.legends.values()])
        bar.addWidget(self.legend_btn)
        self.cross_btn = QPushButton("十字光标")
        self.cross_btn.setCheckable(True)
        self.cross_btn.setChecked(True)
        self.cross_btn.setToolTip("鼠标放在图上时，竖线所在时刻各曲线的数值")
        self.cross_btn.toggled.connect(self._on_cross_toggled)
        bar.addWidget(self.cross_btn)
        bar.addSpacing(8)
        bar.addWidget(QLabel("显示"))
        self.solo_combo = QComboBox()
        self.solo_combo.addItem("全部三幅", None)
        for g in GROUPS:
            self.solo_combo.addItem(f"只看 {GROUP_TITLE[g]}", g)
        self.solo_combo.setToolTip("放大其中一幅图，占满三幅的面积；在图上双击也可以切换")
        self.solo_combo.currentIndexChanged.connect(lambda: self.set_solo(self.solo_combo.currentData()))
        bar.addWidget(self.solo_combo)
        bar.addStretch(1)
        self.info = QLabel("")
        self.info.setObjectName("title")
        bar.addWidget(self.info)

        # ---------- 图
        self.glw = pg.GraphicsLayoutWidget()
        self.plots: dict[str, pg.PlotItem] = {}
        self.legends: dict[str, pg.LegendItem] = {}
        self.vlines: dict[str, pg.InfiniteLine] = {}
        self.readouts: dict[str, pg.TextItem] = {}
        self.solo: str | None = None
        first = None
        for r, g in enumerate(GROUPS):
            p = self.glw.addPlot(row=r, col=0)
            p.showGrid(x=True, y=True, alpha=theme.GRID_ALPHA)
            p.setLabel("left", GROUP_TITLE[g])
            p.getAxis("left").enableAutoSIPrefix(False)      # 单位已在标题里，不要自动加 ×0.001
            p.getViewBox().sigRangeChangedManually.connect(self._on_user_range)
            lg = p.addLegend(offset=(6, 6), labelTextSize="8pt", colCount=4)
            lg.setBrush(pg.mkBrush(21, 23, 27, 210))          # 半透明底，盖住曲线时仍可读
            lg.setPen(pg.mkPen(theme.BORDER))
            self.legends[g] = lg
            # 十字光标：竖线 + 数值框，ignoreBounds 不参与自动缩放
            vl = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#c8ccd4", width=1, style=Qt.DotLine))
            vl.setVisible(False)
            p.addItem(vl, ignoreBounds=True)
            self.vlines[g] = vl
            ro = pg.TextItem(anchor=(0, 0), fill=pg.mkBrush(21, 23, 27, 225), border=pg.mkPen(theme.BORDER))
            ro.setZValue(1000)
            ro.setVisible(False)
            p.addItem(ro, ignoreBounds=True)
            self.readouts[g] = ro
            if first is None:
                first = p
            else:
                p.setXLink(first)
            self.plots[g] = p
        self._apply_axes()
        self._mouse_x: float | None = None          # 光标所在 x（数据坐标），None = 不在图上
        self._last_x: np.ndarray | None = None      # 最近一次刷新的原始 x / 数据，读数用原始点而不是抽稀后的点
        self._last_v: BufferView | None = None
        self._mouse_proxy = pg.SignalProxy(self.glw.scene().sigMouseMoved, rateLimit=60, slot=self._on_mouse_moved)
        self.glw.scene().sigMouseClicked.connect(self._on_scene_click)
        self.glw.viewport().installEventFilter(self)

        # ---------- 曲线（一次性建好）
        qmap = {q.key: q for q in QUANTITIES}
        self.selector = ChannelSelector(joint_count)
        self.curves: dict[tuple[int, str], pg.PlotDataItem] = {}
        for j in range(joint_count):
            for q in QUANTITIES:
                c = pg.PlotDataItem(name=f"J{j} {q.label}", pen=self._pen(j, q.key), skipFiniteCheck=True)
                c.setVisible(False)
                self.plots[qmap[q.key].group].addItem(c)
                self.curves[(j, q.key)] = c
        self.selector.changed.connect(self._apply_selection)
        self.selector.color_changed.connect(self._recolor)
        self._apply_selection()

        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.glw)
        split.addWidget(self.selector)
        split.setStretchFactor(0, 1)
        split.setCollapsible(1, True)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        bar_widget = QWidget()
        bar_widget.setLayout(bar)
        lay.addWidget(hscrollable(bar_widget))      # 窗口窄时工具条横向滚动，不撑大窗口
        lay.addWidget(split, 1)

        self.timer = QTimer(self)
        self.timer.setInterval(int(1000 / fps))
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        self._zoom_timer = QTimer(self, singleShot=True, interval=50)
        self._zoom_timer.timeout.connect(lambda: self.refresh(force=True))

    # ------------------------------------------------------------------ 外观
    def _pen(self, j: int, key: str):
        return pg.mkPen(self.selector.colors[j] if hasattr(self, "selector") else theme.JOINT_COLORS[j % 7],
                        width=1, style=Qt.DashLine if key in DASHED else Qt.SolidLine)

    def _recolor(self, j: int, _c: QColor) -> None:
        for q in QUANTITIES:
            self.curves[(j, q.key)].setPen(self._pen(j, q.key))
        self._apply_selection()       # 让图例色块跟着变

    def _visible_groups(self) -> list[str]:
        return [self.solo] if self.solo else list(GROUPS)

    def _main_vb(self) -> pg.ViewBox:
        """当前显示的第一幅图的 ViewBox（放大单图时被隐藏的图尺寸为 0，不能拿来算宽度）"""
        return self.plots[self._visible_groups()[0]].getViewBox()

    def _apply_axes(self) -> None:
        """只有最下面那幅可见图显示时间刻度和横轴标题"""
        lbl = "MCU 采样时间" if self.time_base == TIME_MCU else "上位机接收时间"
        vis = self._visible_groups()
        for g, p in self.plots.items():
            last = g == vis[-1]
            ax = p.getAxis("bottom")
            ax.setStyle(showValues=last)
            p.setLabel("bottom", f"{lbl}，相对最新样本 (s)" if last else "")
            ax.showLabel(last)

    def set_solo(self, group: str | None) -> None:
        """放大单幅图（占满原来三幅的面积），None 恢复三幅"""
        if group == self.solo:
            return
        self.solo = group
        # 不把图移出布局（pyqtgraph 的 removeItem 会顺带移出场景，再加回来尺寸会错乱），
        # 只隐藏其余两幅并把它们的高度压到 0，让可见的那幅占满
        lay = self.glw.ci.layout
        vis = self._visible_groups()
        for r, g in enumerate(GROUPS):
            p = self.plots[g]
            on = g in vis
            p.setVisible(on)
            p.setMinimumHeight(0)
            p.setMaximumHeight(16777215 if on else 0)
            lay.setRowStretchFactor(r, 1 if on else 0)
            lay.setRowSpacing(r, 0)
        self.glw.ci.updateGeometry()
        self._apply_axes()
        i = self.solo_combo.findData(group)
        if i != self.solo_combo.currentIndex():
            self.solo_combo.blockSignals(True)
            self.solo_combo.setCurrentIndex(i)
            self.solo_combo.blockSignals(False)
        self.refresh(force=True)

    def _on_scene_click(self, ev) -> None:
        if not ev.double():
            return
        for g in self._visible_groups():
            if self.plots[g].sceneBoundingRect().contains(ev.scenePos()):
                self.set_solo(None if self.solo == g else g)
                ev.accept()
                return

    # ------------------------------------------------------------------ 十字光标
    def eventFilter(self, obj, ev) -> bool:
        if obj is self.glw.viewport() and ev.type() == QEvent.Leave:
            self._mouse_x = None
            self._update_crosshair()
        return super().eventFilter(obj, ev)

    def _on_cross_toggled(self, on: bool) -> None:
        if not on:
            self._mouse_x = None
        self._update_crosshair()

    def _on_mouse_moved(self, args) -> None:
        if not self.cross_btn.isChecked():
            return
        pos = args[0]
        self._mouse_x = None
        for g in self._visible_groups():
            p = self.plots[g]
            if p.sceneBoundingRect().contains(pos):
                self._mouse_x = float(p.getViewBox().mapSceneToView(pos).x())
                break
        self._update_crosshair()

    def _update_crosshair(self) -> None:
        """在每幅可见图上画竖线，并列出该时刻本图所有已勾选曲线的数值（取最近的原始样本）"""
        x = self._mouse_x
        x_raw, v = self._last_x, self._last_v
        show = x is not None and x_raw is not None and v is not None and len(x_raw) > 0
        if not show:
            for g in GROUPS:
                self.vlines[g].setVisible(False)
                self.readouts[g].setVisible(False)
            return
        i = int(np.clip(np.searchsorted(x_raw, x), 1, len(x_raw) - 1))
        i = i - 1 if abs(x_raw[i - 1] - x) <= abs(x_raw[i] - x) else i
        xi = float(x_raw[i])
        outside = x < x_raw[0] or x > x_raw[-1]
        qmap = {q.key: q for q in QUANTITIES}
        sel = self.selector.selected()
        for g in GROUPS:
            visible = g in self._visible_groups()
            vl, ro = self.vlines[g], self.readouts[g]
            vl.setVisible(visible)
            if not visible:
                ro.setVisible(False)
                continue
            vl.setPos(xi)
            rows = [f"<span style='color:{theme.MUTED}'>t = {xi:+.4f} s{'（超出数据）' if outside else ''}</span>"]
            for j, k in sel:
                q = qmap[k]
                if q.group != g:
                    continue
                c = self.selector.colors[j].name()
                val = float(v.data[i, j, QKEYS.index(k)])
                rows.append(f"<span style='color:{c}'>J{j} {q.label}</span>&nbsp;"
                            f"<span style='color:{theme.TEXT}'>{val:+.4f} {q.unit}</span>")
            ro.setHtml("<div style='font-size:8pt; font-family:Consolas,monospace'>"
                       + "<br>".join(rows) + "</div>")
            vb = self.plots[g].getViewBox()
            (x0, x1), (y0, y1) = vb.viewRange()
            right_half = xi > (x0 + x1) / 2
            ro.setAnchor((1, 0) if right_half else (0, 0))     # 靠右时文字放在竖线左边，不出界
            ro.setPos(xi, y1)
            ro.setVisible(True)

    def _apply_selection(self) -> None:
        sel = set(self.selector.selected())
        for lg in self.legends.values():
            lg.clear()
        qmap = {q.key: q for q in QUANTITIES}
        for key, c in self.curves.items():
            on = key in sel
            c.setVisible(on)
            if on:
                self.legends[qmap[key[1]].group].addItem(c, c.name())
            else:
                c.setData([], [])
        self.refresh(force=True)

    # ------------------------------------------------------------------ 交互
    def _on_window(self) -> None:
        self.window_s = float(self.win_combo.currentData())
        self.refresh(force=True)

    def _on_base(self) -> None:
        self.time_base = self.base_combo.currentData()
        self._apply_axes()
        self.refresh(force=True)

    def _on_user_range(self, *_):
        # 用户拖动 / 缩放：说明要看细节，自动进入暂停，避免下一帧把视图拉回去
        if not self.paused:
            self.set_paused(True)
        else:
            self._zoom_timer.start()      # 暂停时按新的可见范围重新抽稀，放大后能看到原始点

    def set_paused(self, paused: bool) -> None:
        if paused == self.paused:
            return
        self.paused = paused
        self.pause_btn.blockSignals(True)
        self.pause_btn.setChecked(paused)
        self.pause_btn.setText("恢复" if paused else "暂停")
        self.pause_btn.blockSignals(False)
        self._frozen = self.buffer.view(base=self.time_base) if paused else None
        if paused:
            self.refresh(force=True)
        else:
            self.auto_y()
        self.paused_changed.emit(paused)

    def auto_y(self) -> None:
        for p in self.plots.values():
            p.enableAutoRange(axis="y", enable=True)
        self.refresh(force=True)

    def visible_channels(self) -> list[tuple[int, str]]:
        return self.selector.selected()

    def current_view(self) -> BufferView:
        """统计面板用：暂停时是冻结的数据，否则是当前窗口"""
        if self._frozen is not None:
            return self._frozen
        return self.buffer.view(self.window_s, self.time_base)

    # ------------------------------------------------------------------ 刷新
    def refresh(self, force: bool = False) -> None:
        if self.paused and not force:
            return
        if self.paused and self._frozen is not None:
            v = self._frozen
        else:
            v = self.buffer.view(self.window_s * 1.05, self.time_base)
        n = len(v)
        if n == 0:
            self.info.setText("无数据")
            return
        if v.joint_count != self.joint_count:
            self.info.setText(f"关节数变化：{v.joint_count}（重新打开窗口生效）")
            return
        t = v.time(self.time_base)
        x = (t - t[-1]).astype(np.float64)
        sel = self.selector.selected()
        vb = self._main_vb()
        i0, i1 = 0, len(x)
        if sel:
            if self.paused:
                # 只抽稀当前可见的那一段（左右各留一点余量，平移时边缘不空）
                x0, x1 = vb.viewRange()[0]
                pad = (x1 - x0) * 0.05
                i0, i1 = np.searchsorted(x, (x0 - pad, x1 + pad))
                i0, i1 = max(0, i0 - 1), min(len(x), i1 + 1)
            jidx = np.fromiter((j for j, _ in sel), int, len(sel))
            qidx = np.fromiter((QKEYS.index(k) for _, k in sel), int, len(sel))
            ys = v.data[i0:i1, jidx, qidx]
            bins = max(200, int(vb.width()) or 1000)
            xs, ys = decimate_minmax(x[i0:i1], ys, bins)
            for c, key in enumerate(sel):
                self.curves[key].setData(xs, ys[:, c], skipFiniteCheck=True)
        # 十字光标读的是原始点；保存一份引用即可（view() 返回的本来就是拷贝）
        self._last_x = x
        self._last_v = v
        if not self.paused:
            vb.setXRange(-self.window_s, 0.0, padding=0.0, update=False)
        self._update_crosshair()
        self.frames += 1
        span = t[-1] - t[0]
        self.info.setText(f"{'已暂停  ' if self.paused else ''}点数 {n}  跨度 {span:.1f} s  通道 {len(sel)}")
