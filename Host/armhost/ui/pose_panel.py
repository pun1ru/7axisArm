"""三维姿态面板 = View3D + 显示开关 + 数据来源 + 手动滑块 + DH 参数表编辑。

与串口完全解耦：面板只认一个"取关节角"的函数。
  实时数据：controller.latest() 里的 angle_deg（固件 observer.joint_deg，多圈、已按 direction 折算）
  手动    ：滑块，用来离线核对 DH 表
渲染节流：面板定时器 30 Hz 拉一次关节角；角度没变就不重绘。
"""

from __future__ import annotations

from concurrent.futures import Future
from pathlib import Path
from typing import Callable

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (QAbstractItemView, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox,
                               QFileDialog, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel,
                               QMessageBox, QPushButton, QRadioButton, QScrollArea, QSlider, QSplitter,
                               QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget)

from ..kinematics import DHRow, DHTable, default_dh
from ..model import RobotSample
from ..protocol.enums import MotorStatus
from .view3d import View3D

CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
DH_FILE = CONFIG_DIR / "dh.json"
DH_COLS = (("alpha (deg)", "alpha_deg", 3), ("a (m)", "a_m", 5), ("d (m)", "d_m", 5), ("θ偏置 (deg)", "theta_offset_deg", 3))


class DHEditor(QWidget):
    """DH 参数表编辑。默认读单片机工程 module_init.c；保存到 config/dh.json 后启动优先用它。"""
    applied = Signal(object)            # DHTable

    def __init__(self, table: DHTable, read_device: Callable[[], Future | None] | None = None, parent=None) -> None:
        super().__init__(parent)
        self._read_device = read_device
        self.src = QLabel()
        self.src.setObjectName("title")
        self.src.setWordWrap(True)
        self.tbl = QTableWidget(0, len(DH_COLS))
        self.tbl.setHorizontalHeaderLabels([c[0] for c in DH_COLS])
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl.verticalHeader().setDefaultSectionSize(24)
        self.tbl.setSelectionMode(QAbstractItemView.NoSelection)
        self.auto = QCheckBox("改动后立即生效")
        self.auto.setChecked(True)
        b_apply = QPushButton("应用")
        b_fw = QPushButton("恢复单片机默认")
        b_fw.setToolTip("重新读取 Real/DIYcontroller/App/module_init.c 的 ARM_DH_Jx（只读，不改固件）")
        self.b_dev = QPushButton("从设备读取")
        self.b_dev.setToolTip("通过 PARAM_READ 0x0400 读取单片机当前 RAM 里的 DH 表（需要固件实现参数读写）")
        b_load = QPushButton("导入…")
        b_save = QPushButton("保存为默认")
        b_save.setToolTip(f"保存到 {DH_FILE.relative_to(DH_FILE.parents[1])}，下次启动优先使用")
        b_export = QPushButton("导出…")
        b_apply.clicked.connect(self._apply)
        b_fw.clicked.connect(lambda: self.set_table(default_dh(), emit=True))
        self.b_dev.clicked.connect(self._from_device)
        b_load.clicked.connect(self._import)
        b_save.clicked.connect(lambda: self._save(DH_FILE))
        b_export.clicked.connect(self._export)
        self.tbl.itemChanged.connect(self._on_edit)

        grid = QGridLayout()
        for k, b in enumerate((b_apply, b_fw, self.b_dev, b_load, b_save, b_export)):
            grid.addWidget(b, k // 3, k % 3)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(QLabel("改进 DH（Craig），第 i 行对应关节 Ji：T = RotX(α)·TransX(a)·RotZ(q+θ偏置)·TransZ(d)"))
        lay.addWidget(self.tbl, 1)
        lay.addWidget(self.auto)
        lay.addLayout(grid)
        lay.addWidget(self.src)
        self._loading = False
        self.set_table(table)

    def set_table(self, t: DHTable, emit: bool = False) -> None:
        self._loading = True
        self.tbl.setRowCount(t.n)
        self.tbl.setVerticalHeaderLabels([f"J{i}" for i in range(t.n)])
        for r, row in enumerate(t.rows):
            for c, (_, key, dec) in enumerate(DH_COLS):
                it = QTableWidgetItem(f"{getattr(row, key):.{dec + 3}g}")
                it.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.tbl.setItem(r, c, it)
        self._loading = False
        self._source = t.source
        self.src.setText(f"当前来源：{t.source}")
        if emit:
            self.applied.emit(t)

    def table(self) -> DHTable:
        rows = []
        for r in range(self.tbl.rowCount()):
            vals = []
            for c in range(len(DH_COLS)):
                txt = self.tbl.item(r, c).text().strip()
                try:
                    vals.append(float(txt))
                except ValueError:
                    raise ValueError(f"J{r} 第 {c + 1} 列不是数字：{txt!r}") from None
            rows.append(DHRow(*vals))
        return DHTable(rows, "手动编辑")

    def _on_edit(self, _it) -> None:
        if self._loading or not self.auto.isChecked():
            return
        try:
            t = self.table()
        except ValueError:
            return                          # 输入到一半（比如只有 "-"）先不生效
        self.src.setText("当前来源：手动编辑（未保存）")
        self.applied.emit(t)

    def _apply(self) -> None:
        try:
            self.applied.emit(self.table())
        except ValueError as e:
            QMessageBox.warning(self, "DH 参数", str(e))

    def _save(self, path: Path) -> None:
        try:
            t = self.table()
            path.parent.mkdir(parents=True, exist_ok=True)
            t.save_json(path)
            self.src.setText(f"当前来源：已保存到 {path.name}")
        except (ValueError, OSError) as e:
            QMessageBox.warning(self, "保存失败", str(e))

    def _export(self) -> None:
        p, _ = QFileDialog.getSaveFileName(self, "导出 DH", str(CONFIG_DIR / "dh_export.json"), "JSON (*.json)")
        if p:
            self._save(Path(p))

    def _import(self) -> None:
        p, _ = QFileDialog.getOpenFileName(self, "导入 DH", str(CONFIG_DIR), "JSON (*.json)")
        if p:
            try:
                self.set_table(DHTable.load_json(p), emit=True)
            except (OSError, ValueError, KeyError) as e:
                QMessageBox.warning(self, "导入失败", str(e))

    def _from_device(self) -> None:
        fut = self._read_device() if self._read_device else None
        if fut is None:
            QMessageBox.information(self, "从设备读取", "当前数据源不支持参数读取（需要连上串口设备或模拟设备）")
            return
        self.b_dev.setEnabled(False)
        self.src.setText("当前来源：正在从设备读取…")
        self._pending = fut
        QTimer.singleShot(50, self._poll_device)

    def _poll_device(self) -> None:
        fut = self._pending
        if not fut.done():
            QTimer.singleShot(50, self._poll_device)
            return
        self.b_dev.setEnabled(True)
        try:
            self.set_table(fut.result(), emit=True)
        except Exception as e:
            self.src.setText(f"当前来源：{self._source}（设备读取失败：{e}）")


def load_startup_dh() -> DHTable:
    """启动顺序：config/dh.json（用户保存过）→ 单片机工程 → 内置副本"""
    if DH_FILE.exists():
        try:
            return DHTable.load_json(DH_FILE)
        except (OSError, ValueError, KeyError):
            pass
    return default_dh()


class PosePanel(QWidget):
    SRC_LIVE, SRC_MANUAL = "live", "manual"

    def __init__(self, get_sample: Callable[[], RobotSample | None],
                 read_device_dh: Callable[[], Future | None] | None = None,
                 table: DHTable | None = None, parent=None, fps: int = 30) -> None:
        super().__init__(parent)
        self._get_sample = get_sample
        table = table or load_startup_dh()
        self.view = View3D(table, max_fps=fps)
        self.view.joint_clicked.connect(self._select_joint)

        # ---------------- 视角 / 显示
        views = QHBoxLayout()
        for name, txt in (("iso", "等轴"), ("front", "前视"), ("side", "侧视"), ("top", "俯视")):
            b = QPushButton(txt)
            b.clicked.connect(lambda _=False, n=name: self.view.reset_view(n))
            views.addWidget(b)
        disp = QGroupBox("显示")
        dg = QGridLayout(disp)
        self.checks: dict[str, QCheckBox] = {}
        for k, (attr, txt) in enumerate((("show_links", "连杆"), ("show_frames", "关节坐标系"),
                                         ("show_axes_line", "关节转轴"), ("show_labels", "关节名称"),
                                         ("show_grid", "地面网格"), ("show_world", "世界坐标轴"))):
            cb = QCheckBox(txt)
            cb.setChecked(getattr(self.view, attr))
            cb.toggled.connect(lambda on, a=attr: self.view.set_option(a, on))
            self.checks[attr] = cb
            dg.addWidget(cb, k // 3, k % 3)
        # 每个坐标系单独开关
        fr = QHBoxLayout()
        fr.addWidget(QLabel("坐标系"))
        self.frame_checks: list[QCheckBox] = []
        for idx in range(table.n + 1):
            cb = QCheckBox("基座" if idx == 0 else f"J{idx - 1}")
            cb.setChecked(True)
            cb.toggled.connect(lambda on, i=idx: self.view.set_frame_visible(i, on))
            self.frame_checks.append(cb)
            fr.addWidget(cb)
        fr.addStretch(1)
        dg.addLayout(fr, 2, 0, 1, 3)
        self.frame_len = QDoubleSpinBox()
        self.frame_len.setRange(5, 200)
        self.frame_len.setValue(self.view.frame_len * 1000)
        self.frame_len.setSuffix(" mm")
        self.frame_len.valueChanged.connect(lambda v: self.view.set_option("frame_len", v / 1000))
        row = QHBoxLayout()
        row.addWidget(QLabel("坐标轴长度"))
        row.addWidget(self.frame_len)
        row.addStretch(1)
        dg.addLayout(row, 3, 0, 1, 3)

        # ---------------- 数据来源 + 选中关节
        srcbox = QGroupBox("姿态来源")
        sl = QVBoxLayout(srcbox)
        self.src_live = QRadioButton("实时数据（当前连接的数据源）")
        self.src_manual = QRadioButton("手动滑块（离线核对 DH）")
        self.src_live.setChecked(True)
        grp = QButtonGroup(self)
        grp.addButton(self.src_live)
        grp.addButton(self.src_manual)
        self.src_manual.toggled.connect(self._on_src)
        sl.addWidget(self.src_live)
        sl.addWidget(self.src_manual)
        self.sliders: list[QSlider] = []
        self.slider_lbl: list[QLabel] = []
        sg = QGridLayout()
        for i in range(table.n):
            s = QSlider(Qt.Horizontal)
            s.setRange(-1800, 1800)       # 0.1° 分辨率
            s.valueChanged.connect(self._on_slider)
            lb = QLabel("  +0.0°")
            lb.setObjectName("value")
            lb.setMinimumWidth(56)
            sg.addWidget(QLabel(f"J{i}"), i, 0)
            sg.addWidget(s, i, 1)
            sg.addWidget(lb, i, 2)
            self.sliders.append(s)
            self.slider_lbl.append(lb)
        zero = QPushButton("全部归零")
        zero.clicked.connect(lambda: [s.setValue(0) for s in self.sliders])
        copy = QPushButton("取当前实时姿态")
        copy.clicked.connect(self._copy_live)
        sb = QHBoxLayout()
        sb.addWidget(zero)
        sb.addWidget(copy)
        sl.addLayout(sg)
        sl.addLayout(sb)
        selrow = QHBoxLayout()
        selrow.addWidget(QLabel("选中关节"))
        self.sel = QComboBox()
        self.sel.addItem("无", None)
        for i in range(table.n):
            self.sel.addItem(f"J{i}", i)
        self.sel.currentIndexChanged.connect(lambda: self._select_joint(self.sel.currentData()))
        selrow.addWidget(self.sel)
        selrow.addStretch(1)
        sl.addLayout(selrow)
        self._on_src()

        # ---------------- DH 编辑
        self.dh = DHEditor(table, read_device_dh)
        self.dh.applied.connect(self.view.set_table)

        ctrl = QWidget()
        cl = QVBoxLayout(ctrl)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.addLayout(views)
        cl.addWidget(disp)
        cl.addWidget(srcbox)
        cl.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(ctrl)
        tabs = QTabWidget()
        tabs.addTab(scroll, "显示 / 姿态")
        tabs.addTab(self.dh, "DH 参数")
        self.side_tabs = tabs
        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.view)
        split.addWidget(tabs)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(split)

        self.timer = QTimer(self, interval=max(10, int(1000 / fps)), timeout=self.tick)      # ≤30 Hz，与波形刷新独立
        self.timer.start()

    # ================================================================
    def _on_src(self) -> None:
        manual = self.src_manual.isChecked()
        for s in self.sliders:
            s.setEnabled(manual)
        if manual:
            self._on_slider()

    def _on_slider(self) -> None:
        q = [s.value() / 10 for s in self.sliders]
        for lb, v in zip(self.slider_lbl, q):
            lb.setText(f"{v:+6.1f}°")
        if self.src_manual.isChecked():
            self.view.set_angles(q)

    def _copy_live(self) -> None:
        s = self._get_sample()
        if s is None:
            return
        for sl, j in zip(self.sliders, s.joints):
            a = (j.angle_deg + 180.0) % 360.0 - 180.0         # 多圈角折到 ±180 显示在滑块上
            sl.setValue(int(round(a * 10)))
        self.src_manual.setChecked(True)

    def _select_joint(self, j: int | None) -> None:
        i = self.sel.findData(j)
        if i != self.sel.currentIndex():
            self.sel.blockSignals(True)
            self.sel.setCurrentIndex(i)
            self.sel.blockSignals(False)
        self.view.set_selected(j, self._joint_info(j))

    def _joint_info(self, j: int | None) -> list[str]:
        s = self._get_sample()
        if j is None or s is None or j >= len(s.joints) or self.src_manual.isChecked():
            return []
        r = s.joints[j]
        try:
            st = MotorStatus(r.motor_status).name
        except ValueError:
            st = str(r.motor_status)
        return [f"  目标   {r.target_deg:+9.3f}°",
                f"  角速度 {r.velocity_dps:+9.2f}°/s",
                f"  力矩   {r.torque_nm:+9.3f} N·m（指令 {r.torque_cmd_nm:+.3f}）",
                f"  状态   {st}  错误 {r.error_id}  {r.temperature_c}℃"]

    def tick(self) -> None:
        if not self.isVisible():
            return
        if self.src_live.isChecked():
            s = self._get_sample()
            if s is not None:
                self.view.set_angles([j.angle_deg for j in s.joints])
        if self.view.selected is not None:
            self.view.set_selected(self.view.selected, self._joint_info(self.view.selected))
