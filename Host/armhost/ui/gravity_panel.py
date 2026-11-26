"""底部「重力拟合」标签页：选 CSV -> 按当前 DH 拟合 tau = Y(q)·pi -> 看报告 / 导出 JSON。

算法在 model/gravity_fit.py（命令行 tools/gravity_fit.py 用同一份）。DH 取三维姿态页当前生效的那份。
"""

from __future__ import annotations

import json
import re
import time
from pathlib import Path
from typing import Callable

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QGuiApplication
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QHBoxLayout, QLabel,
                               QLineEdit, QListWidget, QListWidgetItem, QPlainTextEdit, QPushButton,
                               QVBoxLayout, QWidget)

from ..kinematics import DHTable
from ..model import gravity_fit as gf

LOG_DIR = Path(__file__).resolve().parents[2] / "logs"
GRAVITY_DIR = LOG_DIR / "gravity"
NOISE_TIMER_MS = 20
NOISE_MIN_SAMPLES = 10
NOISE_STATIC_VEL_DPS = 2.0


class GravityFitPanel(QWidget):
    def __init__(self, get_table: Callable[[], DHTable],
                 get_sample: Callable[[], object | None] | None = None, parent=None) -> None:
        super().__init__(parent)
        self._get_table = get_table
        self._get_sample = get_sample
        self.result: gf.FitResult | None = None
        self.noise_sigma: np.ndarray | None = None
        self._noise_groups: list[np.ndarray] = []
        self._noise_frames: list[np.ndarray] = []
        self._noise_start_ns = 0

        # ---- 文件列表
        self.files = QListWidget()
        self.files.setToolTip("勾选的文件参与拟合。默认扫描 Host/logs/gravity/，也可以添加别处的 CSV")
        self.refresh_btn = QPushButton("扫描 logs/gravity")
        self.refresh_btn.setToolTip("把目录里新增的 CSV 加进列表（包括正在采集的那个）")
        self.refresh_btn.clicked.connect(self.scan)
        self.add_btn = QPushButton("添加…")
        self.add_btn.clicked.connect(self._add_files)
        self.clear_btn = QPushButton("清空")
        self.clear_btn.clicked.connect(self.files.clear)
        fb = QHBoxLayout()
        for b in (self.refresh_btn, self.add_btn, self.clear_btn):
            fb.addWidget(b)
        left = QVBoxLayout()
        left.addWidget(self.files, 1)
        left.addLayout(fb)

        # ---- 选项
        self.torque = QComboBox()
        self.torque.addItem("实测力矩 torque_nm", "torque")
        self.torque.addItem("指令力矩 torque_cmd_nm", "cmd")
        self.sign = QComboBox()
        self.sign.addItem("+1（CSV 力矩 = 重力矩）", 1.0)
        self.sign.addItem("-1（CSV 力矩与重力矩反号）", -1.0)
        self.sign.setToolTip("符号无法从拟合判断：只会让 pi 整体反号。算出质量为负时改这里")
        self.rcond = QComboBox()
        for v in (1e-2, 1e-3, 1e-4, 1e-6):
            self.rcond.addItem(f"{v:g}", v)
        self.rcond.setCurrentIndex(1)
        self.rcond.setToolTip("截断 SVD 的相对阈值：越大越保守（只保留强激励方向）")
        self.bias = QCheckBox("拟合力矩零偏")
        self.bias.setToolTip("数据少时容易过拟合（交叉验证会变差），默认关")
        self.vel_on = QCheckBox("丢弃运动中的行，|速度| >")
        self.vel_on.setToolTip("默认按全部静止处理；速度列有孤立尖峰时开启会误删有效点")
        self.vel = QDoubleSpinBox()
        self.vel.setRange(0.1, 1000)
        self.vel.setValue(20)
        self.vel.setSuffix(" dps")
        self.vel.setEnabled(False)
        self.vel_on.toggled.connect(self.vel.setEnabled)
        self.states = QLineEdit()
        self.states.setPlaceholderText("空=全部，如 6 或 5,6")
        self.states.setToolTip("只用这些 arm_state（模式）的行")
        self.cv = QCheckBox("留一文件交叉验证")
        self.cv.setChecked(True)

        # ---- 静止噪声采集
        self.noise_duration = QDoubleSpinBox()
        self.noise_duration.setRange(0.2, 60.0)
        self.noise_duration.setDecimals(1)
        self.noise_duration.setValue(2.0)
        self.noise_duration.setSuffix(" s")
        self.noise_btn = QPushButton("采集一组噪声")
        self.noise_btn.setToolTip("机械臂保持静止，连续采集一段时间的力矩标准差；可重复采集多组")
        self.noise_btn.setEnabled(get_sample is not None)
        self.noise_clear_btn = QPushButton("清除")
        self.noise_weighted = QCheckBox("按关节噪声加权")
        self.noise_weighted.setEnabled(False)
        self.noise_weighted.setToolTip("用采集到的每轴 torque_nm 标准差归一化方程；多组取均值")
        self.noise_info = QLabel("尚未采集")
        self.noise_info.setObjectName("value")
        self.noise_info.setWordWrap(True)
        self.noise_timer = QTimer(self)
        self.noise_timer.setInterval(NOISE_TIMER_MS)
        self.noise_timer.timeout.connect(self._collect_noise_sample)
        self.noise_btn.clicked.connect(self.start_noise_capture)
        self.noise_clear_btn.clicked.connect(self.clear_noise)

        def row(label: str, w: QWidget) -> QHBoxLayout:
            r = QHBoxLayout()
            r.addWidget(QLabel(label))
            r.addWidget(w, 1)
            return r
        mid = QVBoxLayout()
        mid.addLayout(row("拟合对象", self.torque))
        mid.addLayout(row("符号", self.sign))
        mid.addLayout(row("rcond", self.rcond))
        mid.addLayout(row("模式", self.states))
        vr = QHBoxLayout()
        vr.addWidget(self.vel_on)
        vr.addWidget(self.vel)
        mid.addLayout(vr)
        cr = QHBoxLayout()
        cr.addWidget(self.bias)
        cr.addWidget(self.cv)
        mid.addLayout(cr)
        nr = QHBoxLayout()
        nr.addWidget(self.noise_btn)
        nr.addWidget(self.noise_duration)
        nr.addWidget(self.noise_clear_btn)
        mid.addLayout(nr)
        mid.addWidget(self.noise_weighted)
        mid.addWidget(self.noise_info)
        mid.addStretch(1)

        # ---- 动作 + 报告
        self.run_btn = QPushButton("拟合")
        self.run_btn.clicked.connect(self.run)
        self.save_btn = QPushButton("导出 JSON…")
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self._save)
        self.copy_btn = QPushButton("复制报告")
        self.copy_btn.setEnabled(False)
        self.copy_btn.clicked.connect(lambda: QGuiApplication.clipboard().setText(self.report.toPlainText()))
        self.info = QLabel("")
        self.info.setObjectName("value")
        ab = QHBoxLayout()
        for b in (self.run_btn, self.save_btn, self.copy_btn):
            ab.addWidget(b)
        ab.addWidget(self.info, 1)
        self.report = QPlainTextEdit()
        self.report.setReadOnly(True)
        self.report.setLineWrapMode(QPlainTextEdit.NoWrap)
        f = QFont("Consolas")
        f.setStyleHint(QFont.Monospace)
        self.report.setFont(f)
        self.report.setPlaceholderText("勾选 CSV 后点「拟合」。DH 取三维姿态页当前生效的那份。")
        right = QVBoxLayout()
        right.addLayout(ab)
        right.addWidget(self.report, 1)

        lay = QHBoxLayout(self)
        lay.addLayout(left, 2)
        lay.addLayout(mid, 2)
        lay.addLayout(right, 5)
        self.scan()

    # ------------------------------------------------------------ 文件
    def _paths(self, checked_only: bool = True) -> list[str]:
        out = []
        for i in range(self.files.count()):
            it = self.files.item(i)
            if not checked_only or it.checkState() == Qt.Checked:
                out.append(it.data(Qt.UserRole))
        return out

    def add_file(self, path: str | Path, checked: bool = True) -> bool:
        p = str(Path(path))
        if p in self._paths(False):
            return False
        it = QListWidgetItem(Path(p).name)
        it.setData(Qt.UserRole, p)
        it.setToolTip(p)
        it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
        it.setCheckState(Qt.Checked if checked else Qt.Unchecked)
        self.files.addItem(it)
        return True

    def scan(self) -> int:
        """把 logs/gravity 里还没在列表里的 CSV 加进来（新增的默认勾选）。"""
        n = 0
        if GRAVITY_DIR.is_dir():
            for p in sorted(GRAVITY_DIR.glob("*.csv")):
                n += self.add_file(p)
        return n

    def _add_files(self) -> None:
        ps, _ = QFileDialog.getOpenFileNames(self, "选择重力采集 CSV",
                                             str(GRAVITY_DIR if GRAVITY_DIR.is_dir() else LOG_DIR), "CSV (*.csv)")
        for p in ps:
            self.add_file(p)

    # ------------------------------------------------------------ 拟合
    def start_noise_capture(self) -> None:
        if self._get_sample is None:
            self.noise_info.setText("当前数据源不支持在线噪声采集")
            return
        self._noise_frames = []
        self._noise_start_ns = time.monotonic_ns()
        self.noise_btn.setEnabled(False)
        self.noise_info.setText("正在采集静止噪声，请保持机械臂静止…")
        self.noise_timer.start()

    def _collect_noise_sample(self) -> None:
        sample = self._get_sample() if self._get_sample is not None else None
        if sample is not None:
            joints = getattr(sample, "joints", ())
            if joints and all(abs(float(j.velocity_dps)) <= NOISE_STATIC_VEL_DPS for j in joints):
                self._noise_frames.append(np.array([float(j.torque_nm) for j in joints], dtype=float))
        elapsed_s = (time.monotonic_ns() - self._noise_start_ns) * 1e-9
        if elapsed_s >= self.noise_duration.value():
            self.noise_timer.stop()
            self.noise_btn.setEnabled(True)
            if len(self._noise_frames) < NOISE_MIN_SAMPLES:
                self.noise_info.setText(
                    f"有效静止样本不足（{len(self._noise_frames)}），请保持静止后重试")
                return
            self.apply_noise_group(np.vstack(self._noise_frames))

    def apply_noise_group(self, frames_nm) -> np.ndarray:
        sigma = gf.estimate_noise_group(frames_nm)
        self._noise_groups.append(sigma)
        self.noise_sigma = gf.average_noise_groups(self._noise_groups)
        self.noise_weighted.setEnabled(True)
        self.noise_weighted.setChecked(True)
        values = " ".join(f"J{j}={v:.3g}" for j, v in enumerate(self.noise_sigma))
        self.noise_info.setText(f"{len(self._noise_groups)} 组平均噪声 σ [N·m]：{values}")
        return self.noise_sigma

    def clear_noise(self) -> None:
        self.noise_timer.stop()
        self.noise_btn.setEnabled(self._get_sample is not None)
        self._noise_frames = []
        self._noise_groups = []
        self.noise_sigma = None
        self.noise_weighted.setChecked(False)
        self.noise_weighted.setEnabled(False)
        self.noise_info.setText("尚未采集")

    def options(self) -> gf.FitOptions:
        txt = self.states.text().strip()
        states = [int(s) for s in re.split(r"[,\s]+", txt) if s] if txt else None
        return gf.FitOptions(torque_col=self.torque.currentData(), sign=float(self.sign.currentData()),
                             arm_states=states, max_vel=self.vel.value() if self.vel_on.isChecked() else None,
                             bias=self.bias.isChecked(), rcond=float(self.rcond.currentData()), cv=self.cv.isChecked(),
                             noise_sigma=(self.noise_sigma.tolist()
                                          if self.noise_weighted.isChecked() and self.noise_sigma is not None else None))

    def run(self) -> gf.FitResult | None:
        self.result = None
        self.save_btn.setEnabled(False)
        self.copy_btn.setEnabled(False)
        try:
            paths = self._paths()
            if not paths:
                raise ValueError("没有勾选任何 CSV")
            if self.noise_weighted.isChecked() and self.noise_sigma is None:
                raise ValueError("已选择噪声加权，但还没有采集噪声")
            self.result = gf.run(paths, self._get_table(), self.options())
        except (OSError, ValueError, KeyError) as e:
            self.report.setPlainText(f"错误: {e}")
            self.info.setText("拟合失败")
            return None
        r = self.result
        self.report.setPlainText(gf.format_report(r))
        self.save_btn.setEnabled(True)
        self.copy_btn.setEnabled(True)
        rmse_text = (f"加权 RMSE {r.weighted_rmse:.4f}σ" if r.weighted_rmse is not None
                     else f"RMSE {r.rmse:.4f} N·m")
        self.info.setText(f"{r.n_poses} 个位姿，秩 {r.rank}/{r.n_unknowns}，{rmse_text}")
        return r

    def _save(self) -> None:
        if self.result is None:
            return
        GRAVITY_DIR.mkdir(parents=True, exist_ok=True)
        p, _ = QFileDialog.getSaveFileName(self, "导出拟合结果", str(GRAVITY_DIR / "gravity_fit.json"), "JSON (*.json)")
        if p:
            Path(p).write_text(json.dumps(self.result.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
            self.info.setText(f"已导出 {Path(p).name}")
