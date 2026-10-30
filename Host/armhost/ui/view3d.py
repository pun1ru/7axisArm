"""三维姿态显示：QPainter 透视投影的线框渲染器（连杆 = 长杆，关节 = 坐标系）。

选型见 docs/05_view3d.md：场景只有几十条线段，用 QPainter 自己投影比引入 OpenGL 依赖更稳：
零额外依赖、远程桌面 / 虚拟机 / 无显卡驱动都能用、offscreen 可截图测试、中文标签直接画。

坐标约定：世界系 = DH 基座系 O_0（与 Simulation/mujoco/robot.xml 世界系平行），Z 朝上，单位 m。
交互：左键拖动旋转，右键 / 中键拖动平移，滚轮缩放，双击复位；视角按钮：等轴 / 前 / 侧 / 俯。
渲染节流：只有姿态、视角或开关变化才重绘，并由定时器限到 ≤ max_fps。
"""

from __future__ import annotations

import math
import time

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

from ..kinematics import DHTable, euler_zyx_deg, forward_frames
from . import theme

AXIS_COLORS = (QColor("#ef5350"), QColor("#66bb6a"), QColor("#42a5f5"))   # x y z
VIEWS = {"iso": (-45.0, 25.0), "front": (-90.0, 0.0), "side": (0.0, 0.0), "top": (-90.0, 89.0)}


class View3D(QWidget):
    joint_clicked = Signal(int)

    def __init__(self, table: DHTable | None = None, parent=None, max_fps: int = 30) -> None:
        super().__init__(parent)
        self.setMinimumSize(320, 260)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.table = table or DHTable()
        self.q_deg = np.zeros(self.table.n)
        self.frames: list[np.ndarray] = forward_frames(self.table, self.q_deg)
        # 开关
        self.show_links = True
        self.show_frames = True                   # 关节坐标系总开关
        self.frame_mask = [True] * (self.table.n + 1)   # 每个坐标系单独开关：0 = 基座，i+1 = 关节 i
        self.show_axes_line = True                # 关节转轴（沿 z_i 的短线）
        self.show_labels = True
        self.show_grid = True
        self.show_world = True
        self.selected: int | None = None
        self.selected_info: list[str] = []
        self.frame_len = 0.04
        # 相机（轨道相机，围绕 target）
        self.reset_view()
        self._drag_btn = None
        self._drag_pos = QPointF()
        self._dirty = True
        self.renders = 0
        self.last_render_ms = 0.0
        self._timer = QTimer(self, interval=max(1, int(1000 / max_fps)), timeout=self._maybe_update)
        self._timer.start()

    # ================================================================ 外部接口
    def set_table(self, table: DHTable) -> None:
        self.table = table
        if len(self.q_deg) != table.n:
            self.q_deg = np.resize(self.q_deg, table.n)
        if len(self.frame_mask) != table.n + 1:
            self.frame_mask = (self.frame_mask + [True] * (table.n + 1))[: table.n + 1]
        self._recompute()

    def set_angles(self, q_deg) -> None:
        q = np.asarray(q_deg, dtype=float)[: self.table.n]
        if q.shape == self.q_deg.shape and np.array_equal(q, self.q_deg):
            return
        self.q_deg = np.resize(q, self.table.n)
        self._recompute()

    def set_option(self, name: str, value) -> None:
        setattr(self, name, value)
        self._dirty = True

    def set_frame_visible(self, idx: int, on: bool) -> None:
        if 0 <= idx < len(self.frame_mask):
            self.frame_mask[idx] = on
            self._dirty = True

    def set_selected(self, joint: int | None, info: list[str] | None = None) -> None:
        self.selected = joint
        self.selected_info = info or []
        self._dirty = True

    def end_pose(self) -> tuple[np.ndarray, tuple[float, float, float]]:
        T = self.frames[-1]
        return T[:3, 3].copy(), euler_zyx_deg(T[:3, :3])

    def reset_view(self, name: str = "iso") -> None:
        self.yaw, self.pitch = VIEWS.get(name, VIEWS["iso"])
        reach = sum(abs(r.a_m) + abs(r.d_m) for r in self.table.rows) or 0.3
        self.dist = max(0.25, reach * 2.6)
        self.target = np.array([0.0, 0.0, reach * 0.45])
        self._dirty = True

    def _recompute(self) -> None:
        self.frames = forward_frames(self.table, self.q_deg)
        self._dirty = True

    def _maybe_update(self) -> None:
        if self._dirty and self.isVisible():
            self.update()

    # ================================================================ 相机
    def _basis(self):
        cy, sy = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        cp, sp = math.cos(math.radians(self.pitch)), math.sin(math.radians(self.pitch))
        back = np.array([cp * cy, cp * sy, sp])          # 从 target 指向相机
        eye = self.target + self.dist * back
        fwd = -back
        right = np.cross(fwd, (0.0, 0.0, 1.0))
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        return eye, fwd, right, up

    def _projector(self):
        eye, fwd, right, up = self._basis()
        w, h = self.width(), self.height()
        f = 0.5 * h / math.tan(math.radians(40.0) / 2)
        cx, cy = w / 2, h / 2
        M = np.stack([right, up, fwd])

        def project(P: np.ndarray):
            """P: (n,3) -> (n,2) 屏幕坐标, (n,) 深度；深度 <= near 的点返回 nan"""
            c = (np.atleast_2d(P) - eye) @ M.T
            z = c[:, 2]
            zz = np.where(z > 1e-3, z, np.nan)
            return np.stack([cx + f * c[:, 0] / zz, cy - f * c[:, 1] / zz], axis=1), z

        return project

    # ================================================================ 绘制
    def paintEvent(self, _ev) -> None:
        t0 = time.perf_counter()
        self._dirty = False
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.PLOT_BG))
        project = self._projector()
        segs: list[tuple[float, np.ndarray, QPen]] = []   # (深度, 两端点, 笔)

        def add(a, b, pen):
            segs.append((None, np.array([a, b], float), pen))

        # 地面网格
        if self.show_grid:
            g = QPen(QColor(255, 255, 255, 16), 1)
            g0 = QPen(QColor(255, 255, 255, 40), 1)
            step, n = 0.05, 5                         # ±0.25 m，与臂展同量级，不抢视线
            for k in range(-n, n + 1):
                pen = g0 if k == 0 else g
                add((k * step, -n * step, 0), (k * step, n * step, 0), pen)
                add((-n * step, k * step, 0), (n * step, k * step, 0), pen)
        # 世界坐标轴
        if self.show_world:
            for a in range(3):
                e = np.zeros(3)
                e[a] = 0.08
                add((0, 0, 0), e, QPen(AXIS_COLORS[a], 2))
        F = self.frames
        origins = np.array([T[:3, 3] for T in F])
        # 连杆：相邻且不重合的原点之间画长杆
        if self.show_links:
            pen = QPen(QColor("#b0b8c4"), 7, Qt.SolidLine, Qt.RoundCap)
            for i in range(len(origins) - 1):
                if np.linalg.norm(origins[i + 1] - origins[i]) > 1e-6:
                    add(origins[i], origins[i + 1], pen)
        # 关节转轴：沿 z_{i+1} 的短线，表示关节 i 绕哪根轴转
        if self.show_axes_line:
            for i in range(self.table.n):
                T = F[i + 1]
                c = QColor(theme.JOINT_COLORS[i % 7])
                c.setAlpha(200)
                z = T[:3, 2] * 0.025
                add(T[:3, 3] - z, T[:3, 3] + z, QPen(c, 4, Qt.SolidLine, Qt.RoundCap))
        # 坐标系
        if self.show_frames:
            for idx, T in enumerate(F):
                if not self.frame_mask[idx]:
                    continue
                L = self.frame_len * (1.6 if idx - 1 == self.selected else 1.0)
                w = 3 if idx - 1 == self.selected else 2
                for a in range(3):
                    add(T[:3, 3], T[:3, 3] + T[:3, a] * L, QPen(AXIS_COLORS[a], w))

        # 画家算法：按线段中点深度从远到近
        mids = np.array([s[1].mean(axis=0) for s in segs])
        if len(mids):
            _, depth = project(mids)
            order = np.argsort(-depth)
            ends = np.concatenate([s[1] for s in segs])
            scr, ez = project(ends)
            for k in order:
                a, b = scr[2 * k], scr[2 * k + 1]
                if np.isnan(a).any() or np.isnan(b).any():
                    continue
                p.setPen(segs[k][2])
                p.drawLine(QPointF(*a), QPointF(*b))

        # 关节球 + 标签（最后画，保证在最上面）
        scr, z = project(origins)
        font = QFont(self.font())
        font.setPointSizeF(8.5)
        p.setFont(font)
        stacked: dict[tuple[int, int], int] = {}
        for i in range(self.table.n):
            P = scr[i + 1]
            if np.isnan(P).any():
                continue
            c = QColor(theme.JOINT_COLORS[i % 7])
            sel = i == self.selected
            r = 7 if sel else 4.5
            p.setPen(QPen(QColor("white") if sel else c.darker(150), 2 if sel else 1))
            p.setBrush(c)
            p.drawEllipse(QPointF(*P), r, r)
            if self.show_labels:
                key = (int(P[0] // 6), int(P[1] // 6))          # 共点关节的标签往下错开
                k = stacked.get(key, 0)
                stacked[key] = k + 1
                p.setPen(c.lighter(120) if not sel else QColor("white"))
                p.drawText(QPointF(P[0] + 10, P[1] - 6 + 13 * k), f"J{i}")
        if self.show_labels and not np.isnan(scr[0]).any():
            p.setPen(QColor(theme.MUTED))
            p.drawText(QPointF(scr[0][0] + 8, scr[0][1] + 14), "基座 O₀")
        self._draw_hud(p)
        p.end()
        self.renders += 1
        self.last_render_ms = (time.perf_counter() - t0) * 1e3

    def _draw_hud(self, p: QPainter) -> None:
        pos, (r, pi, y) = self.end_pose()
        lines = [f"末端 x {pos[0] * 1e3:+8.1f}  y {pos[1] * 1e3:+8.1f}  z {pos[2] * 1e3:+8.1f} mm",
                 f"     roll {r:+7.1f}  pitch {pi:+7.1f}  yaw {y:+7.1f} °",
                 f"DH：{self.table.source}"]
        if self.selected is not None and self.selected < self.table.n:
            lines.append("")
            lines.append(f"▶ J{self.selected}  q = {self.q_deg[self.selected]:+.3f}°")
            lines.extend(self.selected_info)
        font = QFont("Consolas")
        font.setStyleHint(QFont.Monospace)
        font.setPointSizeF(8.5)
        p.setFont(font)
        fm = p.fontMetrics()
        wbox = max(fm.horizontalAdvance(s) for s in lines) + 16
        hbox = fm.height() * len(lines) + 10
        p.setPen(QPen(QColor(theme.BORDER)))
        p.setBrush(QColor(21, 23, 27, 215))
        p.drawRect(QRectF(8, 8, wbox, hbox))
        p.setPen(QColor(theme.TEXT))
        for k, s in enumerate(lines):
            p.drawText(QPointF(16, 8 + 5 + fm.ascent() + k * fm.height()), s)
        p.setPen(QColor(theme.MUTED))
        hint = "左键旋转  右键/中键平移  滚轮缩放  双击复位  点关节选中"
        p.drawText(QPointF(10, self.height() - 8), hint)

    # ================================================================ 交互
    def mousePressEvent(self, ev) -> None:
        self._drag_btn = ev.button()
        self._drag_pos = ev.position()
        self._press_pos = ev.position()

    def mouseReleaseEvent(self, ev) -> None:
        if ev.button() == Qt.LeftButton and (ev.position() - self._press_pos).manhattanLength() < 4:
            self._pick(ev.position())
        self._drag_btn = None

    def mouseMoveEvent(self, ev) -> None:
        if self._drag_btn is None:
            return
        d = ev.position() - self._drag_pos
        self._drag_pos = ev.position()
        if self._drag_btn == Qt.LeftButton:
            self.yaw -= d.x() * 0.4
            self.pitch = max(-89.0, min(89.0, self.pitch + d.y() * 0.4))
        elif self._drag_btn in (Qt.RightButton, Qt.MiddleButton):
            _, _, right, up = self._basis()
            k = self.dist * 0.0015
            self.target = self.target - right * d.x() * k + up * d.y() * k
        self._dirty = True

    def wheelEvent(self, ev) -> None:
        self.dist = max(0.05, min(20.0, self.dist * (0.88 ** (ev.angleDelta().y() / 120))))
        self._dirty = True

    def mouseDoubleClickEvent(self, _ev) -> None:
        self.reset_view()

    def _pick(self, pos: QPointF) -> None:
        project = self._projector()
        scr, _ = project(np.array([T[:3, 3] for T in self.frames[1:]]))
        d = np.hypot(scr[:, 0] - pos.x(), scr[:, 1] - pos.y())
        d = np.where(np.isnan(d), np.inf, d)
        i = int(np.argmin(d)) if len(d) else -1
        if i >= 0 and d[i] < 14:
            self.joint_clicked.emit(i)
