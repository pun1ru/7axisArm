"""三维姿态显示：QPainter 透视投影的线框渲染器（连杆 = 长杆，关节 = 坐标系）。

选型见 docs/05_view3d.md：场景只有几十条线段，用 QPainter 自己投影比引入 OpenGL 依赖更稳：
零额外依赖、远程桌面 / 虚拟机 / 无显卡驱动都能用、offscreen 可截图测试、中文标签直接画。

坐标约定：世界系来自 Simulation/mujoco/BigArm/robot.xml，Z 朝上，单位 m；DH 基座 O_0 是腰部
连杆坐标系，显示时可用 world_mount 叠加 main.cpp 里 joint1 的安装 xyz/rpy。
交互：左键拖动旋转，右键 / 中键拖动平移，滚轮缩放，双击复位；视角按钮：等轴 / 前 / 侧 / 俯。
渲染节流：只有姿态、视角或开关变化才重绘，并由定时器限到 ≤ max_fps。
"""

from __future__ import annotations

import math
import time

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from ..kinematics import DHTable, euler_zyx_deg, forward_frames, transform_from_xyz_rpy_deg
from . import theme

AXIS_COLORS = (QColor("#ef5350"), QColor("#66bb6a"), QColor("#42a5f5"))   # x y z
VIEWS = {"iso": (-45.0, 25.0), "front": (-90.0, 0.0), "side": (0.0, 0.0), "top": (-90.0, 89.0)}
# 可达区使用纯蓝渐变；灵活区单独用橙色叠加，避免和密度色混淆。
WORKSPACE_COLORS = (QColor(44, 86, 130, 105), QColor(42, 111, 170, 110), QColor(45, 134, 190, 115),
                    QColor(51, 158, 205, 120), QColor(72, 184, 218, 128), QColor(110, 210, 230, 140))


class View3D(QWidget):
    joint_clicked = Signal(int)

    def __init__(self, table: DHTable | None = None, parent=None, max_fps: int = 30) -> None:
        super().__init__(parent)
        self.setMinimumSize(320, 260)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.table = table or DHTable()
        self.q_deg = np.zeros(self.table.n)
        self.target_q_deg: np.ndarray | None = None
        self.target_pose_base: np.ndarray | None = None
        self.target_frames: list[np.ndarray] = []
        self.target_tcp_frame: np.ndarray | None = None
        self.target_valid = False
        self.frames: list[np.ndarray] = forward_frames(self.table, self.q_deg)
        self.tcp_frame: np.ndarray = self.frames[-1] @ self.table.tool_transform
        # 开关
        self.show_links = True
        self.show_frames = True                   # 关节坐标系总开关
        self.frame_mask = [True] * (self.table.n + 1)   # 每个坐标系单独开关：0 = 基座，i+1 = 关节 i
        self.show_axes_line = True                # 关节转轴（沿 z_i 的短线）
        self.show_labels = True
        self.show_tcp = True
        self.show_target = False
        self.show_grid = True
        self.show_world = True
        self.world_mount = True                    # 显示时叠加 URDF joint1 的腰部安装变换
        self.show_workspace = False
        self.workspace_points = np.empty((0, 3))
        self.workspace_density = np.empty(0, dtype=float)
        self.workspace_flexible_points = np.empty((0, 3))
        self.workspace_flexible_density = np.empty(0, dtype=float)
        self.show_workspace_reachable = True
        self.show_workspace_flexible = True
        self.workspace_info = ""
        self.selected: int | None = None
        self.selected_info: list[str] = []
        self.frame_len = 0.04
        # 相机（轨道相机，围绕 target）
        self._recompute()
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
        if self.target_q_deg is not None and len(self.target_q_deg) != table.n:
            self.target_q_deg = np.resize(self.target_q_deg, table.n)
        if len(self.frame_mask) != table.n + 1:
            self.frame_mask = (self.frame_mask + [True] * (table.n + 1))[: table.n + 1]
        self._recompute()

    def set_angles(self, q_deg) -> None:
        q = np.asarray(q_deg, dtype=float)[: self.table.n]
        if q.shape == self.q_deg.shape and np.array_equal(q, self.q_deg):
            return
        self.q_deg = np.resize(q, self.table.n)
        self._recompute()

    def set_target_angles(self, q_deg) -> None:
        """设置目标关节角；None 表示当前数据源没有可显示的目标构型。"""
        if q_deg is None:
            if self.target_q_deg is None:
                return
            self.target_q_deg = None
            self.target_pose_base = None
            self.target_frames = []
            self.target_tcp_frame = None
            self.target_valid = False
            self._dirty = True
            return
        q = np.asarray(q_deg, dtype=float)[: self.table.n]
        if self.target_q_deg is not None and q.shape == self.target_q_deg.shape \
                and np.array_equal(q, self.target_q_deg):
            return
        self.target_q_deg = np.resize(q, self.table.n)
        self.target_pose_base = None
        self._recompute()

    def set_target_pose_deg(self, x: float, y: float, z: float,
                            roll: float, pitch: float, yaw: float) -> None:
        """直接设置腰部连杆坐标系中的原始 IK 末端 TCP 目标位姿。"""
        self.target_q_deg = None
        self.target_pose_base = transform_from_xyz_rpy_deg(x, y, z, roll, pitch, yaw)
        self._recompute()

    def target_pose(self) -> tuple[np.ndarray, tuple[float, float, float]] | None:
        if not self.target_valid or self.target_tcp_frame is None:
            return None
        T = self.target_tcp_frame
        return T[:3, 3].copy(), euler_zyx_deg(T[:3, :3])

    def set_option(self, name: str, value) -> None:
        if name == "world_mount":
            self.world_mount = bool(value)
            self._recompute()
            return
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

    def set_workspace(self, points_m, density=None, info: str = "",
                      flexible_points_m=None, flexible_density=None) -> None:
        pts = np.asarray(points_m, dtype=float).reshape(-1, 3)
        if density is None:
            den = np.ones(len(pts), dtype=float)
        else:
            den = np.asarray(density, dtype=float).reshape(-1)
            if len(den) != len(pts):
                raise ValueError("工作空间点数和密度长度不一致")
        self.workspace_points = pts
        self.workspace_density = np.clip(den, 0.0, 1.0)
        fpts = np.empty((0, 3)) if flexible_points_m is None else \
            np.asarray(flexible_points_m, dtype=float).reshape(-1, 3)
        fden = np.ones(len(fpts), dtype=float) if flexible_density is None else \
            np.asarray(flexible_density, dtype=float).reshape(-1)
        if len(fden) != len(fpts):
            raise ValueError("灵活工作区点数和密度长度不一致")
        self.workspace_flexible_points = fpts
        self.workspace_flexible_density = np.clip(fden, 0.0, 1.0)
        self.workspace_info = info
        self._dirty = True

    def clear_workspace(self) -> None:
        self.workspace_points = np.empty((0, 3))
        self.workspace_density = np.empty(0, dtype=float)
        self.workspace_flexible_points = np.empty((0, 3))
        self.workspace_flexible_density = np.empty(0, dtype=float)
        self.workspace_info = ""
        self._dirty = True

    def fit_workspace(self) -> None:
        points = self._visible_workspace_points()
        if len(points) == 0:
            return
        points = self._display_points(points)
        lo = points.min(axis=0)
        hi = points.max(axis=0)
        self.target = 0.5 * (lo + hi)
        radius = 0.5 * float(np.linalg.norm(hi - lo))
        self.dist = max(0.15, radius / math.tan(math.radians(20.0)) * 1.15)
        self._dirty = True

    def end_pose(self) -> tuple[np.ndarray, tuple[float, float, float]]:
        T = self.tcp_frame
        return T[:3, 3].copy(), euler_zyx_deg(T[:3, :3])

    def reset_view(self, name: str = "iso") -> None:
        self.yaw, self.pitch = VIEWS.get(name, VIEWS["iso"])
        reach = sum(abs(r.a_m) + abs(r.d_m) for r in self.table.rows) or 0.3
        self.dist = max(0.25, reach * 2.6)
        origins = [T[:3, 3] for T in self.frames] + [self.tcp_frame[:3, 3]]
        if self.show_target and self.target_valid:
            origins += [T[:3, 3] for T in self.target_frames] + [self.target_tcp_frame[:3, 3]]
        origins = np.array(origins)
        self.target = 0.5 * (origins.min(axis=0) + origins.max(axis=0))
        self._dirty = True

    def _recompute(self) -> None:
        base = self.table.base_transform if self.world_mount else np.eye(4)
        frames = [base @ T for T in forward_frames(self.table, self.q_deg)]
        tcp = frames[-1] @ self.table.tool_transform
        target_frames = []
        target_tcp = None
        if self.target_pose_base is not None:
            target_tcp = base @ self.target_pose_base
        elif self.target_q_deg is not None:
            target_frames = [base @ T for T in forward_frames(self.table, self.target_q_deg)]
            target_tcp = target_frames[-1] @ self.table.tool_transform
        self.frames = frames
        self.tcp_frame = tcp
        self.target_frames = target_frames
        self.target_tcp_frame = target_tcp
        self.target_valid = target_tcp is not None
        self._dirty = True

    def _visible_workspace_points(self) -> np.ndarray:
        parts = []
        if self.show_workspace_reachable and len(self.workspace_points):
            parts.append(self.workspace_points)
        if self.show_workspace_flexible and len(self.workspace_flexible_points):
            parts.append(self.workspace_flexible_points)
        return np.concatenate(parts) if parts else np.empty((0, 3))

    def _display_points(self, points: np.ndarray) -> np.ndarray:
        if not self.world_mount or len(points) == 0:
            return points
        base = self.table.base_transform
        return points @ base[:3, :3].T + base[:3, 3]

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
        if self.show_workspace and len(self._visible_workspace_points()):
            self._draw_workspace(p, project)
        F = self.frames
        if self.world_mount and np.linalg.norm(F[0][:3, 3]) > 1e-6:
            pen = QPen(QColor(255, 255, 255, 55), 2, Qt.DashLine)
            add((0, 0, 0), F[0][:3, 3], pen)
        if self.show_target and self.target_valid:
            Ft = self.target_frames
            target_pen = QPen(QColor(96, 165, 250, 105), 6, Qt.DashLine, Qt.RoundCap)
            target_tcp_pen = QPen(QColor(96, 165, 250, 135), 5, Qt.SolidLine, Qt.RoundCap)
            target_origins = np.array([T[:3, 3] for T in Ft])
            if self.show_links and len(target_origins):
                for i in range(len(target_origins) - 1):
                    if np.linalg.norm(target_origins[i + 1] - target_origins[i]) > 1e-6:
                        add(target_origins[i], target_origins[i + 1], target_pen)
                if np.linalg.norm(self.target_tcp_frame[:3, 3] - target_origins[-1]) > 1e-6:
                    add(target_origins[-1], self.target_tcp_frame[:3, 3], target_tcp_pen)
            if self.show_frames:
                T = self.target_tcp_frame
                for a in range(3):
                    color = QColor(AXIS_COLORS[a])
                    color.setAlpha(85)
                    add(T[:3, 3], T[:3, 3] + T[:3, a] * self.frame_len * 1.4,
                        QPen(color, 2, Qt.DashLine))
        origins = np.array([T[:3, 3] for T in F])
        # 连杆：相邻且不重合的原点之间画长杆
        if self.show_links:
            pen = QPen(QColor("#b0b8c4"), 7, Qt.SolidLine, Qt.RoundCap)
            for i in range(len(origins) - 1):
                if np.linalg.norm(origins[i + 1] - origins[i]) > 1e-6:
                    add(origins[i], origins[i + 1], pen)
            tool_pen = QPen(QColor("#66bb6a"), 6, Qt.SolidLine, Qt.RoundCap)
            if np.linalg.norm(self.tcp_frame[:3, 3] - origins[-1]) > 1e-6:
                add(origins[-1], self.tcp_frame[:3, 3], tool_pen)
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
            if self.show_tcp:
                T = self.tcp_frame
                for a in range(3):
                    add(T[:3, 3], T[:3, 3] + T[:3, a] * self.frame_len * 1.4,
                        QPen(AXIS_COLORS[a], 3))

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
        if self.show_labels and self.show_world:
            world_scr, _ = project(np.array([[0.0, 0.0, 0.0]]))
            if not np.isnan(world_scr[0]).any():
                p.setPen(QColor(theme.MUTED))
                p.drawText(QPointF(world_scr[0][0] + 8, world_scr[0][1] - 4), "世界 O_w")
        if self.show_labels and not np.isnan(scr[0]).any():
            p.setPen(QColor(theme.MUTED))
            p.drawText(QPointF(scr[0][0] + 8, scr[0][1] + 14), "腰部 O₀" if self.world_mount else "DH 基座 O₀")
        tcp_scr, _ = project(np.array([self.tcp_frame[:3, 3]]))
        if self.show_labels and self.show_tcp and not np.isnan(tcp_scr[0]).any():
            p.setPen(QColor("#8ee89a"))
            p.setBrush(QColor("#2e7d32"))
            p.drawEllipse(QPointF(*tcp_scr[0]), 5.5, 5.5)
            p.drawText(QPointF(tcp_scr[0][0] + 9, tcp_scr[0][1] - 7), "TCP")
        if self.show_target and self.target_valid:
            target_scr, _ = project(np.array([self.target_tcp_frame[:3, 3]]))
            if self.show_labels and not np.isnan(target_scr[0]).any():
                p.setPen(QColor(147, 197, 253, 220))
                p.setBrush(QColor(59, 130, 246, 90))
                p.drawEllipse(QPointF(*target_scr[0]), 5.0, 5.0)
                p.drawText(QPointF(target_scr[0][0] + 9, target_scr[0][1] + 14), "目标 TCP")
        if self.show_workspace and len(self._visible_workspace_points()):
            self._draw_workspace_end(p, project)
        self._draw_hud(p)
        p.end()
        self.renders += 1
        self.last_render_ms = (time.perf_counter() - t0) * 1e3

    def _draw_workspace(self, p: QPainter, project) -> None:
        if self.show_workspace_reachable and len(self.workspace_points):
            scr, z = project(self._display_points(self.workspace_points))
            ok = np.isfinite(scr).all(axis=1) & (z > 1e-3)
            if ok.any():
                scr, den = scr[ok], self.workspace_density[ok]
                bins = np.minimum((den * len(WORKSPACE_COLORS)).astype(np.int64),
                                  len(WORKSPACE_COLORS) - 1)
                for k, color in enumerate(WORKSPACE_COLORS):
                    pts = scr[bins == k]
                    if len(pts):
                        p.setPen(QPen(color, 2.0))
                        p.drawPoints(QPolygonF([QPointF(float(x), float(y)) for x, y in pts]))
        if self.show_workspace_flexible and len(self.workspace_flexible_points):
            scr, z = project(self._display_points(self.workspace_flexible_points))
            ok = np.isfinite(scr).all(axis=1) & (z > 1e-3)
            pts = scr[ok]
            if len(pts):
                # 灵活区是可达区的子集，用橙色/暖色叠加突出。
                p.setPen(QPen(QColor(255, 183, 77, 220), 3.0))
                p.drawPoints(QPolygonF([QPointF(float(x), float(y)) for x, y in pts]))

    def _draw_workspace_end(self, p: QPainter, project) -> None:
        pos, _ = self.end_pose()
        scr, z = project(np.array([pos]))
        P = scr[0]
        if np.isnan(P).any() or z[0] <= 1e-3:
            return
        p.setPen(QPen(QColor("#ffffff"), 2))
        p.setBrush(QColor(theme.WARN))
        p.drawEllipse(QPointF(*P), 5.5, 5.5)
        p.drawLine(QPointF(P[0] - 10, P[1]), QPointF(P[0] + 10, P[1]))
        p.drawLine(QPointF(P[0], P[1] - 10), QPointF(P[0], P[1] + 10))
        p.setPen(QColor("#ffffff"))
        p.drawText(QPointF(P[0] + 9, P[1] - 9), "当前 TCP")

    def _draw_hud(self, p: QPainter) -> None:
        pos, (r, pi, y) = self.end_pose()
        prefix = "世界" if self.world_mount else "DH基座"
        lines = [f"TCP({prefix}) x {pos[0] * 1e3:+8.1f}  y {pos[1] * 1e3:+8.1f}  z {pos[2] * 1e3:+8.1f} mm",
                 f"          roll {r:+7.1f}  pitch {pi:+7.1f}  yaw {y:+7.1f} °"]
        if self.world_mount:
            T_dh = np.linalg.inv(self.table.base_transform) @ self.tcp_frame
            p_dh = T_dh[:3, 3]
            lines.append(f"TCP(DH) x {p_dh[0] * 1e3:+8.1f}  y {p_dh[1] * 1e3:+8.1f}  z {p_dh[2] * 1e3:+8.1f} mm")
        target_pose = self.target_pose()
        if self.show_target and target_pose is not None:
            target_pos, (target_r, target_pi, target_y) = target_pose
            lines.append(f"目标 TCP x {target_pos[0] * 1e3:+8.1f}  y {target_pos[1] * 1e3:+8.1f}  "
                         f"z {target_pos[2] * 1e3:+8.1f} mm")
            lines.append(f"         roll {target_r:+7.1f}  pitch {target_pi:+7.1f}  "
                         f"yaw {target_y:+7.1f} °")
        lines.append(f"DH：{self.table.source}")
        if self.show_workspace and self.workspace_info:
            lines.append(self.workspace_info)
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
