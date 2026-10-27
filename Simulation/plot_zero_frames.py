"""零位 (q=0) 关节坐标系示意图：读取 URDF + visual STL，在模型上画出各关节系 x/y/z 轴。
运行：python plot_zero_frames.py  ->  输出 zero_pose_frames.png
"""
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "【装配体】自定义控制器（DM4310）")
URDF = os.path.join(MODEL, "robot.urdf")
OUT = os.path.join(HERE, "zero_pose_frames.png")

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def parse_vec(s):
    return [float(v) for v in s.split()]


def load_stl(path):
    """二进制 STL -> (N,3,3) 三角形顶点"""
    with open(path, "rb") as f:
        f.read(80)
        n = int(np.frombuffer(f.read(4), dtype="<u4")[0])
        rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        data = np.frombuffer(f.read(n * rec.itemsize), dtype=rec)
    return data["v"].astype(float)


def parse_urdf(path):
    src = open(path, encoding="utf-8").read()
    visuals = {}
    for name, body in re.findall(r'<link name="(\w+)">(.*?)</link>', src, re.S):
        m = re.search(r'<visual>\s*<origin xyz="([^"]+)" rpy="([^"]+)"\s*/>\s*<geometry>\s*'
                      r'<mesh filename="([^"]+)"', body)
        if m:
            visuals[name] = (make_T(parse_vec(m.group(1)), parse_vec(m.group(2))), m.group(3))
    joints = []
    for name, body in re.findall(r'<joint name="(\w+)" type="revolute">(.*?)</joint>', src, re.S):
        o = re.search(r'<origin xyz="([^"]+)" rpy="([^"]+)"', body)
        parent = re.search(r'<parent link="(\w+)"', body).group(1)
        child = re.search(r'<child link="(\w+)"', body).group(1)
        joints.append((name, parent, child, make_T(parse_vec(o.group(1)), parse_vec(o.group(2)))))
    return visuals, joints


# 各关节系在零位下的文字说明（base 系方向，与 DH 表对应）
DESC = {
    1: "z1: base +Z（竖直向上）\nx1: base +X",
    2: "z2: base −Y\nx2 = z2×z3: 垂直上臂, 指向前下方",
    3: "z3: 沿上臂, 肘→肩\nx3 = z3×z4: ≈x2, 垂直上臂前下方",
    4: "z4: ≈base +Y（与 z2 反向）\nx4 = z4×z5: ≈base +X",
    5: "z5: 沿前臂, 肘→腕（向上）\nx5 = z5×z6: ≈base +X",
    6: "z6: ≈base −Y\nx6 = z7×z6: ≈base +Z（沿前臂向上）",
    7: "z7: ≈base −X\nx7 = x6: ≈base +Z",
}
AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")  # x 红, y 绿, z 蓝


def forward(visuals, joints):
    """零位下各 link 位姿（base 系）"""
    T_link = {"base": np.eye(4)}
    frames = []
    for name, parent, child, T in joints:
        T_link[child] = T_link[parent] @ T
        frames.append((int(name.replace("joint", "")), T_link[child]))
    meshes = []
    for link, (T_vis, fn) in visuals.items():
        tri = load_stl(os.path.join(MODEL, fn))
        T = T_link[link] @ T_vis
        meshes.append(tri @ T[:3, :3].T + T[:3, 3])
    return frames, meshes


def draw_mesh(ax, meshes, alpha):
    for tri in meshes:
        pc = Poly3DCollection(tri, facecolor="#9aa5b1", edgecolor="none", alpha=alpha)
        ax.add_collection3d(pc)


def draw_frame(ax, T, L, label=None, lw=2.0, fs=9):
    o = T[:3, 3]
    for k, (c, n) in enumerate(zip(AXIS_COLORS, "xyz")):
        d = T[:3, k] * L
        ax.quiver(*o, *d, color=c, lw=lw, arrow_length_ratio=0.18)
        if label is not None:
            ax.text(*(o + d * 1.15), f"{n}{label}", color=c, fontsize=fs, weight="bold")


def setup_ax(ax, center, half, title):
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=18, azim=-58)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.tick_params(labelsize=6)
    ax.set_title(title, fontsize=11)


def main():
    visuals, joints = parse_urdf(URDF)
    frames, meshes = forward(visuals, joints)
    pts = np.concatenate([m.reshape(-1, 3) for m in meshes])
    center = (pts.max(0) + pts.min(0)) / 2
    half = (pts.max(0) - pts.min(0)).max() / 2 * 1.05

    fig = plt.figure(figsize=(22, 16))
    gs = fig.add_gridspec(3, 4)

    # 总览：模型 + 全部关节系 + base 系
    ax = fig.add_subplot(gs[0:2, 0:2], projection="3d")
    draw_mesh(ax, meshes, 0.25)
    draw_frame(ax, np.eye(4), 0.05, label="b", lw=2.5, fs=11)
    # 原点重合的关节标签错开放置，并用细线引回原点
    label_off = {1: (-0.04, 0, -0.03), 2: (-0.05, 0, 0.02), 3: (0.03, 0, -0.04), 4: (0.03, 0, 0.03),
                 5: (-0.05, 0, 0.03), 6: (0.0, 0, 0.05), 7: (0.05, 0, 0.03)}
    for i, T in frames:
        draw_frame(ax, T, 0.035, lw=1.8)
        o = T[:3, 3]; p = o + np.array(label_off[i])
        ax.plot(*zip(o, p), color="#555", lw=0.8, ls="--")
        ax.text(*p, f"J{i}", fontsize=12, weight="bold")
    setup_ax(ax, center, half, "零位 (q=0) 总览：base 系 + 关节系 1~7\n(红 x / 绿 y / 蓝 z，z 为转轴)")

    # 单个关节系特写（关节系原点有重合，分开画更清楚）
    cells = [(0, 2), (0, 3), (1, 2), (1, 3), (2, 0), (2, 1), (2, 2)]
    for (i, T), (r, c) in zip(frames, cells):
        ax = fig.add_subplot(gs[r, c], projection="3d")
        draw_mesh(ax, meshes, 0.12)
        draw_frame(ax, T, 0.07, label=str(i), lw=2.5, fs=11)
        setup_ax(ax, center, half, f"关节系 {i}")
        ax.text2D(0.02, 0.02, DESC[i], transform=ax.transAxes, fontsize=9,
                  bbox=dict(facecolor="white", alpha=0.85, edgecolor="#999"))

    # 说明栏
    ax = fig.add_subplot(gs[2, 3]); ax.axis("off")
    ax.text(0, 1, (
        "坐标系规定（改进 DH / Craig）\n\n"
        "• z_i：第 i 个电机转轴，URDF axis=(0 0 1)，\n"
        "   q_i>0 绕 +z_i 右手旋转\n"
        "• x_i：z_i 与 z_{i+1} 的公垂线，经两轴交点\n"
        "   i=1~5: x_i = z_i × z_{i+1}  (α_i=+90°)\n"
        "   i=6:   x_6 = z_7 × z_6      (α_6=−90°)\n"
        "   i=7:   x_7 = x_6            (θ7 偏置=0)\n"
        "• y_i = z_i × x_i\n\n"
        "零位姿态：上臂与水平面成 30° 斜向上，前臂竖直\n"
        "上臂 d3=148 mm，前臂 d5=100 mm\n"
        "J1/J2 原点相距 4 mm、J3/J4 2.5 mm、J5~J7 ≤1 mm，\n"
        "总览中近似重合"), va="top", fontsize=11, family="Microsoft YaHei")

    fig.tight_layout()
    fig.savefig(OUT, dpi=130)
    print("saved", OUT)


if __name__ == "__main__":
    main()
