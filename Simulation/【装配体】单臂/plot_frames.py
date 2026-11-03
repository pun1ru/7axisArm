#!/usr/bin/env python3
"""Plot the fixed URDF meshes and joint frames at q=0.

The base frame is intentionally not drawn: link frames J1..J8 and the two
prismatic finger axes are shown on top of the CAD assembly.
"""
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import xml.etree.ElementTree as ET

from fix_urdf import fk, stl_vertices, T_origin


HERE = Path(__file__).resolve().parent
OUT = HERE / "frames.png"
AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")


def load_meshes(root, T):
    out = []
    for lk in root.findall("link"):
        name = lk.get("name")
        if name not in T:
            continue
        for vis in lk.findall("visual"):
            fn = vis.find("geometry").find("mesh").get("filename")
            tri = stl_vertices(HERE / fn).reshape(-1, 3, 3)
            A = T[name] @ T_origin(vis)
            out.append((name, tri @ A[:3, :3].T + A[:3, 3]))
    return out


def draw_frame(ax, T, length, label):
    o = T[:3, 3]
    for k, c in enumerate(AXIS_COLORS):
        d = T[:3, k] * length
        ax.quiver(*o, *d, color=c, arrow_length_ratio=0.2, linewidth=1.6)
    ax.text(*(o + np.array([0, 0, 0.012])), label, fontsize=8, weight="bold", color="#333333")


def main():
    root = ET.parse(HERE / "robot.urdf").getroot()
    T = fk(root)
    meshes = load_meshes(root, T)
    pts = np.concatenate([tri.reshape(-1, 3) for _, tri in meshes])
    ctr = (pts.max(0) + pts.min(0)) / 2
    half = (pts.max(0) - pts.min(0)).max() / 2 * 1.08

    fig = plt.figure(figsize=(16, 8))
    for k, (elev, azim, title) in enumerate(
        [(22, -58, "isometric"), (0, -90, "side XZ"), (90, -90, "top XY")]
    ):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        for name, tri in meshes:
            pc = Poly3DCollection(tri, facecolor="#9aa5b1", edgecolor="none", alpha=0.28)
            ax.add_collection3d(pc)
        for i in range(1, 9):
            name = f"link{i}"
            if name in T:
                draw_frame(ax, T[name], 0.035 if i < 8 else 0.025, f"J{i}")
        for name, label in (("link9", "finger1"), ("link10", "finger2")):
            if name in T:
                o = T[name][:3, 3]
                d = T[name][:3, 2] * 0.05
                ax.quiver(*o, *d, color="#e8710a", arrow_length_ratio=0.25, linewidth=2.0)
                ax.text(*(o + d), label, fontsize=8, color="#e8710a")
        ax.set_xlim(ctr[0] - half, ctr[0] + half)
        ax.set_ylim(ctr[1] - half, ctr[1] + half)
        ax.set_zlim(ctr[2] - half, ctr[2] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.tick_params(labelsize=6)
    fig.suptitle("BigArm q=0: joint frames (z = joint axis); base frame omitted", fontsize=12)
    fig.tight_layout()
    fig.savefig(OUT, dpi=160)
    print("saved", OUT)


if __name__ == "__main__":
    main()
