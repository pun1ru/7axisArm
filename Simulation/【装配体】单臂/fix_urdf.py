#!/usr/bin/env python3
"""Rebuild the CAD-exported single-arm URDF into a usable simulation model.

The OVO exporter writes every joint/link origin as an absolute CAD coordinate and
mis-wires joint2 to link10 (link10 sorts right after link1).  This script:

  1. fixes the kinematic tree (base -> link1 ... link8, fingers link9/link10 on link8);
  2. redefines every joint frame in modified-DH (Craig) style, with the joint axis
     as the frame z axis (all URDF joint axes become 0 0 1);
  3. converts visual / collision / inertial data from CAD-absolute to link-local;
  4. adds prismatic joints for the two fingers along the gripper guide (CAD +/-Y).

q=0 keeps the exact CAD assembly; positive finger travel closes the gripper.

Usage:
  python fix_urdf.py                 # -> robot_fixed.urdf
  python fix_urdf.py --inplace       # backup robot_original.urdf, overwrite robot.urdf
"""
from __future__ import annotations

import argparse
import copy
import math
import re
import struct
import sys
from pathlib import Path

import numpy as np
import xml.etree.ElementTree as ET


def vec(e: ET.Element, key: str) -> np.ndarray:
    return np.array([float(x) for x in e.get(key).split()], dtype=float)


def fmt(v) -> str:
    return " ".join("%.10g" % (0.0 if abs(float(x)) < 1e-12 else float(x)) for x in v)


def normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return v / n


def rpy2R(rpy) -> np.ndarray:
    r, p, y = (float(x) for x in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def R2rpy(R: np.ndarray) -> np.ndarray:
    sp = -R[2, 0]
    if abs(sp) < 1 - 1e-9:
        return np.array(
            [math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, sp))), math.atan2(R[1, 0], R[0, 0])]
        )
    return np.array([math.atan2(-R[1, 2], R[1, 1]), math.copysign(math.pi / 2, sp), 0.0])


def foot(p1, u1, p2, u2):
    """Common-normal feet of two lines; None when parallel."""
    w = p1 - p2
    b, d, e = float(u1 @ u2), float(u1 @ w), float(u2 @ w)
    den = 1.0 - b * b
    if den < 1e-8:
        return None
    s, t = (b * e - d) / den, (e - b * d) / den
    return p1 + s * u1, p2 + t * u2


def frame_with_z(z: np.ndarray, x_hint: np.ndarray) -> np.ndarray:
    z = normalize(z)
    x = x_hint - z * float(x_hint @ z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0]) - z * z[0]
        if np.linalg.norm(x) < 1e-6:
            x = np.array([0.0, 1.0, 0.0]) - z * z[1]
    x = normalize(x)
    return np.column_stack([x, np.cross(z, x), z])


def read_urdf(path: Path):
    tree = ET.parse(path)
    root = tree.getroot()
    links = {lk.get("name"): lk for lk in root.findall("link")}
    joints = sorted(root.findall("joint"), key=lambda j: int(re.sub(r"\D", "", j.get("name"))))
    return tree, root, links, joints


def fix_tree(joints):
    """Re-attach link1..link8 in order; the exporter inserted link10 into the trunk."""
    if len(joints) != 8:
        raise ValueError(f"expected 8 revolute joints, got {len(joints)}")
    targets = [f"link{i}" for i in range(1, 9)]
    parents = ["base"] + targets[:-1]
    for j, parent, child in zip(joints, parents, targets):
        j.find("parent").set("link", parent)
        j.find("child").set("link", child)


def build_frames(p, z):
    """Modified-DH frames: O_i is the foot on axis i of the common normal to axis i+1."""
    n = len(z)
    O = [None] * n
    on_next = [None] * n
    for i in range(n - 1):
        res = foot(p[i], z[i], p[i + 1], z[i + 1])
        if res is None:
            d = p[i + 1] - p[i]
            O[i] = p[i] + z[i] * float(d @ z[i])
            on_next[i + 1] = p[i + 1]
        else:
            O[i], on_next[i + 1] = res
    O[n - 1] = on_next[n - 1] if on_next[n - 1] is not None else p[n - 1]

    x = [None] * n
    for i in range(n - 1):
        c = np.cross(z[i], z[i + 1])
        if np.linalg.norm(c) > 1e-8:
            x[i] = normalize(c)
        else:
            d = p[i + 1] - p[i]
            d = d - z[i] * float(d @ z[i])
            if np.linalg.norm(d) < 1e-9:
                d = np.array([0.0, 0.0, 1.0])
                d = d - z[i] * float(d @ z[i])
            x[i] = normalize(d)
    x[n - 1] = x[n - 2]
    R = [np.column_stack([x[i], np.cross(z[i], x[i]), z[i]]) for i in range(n)]
    return O, R


def transform_link(link: ET.Element, new_o: np.ndarray, new_R: np.ndarray):
    """All origins in the exported file are CAD-absolute; convert them to the new link frame."""
    for tag in ("visual", "collision"):
        for e in link.findall(tag):
            og = e.find("origin")
            q = vec(og, "xyz")
            Ro = rpy2R(vec(og, "rpy"))
            og.set("xyz", fmt(new_R.T @ (q - new_o)))
            og.set("rpy", fmt(R2rpy(new_R.T @ Ro)))

    ine = link.find("inertial")
    if ine is None:
        return
    og = ine.find("origin")
    q = vec(og, "xyz")
    Ro = rpy2R(vec(og, "rpy"))
    ie = ine.find("inertia")
    I_abs = np.array(
        [
            [float(ie.get("ixx")), float(ie.get("ixy")), float(ie.get("ixz"))],
            [float(ie.get("ixy")), float(ie.get("iyy")), float(ie.get("iyz"))],
            [float(ie.get("ixz")), float(ie.get("iyz")), float(ie.get("izz"))],
        ]
    )
    I_abs = Ro @ I_abs @ Ro.T
    og.set("xyz", fmt(new_R.T @ (q - new_o)))
    og.set("rpy", "0 0 0")
    I_new = new_R.T @ I_abs @ new_R
    for k, (r, c) in dict(ixx=(0, 0), ixy=(0, 1), ixz=(0, 2), iyy=(1, 1), iyz=(1, 2), izz=(2, 2)).items():
        ie.set(k, "%.12g" % I_new[r, c])


def stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"not a binary STL: {path}")
    n = struct.unpack_from("<I", data, 80)[0]
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    if len(data) < 84 + n * rec.itemsize:
        raise ValueError(f"bad STL length: {path}")
    return np.frombuffer(data, dtype=rec, count=n, offset=84)["v"].astype(float).reshape(-1, 3)


def T_origin(e: ET.Element) -> np.ndarray:
    og = e.find("origin")
    T = np.eye(4)
    if og is not None:
        T[:3, :3] = rpy2R(vec(og, "rpy"))
        T[:3, 3] = vec(og, "xyz")
    return T


def fk(root: ET.Element, qmap=None) -> dict[str, np.ndarray]:
    """Forward kinematics; qmap maps joint name -> displacement (default 0)."""
    qmap = qmap or {}
    joints = root.findall("joint")
    T = {"base": np.eye(4)}
    pending = list(joints)
    while pending:
        progressed = False
        for j in list(pending):
            parent = j.find("parent").get("link")
            if parent not in T:
                continue
            child = j.find("child").get("link")
            qi = float(qmap.get(j.get("name"), 0.0))
            Tj = T[parent] @ T_origin(j)
            axis = normalize(vec(j.find("axis"), "xyz"))
            Q = np.eye(4)
            if j.get("type") == "prismatic":
                Q[:3, 3] = axis * qi
            else:
                K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                Q[:3, :3] = np.eye(3) + math.sin(qi) * K + (1 - math.cos(qi)) * K @ K
            T[child] = Tj @ Q
            pending.remove(j)
            progressed = True
        if not progressed:
            raise ValueError("joint tree is disconnected: " + str([j.get("name") for j in pending]))
    return T


def make_reference(original_root: ET.Element) -> ET.Element:
    """CAD-absolute -> relative, so the original model can be evaluated with standard FK."""
    ref = copy.deepcopy(original_root)
    fix_tree(sorted(ref.findall("joint"), key=lambda j: int(re.sub(r"\D", "", j.get("name")))))
    links = {lk.get("name"): lk for lk in ref.findall("link")}
    joints = sorted(ref.findall("joint"), key=lambda j: int(re.sub(r"\D", "", j.get("name"))))
    pos = {"base": np.zeros(3)}
    for j in joints:
        pos[j.find("child").get("link")] = vec(j.find("origin"), "xyz")
    for j in joints:
        og = j.find("origin")
        og.set("xyz", fmt(pos[j.find("child").get("link")] - pos[j.find("parent").get("link")]))
    for name, lk in links.items():
        if name not in pos:
            pos[name] = np.zeros(3)
        for og in lk.iter("origin"):
            og.set("xyz", fmt(vec(og, "xyz") - pos[name]))
    return ref


def check_geometry(original_root: ET.Element, new_root: ET.Element):
    Tnew = fk(new_root)
    worst_p = worst_R = 0.0
    for lk in original_root.findall("link"):
        name = lk.get("name")
        if name not in Tnew:
            continue
        T = Tnew[name]
        new_lk = new_root.find(f"link[@name='{name}']")
        for tag in ("visual", "collision", "inertial"):
            for e_old, e_new in zip(lk.findall(tag), new_lk.findall(tag)):
                o_old = e_old.find("origin")
                p_old, R_old = vec(o_old, "xyz"), rpy2R(vec(o_old, "rpy"))
                Tn = T @ T_origin(e_new)
                worst_p = max(worst_p, float(np.abs(p_old - Tn[:3, 3]).max()))
                if tag != "inertial":
                    worst_R = max(worst_R, float(np.abs(R_old - Tn[:3, :3]).max()))
    return worst_p, worst_R


def check_kinematics(original_root: ET.Element, new_root: ET.Element, n: int = 64):
    ref = make_reference(original_root)
    rng = np.random.default_rng(20261104)
    worst = 0.0
    for _ in range(n):
        q = rng.uniform(-math.pi, math.pi, 8)
        qmap = {f"joint{i + 1}": q[i] for i in range(8)}
        Ta = fk(ref, qmap)
        Tb = fk(new_root, qmap)
        for name in ("link1", "link2", "link3", "link4", "link5", "link6", "link7", "link8"):
            # Link frames are allowed to differ by a constant frame redefinition;
            # compare the physical pose of the link's visual geometry instead.
            old_vis = ref.find(f"link[@name='{name}']").find("visual")
            new_vis = new_root.find(f"link[@name='{name}']").find("visual")
            A = Ta[name] @ T_origin(old_vis)
            B = Tb[name] @ T_origin(new_vis)
            worst = max(worst, float(np.abs(A - B).max()))
    return worst


def dh_table(z, O, R):
    rows = []
    for i in range(1, len(z)):
        xp, xi = R[i - 1][:, 0], R[i][:, 0]
        al = math.atan2(float(np.cross(z[i - 1], z[i]) @ xp), float(z[i - 1] @ z[i]))
        a = float((O[i] - O[i - 1]) @ xp)
        d = float((O[i] - O[i - 1]) @ z[i])
        th = math.atan2(float(np.cross(xp, xi) @ z[i]), float(xp @ xi))
        rows.append((i + 1, math.degrees(al), a, d, math.degrees(th)))
    return rows


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__)
    default_src = here / ("robot_original.urdf" if (here / "robot_original.urdf").exists() else "robot.urdf")
    ap.add_argument("src", nargs="?", default=str(default_src))
    ap.add_argument("--dst", default=str(here / "robot_fixed.urdf"))
    ap.add_argument("--inplace", action="store_true", help="overwrite robot.urdf, keeping robot_original.urdf")
    ap.add_argument("--finger-travel", type=float, default=0.015, help="per-finger travel in m (default 15 mm)")
    args = ap.parse_args()

    src = Path(args.src).resolve()
    dst = src if args.inplace else Path(args.dst).resolve()
    if args.inplace:
        backup = src.with_name("robot_original.urdf")
        if not backup.exists():
            backup.write_bytes(src.read_bytes())
            print("backup ->", backup)

    tree, root, links, joints = read_urdf(src)
    original_root = copy.deepcopy(root)
    fix_tree(joints)

    P, Z = [], []
    for j in joints:
        if np.abs(vec(j.find("origin"), "rpy")).max() > 1e-9:
            raise ValueError("joint rpy is not zero; input is not the CAD-absolute export")
        P.append(vec(j.find("origin"), "xyz"))
        Z.append(normalize(vec(j.find("axis"), "xyz")))

    O, R = build_frames(P, Z)

    Rp, op = np.eye(3), np.zeros(3)
    for i, j in enumerate(joints):
        og = j.find("origin")
        og.set("xyz", fmt(Rp.T @ (O[i] - op)))
        og.set("rpy", fmt(R2rpy(Rp.T @ R[i])))
        j.find("axis").set("xyz", "0 0 1")
        Rp, op = R[i], O[i]

    for i, j in enumerate(joints):
        transform_link(links[j.find("child").get("link")], O[i], R[i])
    # base stays at the world origin with identity orientation: the model remains upright.

    wrist_o, wrist_R = O[-1], R[-1]
    finger_specs = [
        ("gripper_finger1_joint", "link9", np.array([0.0, 1.0, 0.0])),
        ("gripper_finger2_joint", "link10", np.array([0.0, -1.0, 0.0])),
    ]
    finger_info = []
    for jname, child, slide_cad in finger_specs:
        verts = stl_vertices(here / "meshes" / "visual" / f"{child}.stl")
        c = (verts.min(axis=0) + verts.max(axis=0)) / 2.0
        Rf = frame_with_z(slide_cad, wrist_R[:, 0])
        finger_info.append((jname, child, c, Rf, slide_cad))
        transform_link(links[child], c, Rf)

        j = ET.SubElement(root, "joint", {"name": jname, "type": "prismatic"})
        ET.SubElement(j, "origin", {"xyz": fmt(wrist_R.T @ (c - wrist_o)), "rpy": fmt(R2rpy(wrist_R.T @ Rf))})
        ET.SubElement(j, "parent", {"link": "link8"})
        ET.SubElement(j, "child", {"link": child})
        ET.SubElement(j, "axis", {"xyz": "0 0 1"})
        ET.SubElement(j, "limit", {"lower": "0", "upper": "%.6g" % args.finger_travel,
                                   "effort": "50", "velocity": "0.1"})

    root.set("name", "single_arm_dh")
    ET.indent(tree, space="  ")
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    print("saved", dst)

    print("\njoint frames (new URDF, q=0):")
    for i, j in enumerate(joints):
        og = j.find("origin")
        print(f"  {j.get('name'):>8}  {j.find('parent').get('link'):>5} -> {j.find('child').get('link'):<6}"
              f"  xyz={og.get('xyz'):<34} rpy={og.get('rpy')}")
    print("\ngripper prismatic joints:")
    for jname, child, c, Rf, s in finger_info:
        print(f"  {jname}: link8 -> {child}, CAD ref {np.round(c, 6)}, guide (CAD) {s}, travel 0~{args.finger_travel} m")

    print("\nmodified DH (Craig): T_i = Rx(alpha_{i-1}) Tx(a_{i-1}) Rz(theta_i) Tz(d_i)")
    j1 = joints[0].find("origin")
    print("  base -> frame1: xyz=", np.round(vec(j1, "xyz"), 6), " rpy=", np.round(vec(j1, "rpy"), 6))
    print(f"  {'i':>2} {'alpha(deg)':>12} {'a(m)':>12} {'d(m)':>12} {'theta_off(deg)':>16}")
    for i, al, a, d, th in dh_table(Z, O, R):
        print(f"  {i:>2} {al:>12.4f} {a:>12.6f} {d:>12.6f} {th:>16.4f}")

    ref = ET.fromstring(ET.tostring(root, encoding="utf-8"))
    worst_p, worst_R = check_geometry(original_root, ref)
    worst_k = check_kinematics(original_root, ref)
    axes_ok = all(np.allclose(vec(j.find("axis"), "xyz"), [0, 0, 1]) for j in ref.findall("joint"))
    print("\nchecks:")
    print(f"  q=0 mesh/COM position error: {worst_p:.3e} m")
    print(f"  q=0 mesh orientation error:  {worst_R:.3e}")
    print(f"  64 random-pose FK error:     {worst_k:.3e}")
    print(f"  every joint axis == (0,0,1): {axes_ok}")
    ok = worst_p < 1e-8 and worst_R < 1e-8 and worst_k < 1e-8 and axes_ok
    print("result:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
