#!/usr/bin/env python3
"""Convert the fixed BigArm URDF to MuJoCo MJCF.

The static base is merged into <worldbody> by default, so the MJCF has no
base body / base frame.  All link-local visual, collision and inertial data
are copied 1:1 from the URDF.  Mesh files are copied into ./meshes.

Usage:
  python make_mjcf.py
  python make_mjcf.py --keep-base        # keep a base body
"""
from __future__ import annotations

import argparse
import copy
import math
import shutil
import sys
from pathlib import Path

import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
DEFAULT_URDF = HERE.parent.parent / "【装配体】单臂" / "robot.urdf"


def vec(e, key):
    return [float(x) for x in e.get(key).split()]


def fmt(v):
    return " ".join("%.10g" % (0.0 if abs(float(x)) < 1e-12 else float(x)) for x in v)


def quat_from_rpy(rpy):
    r, p, y = (float(x) * 0.5 for x in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def rpy_to_matrix(rpy):
    r, p, y = (float(x) for x in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def origin_pose(e):
    og = e.find("origin")
    if og is None:
        return [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]
    return vec(og, "xyz"), quat_from_rpy(vec(og, "rpy"))


def read_tree(urdf_path: Path):
    root = ET.parse(urdf_path).getroot()
    links = {lk.get("name"): lk for lk in root.findall("link")}
    joints = root.findall("joint")
    outgoing = {}
    for j in joints:
        outgoing.setdefault(j.find("parent").get("link"), []).append(j)
    return links, joints, outgoing


class MeshBook:
    """Collect mesh assets and copy the referenced STL files."""

    def __init__(self, urdf_dir: Path, out_dir: Path):
        self.urdf_dir = urdf_dir
        self.out_dir = out_dir
        self.entries = []
        self._used = set()

    def add(self, link_name: str, tag: str, index: int, filename: str) -> str:
        base = f"{link_name}_{'vis' if tag == 'visual' else 'col%d' % index}"
        name = base
        k = 1
        while name in self._used:
            name = f"{base}_{k}"
            k += 1
        self._used.add(name)

        rel = Path(filename)
        src = self.urdf_dir / rel
        dst = self.out_dir / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        if not dst.exists() or src.stat().st_mtime > dst.stat().st_mtime:
            shutil.copy2(src, dst)
        # <compiler meshdir="meshes"> already adds the meshes/ prefix.
        asset_file = rel.relative_to("meshes") if rel.parts and rel.parts[0] == "meshes" else rel
        self.entries.append((name, asset_file.as_posix()))
        return name


def add_inertial(body, link, scale=1.0):
    ine = link.find("inertial")
    if ine is None:
        return
    mass = float(ine.find("mass").get("value")) * scale
    og = ine.find("origin")
    pos = vec(og, "xyz") if og is not None else [0.0, 0.0, 0.0]
    rpy = vec(og, "rpy") if og is not None else [0.0, 0.0, 0.0]
    R = rpy_to_matrix(rpy)
    ie = ine.find("inertia")
    I = [
        [float(ie.get("ixx")), float(ie.get("ixy")), float(ie.get("ixz"))],
        [float(ie.get("ixy")), float(ie.get("iyy")), float(ie.get("iyz"))],
        [float(ie.get("ixz")), float(ie.get("iyz")), float(ie.get("izz"))],
    ]
    # URDF inertia is in the inertial frame; rotate it into the body frame.
    Iw = [[sum(R[i][k] * I[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    Il = [[sum(Iw[i][k] * R[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    full = [Il[0][0], Il[1][1], Il[2][2], Il[0][1], Il[0][2], Il[1][2]]
    ET.SubElement(body, "inertial", {
        "pos": fmt(pos), "mass": "%.12g" % mass,
        "fullinertia": fmt(full),
    })


def add_geoms(parent, link, book: MeshBook, base_static=False):
    for i, g in enumerate(link.findall("visual")):
        mesh = g.find("geometry").find("mesh")
        name = book.add(link.get("name"), "visual", i, mesh.get("filename"))
        pos, quat = origin_pose(g)
        ET.SubElement(parent, "geom", {"class": "visual", "mesh": name,
                                       "pos": fmt(pos), "quat": fmt(quat)})
    for i, g in enumerate(link.findall("collision")):
        mesh = g.find("geometry").find("mesh")
        name = book.add(link.get("name"), "collision", i, mesh.get("filename"))
        pos, quat = origin_pose(g)
        attrs = {"class": "collision", "mesh": name, "pos": fmt(pos), "quat": fmt(quat)}
        if base_static:
            # The base is welded to the world; do not self-collide with link1.
            attrs["contype"] = "0"
            attrs["conaffinity"] = "0"
        ET.SubElement(parent, "geom", attrs)


def add_joint(body, joint):
    typ = joint.get("type")
    jt = "slide" if typ == "prismatic" else "hinge"
    axis = vec(joint.find("axis"), "xyz")
    attrs = {"name": joint.get("name"), "type": jt, "axis": fmt(axis)}
    lim = joint.find("limit")
    if lim is not None:
        attrs["range"] = "%s %s" % (lim.get("lower"), lim.get("upper"))
        attrs["limited"] = "true"
    ET.SubElement(body, "joint", attrs)


def add_body(parent, link_name, joint, links, outgoing, book, site_counter):
    body = ET.SubElement(parent, "body", {"name": link_name})
    if joint is not None:
        pos, quat = origin_pose(joint)
        body.set("pos", fmt(pos))
        body.set("quat", fmt(quat))
        add_joint(body, joint)
        ET.SubElement(body, "site", {
            "class": "dh", "name": "dh%d" % site_counter[0], "pos": "0 0 0",
        })
        site_counter[0] += 1

    link = links[link_name]
    add_inertial(body, link)
    add_geoms(body, link, book)
    for child_joint in outgoing.get(link_name, []):
        add_body(body, child_joint.find("child").get("link"), child_joint,
                 links, outgoing, book, site_counter)
    return body


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--urdf", default=str(DEFAULT_URDF))
    ap.add_argument("--out", default=str(HERE / "robot.xml"))
    ap.add_argument("--keep-base", action="store_true", help="keep a separate base body")
    args = ap.parse_args()

    urdf_path = Path(args.urdf).resolve()
    out_path = Path(args.out).resolve()
    out_dir = out_path.parent
    links, joints, outgoing = read_tree(urdf_path)
    book = MeshBook(urdf_path.parent, out_dir)

    root = ET.Element("mujoco", {"model": "single_arm_dh"})
    ET.SubElement(root, "compiler", {"angle": "radian", "meshdir": "meshes", "autolimits": "true"})
    ET.SubElement(root, "option", {"timestep": "0.002", "integrator": "implicitfast"})

    default = ET.SubElement(root, "default")
    d_vis = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(d_vis, "geom", {"type": "mesh", "group": "2", "contype": "0",
                                  "conaffinity": "0", "rgba": "0.75 0.75 0.78 1"})
    d_col = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(d_col, "geom", {"type": "mesh", "group": "3", "contype": "1",
                                  "conaffinity": "1", "rgba": "0.9 0.3 0.3 0.4"})
    d_dh = ET.SubElement(default, "default", {"class": "dh"})
    ET.SubElement(d_dh, "site", {"type": "sphere", "group": "1", "size": "0.003",
                                 "rgba": "1 0.55 0.1 0.9"})

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", {"pos": "0 0 1.5", "dir": "0 0 -1",
                                       "diffuse": "0.8 0.8 0.8"})
    # Classic checkerboard ground plane. The base geometry already sits on z=0,
    # and the base is the fixed root of the kinematic tree, so it stands on it.
    ET.SubElement(worldbody, "geom", {"name": "floor", "type": "plane",
                                      "size": "0 0 0.05", "material": "groundplane"})

    site_counter = [1]
    if args.keep_base:
        add_body(worldbody, "base", None, links, outgoing, book, site_counter)
    else:
        # Static base: merge its geoms into the world, so no base body/frame exists.
        add_geoms(worldbody, links["base"], book, base_static=True)
        for j in outgoing.get("base", []):
            add_body(worldbody, j.find("child").get("link"), j,
                     links, outgoing, book, site_counter)

    asset = ET.Element("asset")
    for name, filename in book.entries:
        ET.SubElement(asset, "mesh", {"name": name, "file": filename})
    ET.SubElement(asset, "texture", {
        "name": "groundplane", "type": "2d", "builtin": "checker",
        "mark": "edge", "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3",
        "markrgb": "0.8 0.8 0.8", "width": "300", "height": "300"})
    ET.SubElement(asset, "material", {
        "name": "groundplane", "texture": "groundplane",
        "texuniform": "true", "texrepeat": "5 5", "reflectance": "0.2"})
    root.insert(2, asset)

    contact = ET.Element("contact")
    for j in joints:
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        if args.keep_base or p != "base":
            ET.SubElement(contact, "exclude", {"body1": p, "body2": c})
    root.append(contact)

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(out_path, encoding="utf-8", xml_declaration=False)
    print("saved", out_path)
    print("meshes:", len(book.entries))


if __name__ == "__main__":
    sys.exit(main())
