#!/usr/bin/env python3
"""URDF <-> MJCF 等效性检查 (纯标准库, 不依赖 mujoco 的 python 绑定).

检查内容 (世界坐标, q=0 与若干随机位形):
  1. 质量: 每个 body 与 URDF 逐项比较
  2. 惯性: 惯性系质心 (COM) 与绕质心的世界系惯量张量
  3. 几何: 每个 mesh 实例 (visual/collision) 按文件名 1:1 匹配世界位姿
  4. 关节: 轴方向 / 轴线位置 / 限位
  5. 改进 DH: 每个 body 原点相对 O_i = J(i-1)∩J(i) 的偏置 (仅报告, 不判失败)

可选 --dump: 读取 tools/mjdump 的输出, 用 MuJoCo 编译后的 body_inertia /
body_iquat 还原世界惯量, 独立复算一遍 (验证 fullinertia 分量顺序).

用法:
  python3 tools/urdf_check.py
  python3 tools/urdf_check.py --mjcf robot.xml --urdf ../【装配体】.../robot.urdf
  python3 tools/urdf_check.py --dump /tmp/dump.txt

退出码: 0 通过, 1 超差.
"""
import argparse
import math
import os
import random
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
MJ_DIR = os.path.dirname(HERE)

DEFAULT_URDF = os.path.join(MJ_DIR, "..", "【装配体】自定义控制器（DM4310）", "robot.urdf")
DEFAULT_MJCF = os.path.join(MJ_DIR, "robot.xml")

# 判定阈值
TOL = dict(body_R=1e-6, com=1e-6, inertia=1e-9, geom_p=1e-6, geom_R=1e-6,
           axis_dir=1e-3, axis_line=1e-6)
# body 原点是坐标系刻度, 允许与 URDF 不同 (只报告)


# ---------------- 四元数 / 矩阵 ----------------
def fv(s):
    return [float(x) for x in s.split()]


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def qrot(q, v):
    w, x, y, z = q
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def qaxis(axis, ang):
    n = math.sqrt(sum(a * a for a in axis))
    axis = [a / n for a in axis]
    s = math.sin(ang / 2)
    return (math.cos(ang / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def rpy2q(r, p, y):
    """URDF 的 rpy: R = Rz(yaw)·Ry(pitch)·Rx(roll)"""
    def ax(i, a):
        q = [0.0, 0.0, 0.0]
        q[i] = math.sin(a / 2)
        return (math.cos(a / 2), q[0], q[1], q[2])
    return qmul(ax(2, y), qmul(ax(1, p), ax(0, r)))


def Rm(q):
    return [[qrot(q, [1, 0, 0])[i], qrot(q, [0, 1, 0])[i], qrot(q, [0, 0, 1])[i]]
            for i in range(3)]


def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mt(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def rv(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def pose_mul(R1, p1, R2, p2):
    return mm(R1, R2), tuple(a + b for a, b in zip(p1, rv(R1, p2)))


def rot_err(R1, R2):
    return max(abs(R1[i][j] - R2[i][j]) for i in range(3) for j in range(3))


IDENT = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]


# ---------------- URDF 侧 ----------------
def load_urdf(path):
    root = ET.parse(path).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    return links, joints


def urdf_fk(links, joints, qmap):
    child_of = {j.find("child").get("link"): n for n, j in joints.items()}
    cache = {}

    def T(link):
        if link in cache:
            return cache[link]
        if link not in child_of:
            out = (IDENT, (0.0, 0.0, 0.0))
        else:
            j = joints[child_of[link]]
            o = j.find("origin")
            xyz = fv(o.get("xyz", "0 0 0")) if o is not None else [0, 0, 0]
            rpy = fv(o.get("rpy", "0 0 0")) if o is not None else [0, 0, 0]
            Rp, pp = T(j.find("parent").get("link"))
            R, p = pose_mul(Rp, pp, Rm(rpy2q(*rpy)), tuple(xyz))
            q = qmap.get(j.get("name"), 0.0)
            if q:
                ax = fv(j.find("axis").get("xyz", "0 0 1"))
                R, p = pose_mul(R, p, Rm(qaxis(ax, q)), (0, 0, 0))
            out = (R, p)
        cache[link] = out
        return out

    return {n: T(n) for n in links}


# ---------------- MJCF 侧 ----------------
def mjcf_joint_axes(root):
    """q=0 时各关节的世界轴线: {joint: (p, d)}"""
    out = {}

    def walk(el, R, p):
        for b in el.findall("body"):
            Rb, pb = pose_mul(R, p, Rm(tuple(fv(b.get("quat", "1 0 0 0")))),
                              tuple(fv(b.get("pos", "0 0 0"))))
            for j in b.findall("joint"):
                out[j.get("name")] = (pb, rv(Rb, fv(j.get("axis", "0 0 1"))))
            walk(b, Rb, pb)

    walk(root.find("worldbody"), IDENT, (0.0, 0.0, 0.0))
    return out


def mjcf_fk(root, qmap):
    out = {}

    def walk(el, R, p):
        for b in el.findall("body"):
            Rb, pb = pose_mul(R, p, Rm(tuple(fv(b.get("quat", "1 0 0 0")))),
                              tuple(fv(b.get("pos", "0 0 0"))))
            for j in b.findall("joint"):
                q = qmap.get(j.get("name"), 0.0)
                if q:
                    Rb, pb = pose_mul(Rb, pb, Rm(qaxis(fv(j.get("axis", "0 0 1")), q)),
                                      (0, 0, 0))
            out[b.get("name")] = (Rb, pb, b)
            walk(b, Rb, pb)

    walk(root.find("worldbody"), IDENT, (0.0, 0.0, 0.0))
    return out


def closest_on_line1(p1, a1, p2, a2):
    """两条直线的最小二乘公垂点 (取直线 1 上的点)"""
    b = sum(a1[k] * a2[k] for k in range(3))
    d = [p2[k] - p1[k] for k in range(3)]
    e = sum(a1[k] * d[k] for k in range(3))
    f = sum(a2[k] * d[k] for k in range(3))
    s = (e - b * f) / (1 - b * b)
    return (p1[0] + s * a1[0], p1[1] + s * a1[1], p1[2] + s * a1[2])


def parse_dump(path):
    rows = {}
    for line in open(path):
        t = line.split()
        if len(t) < 28:
            continue
        assert (t[5], t[9], t[14], t[18]) == ("I", "Q", "X", "R"), line
        rows[t[0]] = dict(
            mass=float(t[1]),
            ipos=[float(x) for x in t[2:5]],
            Ip=[float(x) for x in t[6:9]],
            iquat=[float(x) for x in t[10:14]],
            com=[float(x) for x in t[15:18]],
            R=[float(x) for x in t[19:28]],
        )
    return rows


def eig3(A):
    """3x3 对称阵特征值 (幂迭代 + 收缩), 仅用于对账"""
    B = [r[:] for r in A]
    out = []
    for _ in range(3):
        v = [1.0, 0.7, 0.3]
        for _ in range(300):
            w = [sum(B[i][j] * v[j] for j in range(3)) for i in range(3)]
            n = math.sqrt(sum(x * x for x in w)) or 1.0
            v = [x / n for x in w]
        lam = sum(v[i] * sum(B[i][j] * v[j] for j in range(3)) for i in range(3))
        out.append(lam)
        for i in range(3):
            for j in range(3):
                B[i][j] -= lam * v[i] * v[j]
    return sorted(out)


def main():
    ap = argparse.ArgumentParser(description="URDF ↔ MJCF 等效性检查")
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--mjcf", default=DEFAULT_MJCF)
    ap.add_argument("--trials", type=int, default=4, help="随机位形数量 (默认 4)")
    ap.add_argument("--dump", help="tools/mjdump 输出文件, 额外做编译后模型对账")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    for p in (args.urdf, args.mjcf):
        if not os.path.isfile(p):
            sys.exit("找不到文件: " + p)

    ulinks, ujoints = load_urdf(args.urdf)
    mroot = ET.parse(args.mjcf).getroot()
    meshes = {m.get("name"): os.path.basename(m.get("file"))
              for m in mroot.find("asset").findall("mesh")}
    bodies0 = mjcf_fk(mroot, {})
    mj_names = [n for n in bodies0 if n in ulinks]

    # ---------- 1. 质量 ----------
    print("=" * 76)
    print("1) 质量")
    print("=" * 76)
    print("%-8s %14s %14s %10s" % ("body", "URDF", "MJCF", "dmass"))
    tot_u = tot_m = 0.0
    fail = False
    for n in mj_names:
        m_u = float(ulinks[n].find("inertial").find("mass").get("value"))
        m_m = float(bodies0[n][2].find("inertial").get("mass"))
        tot_u += m_u
        tot_m += m_m
        print("%-8s %14.9f %14.9f %10.1e" % (n, m_u, m_m, m_m - m_u))
        if abs(m_m - m_u) > 1e-12:
            fail = True
    print("%-8s %14.9f %14.9f %10.1e" % ("TOTAL", tot_u, tot_m, tot_m - tot_u))

    # ---------- 2~4. 位形相关 ----------
    random.seed(args.seed)
    jnames = [j for j in ujoints]
    trials = [{}] + [{j: random.uniform(-3, 3) for j in jnames} for _ in range(args.trials)]
    w = dict(body_R=0.0, com=0.0, inertia=0.0, geom_p=0.0, geom_R=0.0,
             axis_dir=0.0, axis_line=0.0)
    geom_count = 0
    # q=0 的位姿, 供第 4/5 节使用 (与具体位形无关的刻度信息)
    uT0 = urdf_fk(ulinks, ujoints, {})
    mT0 = mjcf_fk(mroot, {})

    for qmap in trials:
        uT = urdf_fk(ulinks, ujoints, qmap)
        mT = mjcf_fk(mroot, qmap)
        for n in mj_names:
            Ru, pu = uT[n]
            Rm_, pm, bel = mT[n]
            w["body_R"] = max(w["body_R"], rot_err(Ru, Rm_))
            iu = ulinks[n].find("inertial")
            o = iu.find("origin")
            ixyz = fv(o.get("xyz", "0 0 0")) if o is not None else [0, 0, 0]
            irpy = fv(o.get("rpy", "0 0 0")) if o is not None else [0, 0, 0]
            ine = iu.find("inertia")
            M = [[float(ine.get("ixx")), float(ine.get("ixy")), float(ine.get("ixz"))],
                 [float(ine.get("ixy")), float(ine.get("iyy")), float(ine.get("iyz"))],
                 [float(ine.get("ixz")), float(ine.get("iyz")), float(ine.get("izz"))]]
            Rui = mm(Ru, Rm(rpy2q(*irpy)))
            com_u = [pu[i] + rv(Ru, ixyz)[i] for i in range(3)]
            Iu = mm(mm(Rui, M), mt(Rui))
            im = bel.find("inertial")
            ipos_m = fv(im.get("pos", "0 0 0"))
            Iq = Rm(tuple(fv(im.get("quat", "1 0 0 0"))))
            a, b_, c, d, e, f_ = fv(im.get("fullinertia"))
            Im_ = [[a, d, e], [d, b_, f_], [e, f_, c]]
            com_m = [pm[i] + rv(Rm_, ipos_m)[i] for i in range(3)]
            Iw = mm(mm(mm(Rm_, Iq), Im_), mt(mm(Rm_, Iq)))
            w["com"] = max(w["com"], math.dist(com_u, com_m))
            w["inertia"] = max(w["inertia"], max(abs(Iu[i][j] - Iw[i][j])
                                                 for i in range(3) for j in range(3)))
            for tag in ("visual", "collision"):
                for el in ulinks[n].findall(tag):
                    o = el.find("origin")
                    xyz = fv(o.get("xyz", "0 0 0")) if o is not None else [0, 0, 0]
                    rpy = fv(o.get("rpy", "0 0 0")) if o is not None else [0, 0, 0]
                    base = os.path.basename(el.find("geometry").find("mesh").get("filename"))
                    Rg, pg = pose_mul(Ru, pu, Rm(rpy2q(*rpy)), tuple(xyz))
                    hit = None
                    for bn, (Rb, pb, bel2) in mT.items():
                        for g in bel2.findall("geom"):
                            if meshes.get(g.get("mesh")) == base:
                                hit = (Rb, pb, g)
                    if hit is None:
                        sys.exit("MJCF 缺少 mesh: " + base)
                    Rb, pb, g = hit
                    Rgg, pgg = pose_mul(Rb, pb, Rm(tuple(fv(g.get("quat", "1 0 0 0")))),
                                        tuple(fv(g.get("pos", "0 0 0"))))
                    w["geom_p"] = max(w["geom_p"], math.dist(pg, pgg))
                    w["geom_R"] = max(w["geom_R"], rot_err(Rg, Rgg))
                    geom_count += 1
        for jn, j in ujoints.items():
            ax = fv(j.find("axis").get("xyz", "0 0 1"))
            child = j.find("child").get("link")
            au = rv(uT[child][0], ax)
            jm = mT[child][2].find("joint")
            am = rv(mT[child][0], fv(jm.get("axis", "0 0 1")))
            cosv = max(-1.0, min(1.0, sum(au[k] * am[k] for k in range(3))))
            w["axis_dir"] = max(w["axis_dir"], math.degrees(math.acos(cosv)))
            v = [mT[child][1][k] - uT[child][1][k] for k in range(3)]
            proj = sum(v[k] * au[k] for k in range(3))
            w["axis_line"] = max(w["axis_line"],
                                 math.sqrt(sum((v[k] - proj * au[k]) ** 2 for k in range(3))))

    print()
    print("=" * 76)
    print("2) 世界坐标误差 (%d 组位形, mesh 实例 %d 次比对)" % (len(trials), geom_count))
    print("=" * 76)
    rows = [("body 旋转", w["body_R"], TOL["body_R"]),
            ("惯性系质心 COM", w["com"], TOL["com"]),
            ("世界惯量张量", w["inertia"], TOL["inertia"]),
            ("mesh 位置", w["geom_p"], TOL["geom_p"]),
            ("mesh 旋转", w["geom_R"], TOL["geom_R"]),
            ("关节轴方向 (deg)", w["axis_dir"], TOL["axis_dir"]),
            ("关节轴线位置", w["axis_line"], TOL["axis_line"])]
    for label, val, tol in rows:
        ok = val <= tol
        fail = fail or not ok
        print("  %-20s %10.3e   (阈值 %.0e)  %s" % (label, val, tol, "OK" if ok else "FAIL"))

    # ---------- 3. 限位 ----------
    print()
    print("=" * 76)
    print("3) 关节限位")
    print("=" * 76)
    for jn in ujoints:
        lim = ujoints[jn].find("limit")
        lo, hi = float(lim.get("lower")), float(lim.get("upper"))
        child = ujoints[jn].find("child").get("link")
        jm = bodies0[child][2].find("joint")
        rng = fv(jm.get("range", "0 0"))
        ok = abs(rng[0] - lo) < 1e-9 and abs(rng[1] - hi) < 1e-9
        fail = fail or not ok
        print("  %-8s URDF [%+.5f, %+.5f]  MJCF [%+.5f, %+.5f]  %s"
              % (jn, lo, hi, rng[0], rng[1], "OK" if ok else "FAIL"))

    # ---------- 4. body 原点 vs 改进 DH 的 O_i ----------
    jax = mjcf_joint_axes(mroot)
    order = sorted(jax, key=lambda s: int("".join(c for c in s if c.isdigit())))
    O = {}
    for i in range(len(order) - 1):
        O[i + 2] = closest_on_line1(*jax[order[i]], *jax[order[i + 1]])
    print()
    print("=" * 76)
    print("4) body 原点相对 O_i = J(i-1)∩J(i) 的偏置 (坐标系刻度, 仅报告)")
    print("=" * 76)
    print("  %-8s %16s %16s" % ("body", "到 O_i (mm)", "到 O_(i+1) (mm)"))
    for n in mj_names:
        if not n.startswith("link"):
            continue
        i = int(n[4:])          # link_k  <->  O_k
        pm = mT0[n][1]
        d_i = 1000 * math.dist(pm, O[i]) if i in O else float("nan")
        d_n = 1000 * math.dist(pm, O[i + 1]) if (i + 1) in O else float("nan")
        print("  %-8s %16.6f %16.6f" % (n, d_i, d_n))

    # ---------- 5. 编译后模型 ----------
    if args.dump:
        d = parse_dump(args.dump)
        print()
        print("=" * 76)
        print("5) MuJoCo 编译后模型 (body_inertia/body_iquat 还原世界惯量)")
        print("=" * 76)
        print("  %-8s %8s %12s %12s %12s %12s"
              % ("body", "dmass", "dCOM_world", "dI_world", "dI_princ", "dCOM_body"))
        dw = [0.0] * 5
        for n in mj_names:
            row = d[n]
            iu = ulinks[n].find("inertial")
            o = iu.find("origin")
            ixyz = fv(o.get("xyz", "0 0 0")) if o is not None else [0, 0, 0]
            irpy = fv(o.get("rpy", "0 0 0")) if o is not None else [0, 0, 0]
            ine = iu.find("inertia")
            M = [[float(ine.get("ixx")), float(ine.get("ixy")), float(ine.get("ixz"))],
                 [float(ine.get("ixy")), float(ine.get("iyy")), float(ine.get("iyz"))],
                 [float(ine.get("ixz")), float(ine.get("iyz")), float(ine.get("izz"))]]
            mu = float(iu.find("mass").get("value"))
            Ru, pu = uT0[n]
            Rui = mm(Ru, Rm(rpy2q(*irpy)))
            com_u = [pu[i] + rv(Ru, ixyz)[i] for i in range(3)]
            Iu = mm(mm(Rui, M), mt(Rui))
            Iq = Rm(tuple(row["iquat"]))
            Ibody = mm(mm(Iq, [[row["Ip"][0], 0, 0], [0, row["Ip"][1], 0],
                               [0, 0, row["Ip"][2]]]), mt(Iq))
            R = [row["R"][0:3], row["R"][3:6], row["R"][6:9]]
            Im_ = mm(mm(R, Ibody), mt(R))
            diffs = [abs(row["mass"] - mu), math.dist(com_u, row["com"]),
                     max(abs(Iu[i][j] - Im_[i][j]) for i in range(3) for j in range(3)),
                     max(abs(eig3(M)[k] - sorted(row["Ip"])[k]) for k in range(3)),
                     math.dist(row["ipos"], ixyz)]
            dw = [max(dw[k], diffs[k]) for k in range(5)]
            print("  %-8s %8.1e %12.2e %12.2e %12.2e %12.2e" % (n, *diffs))
        print("  %-8s %8.1e %12.2e %12.2e %12.2e %12.2e" % ("MAX", *dw))
        for label, val, tol in (("dmass", dw[0], 1e-12), ("dCOM_world", dw[1], 1e-6),
                                ("dI_world", dw[2], 1e-9), ("dI_princ", dw[3], 1e-8)):
            if val > tol:
                print("  !! %s 超差 (%.2e > %.0e)" % (label, val, tol))
                fail = True
        print("  注: dCOM_body 非零表示该 body 原点与 URDF 不同 (刻度平移), "
              "world 量一致即无影响")

    print()
    print("结果:", "FAIL" if fail else "PASS — 与 URDF 等效")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
