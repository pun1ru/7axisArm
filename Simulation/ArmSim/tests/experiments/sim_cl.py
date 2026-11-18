import math
import subprocess
import sys
import time

import numpy as np

from pathlib import Path

HERE = Path(__file__).resolve().parent
ARMSIM_BIN = str(HERE.parents[1] / "build" / "armsim")
HOST = HERE.parents[3] / "Host"

sys.path.insert(0, str(HOST))

from armhost.comm import ArmLink, SerialConfig, SerialTransport  # noqa: E402
from armhost.protocol.enums import ArmState, IkMethod, PostureFlag  # noqa: E402


def rot(roll, pitch, yaw):
    r, p, y = map(math.radians, (roll, pitch, yaw))
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def pose_err(post, tgt):
    dp = math.dist((post.x, post.y, post.z), tgt[:3]) * 1000.0
    r1, r2 = rot(post.roll, post.pitch, post.yaw), rot(*tgt[3:])
    c = max(-1.0, min(1.0, (np.trace(r1.T @ r2) - 1.0) / 2.0))
    return dp, math.degrees(math.acos(c))


targets = []
for line in open(sys.argv[1] if len(sys.argv) > 1 else HERE / "targets.csv"):
    tag, *v = line.strip().split(",")
    targets.append((tag, tuple(map(float, v))))

sim = subprocess.Popen([ARMSIM_BIN, "--host", "--no-viewer",
                        "--no-arena"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
try:
    time.sleep(2.0)
    link = ArmLink(SerialTransport(SerialConfig(port="/tmp/armsim-host", baudrate=921600)))
    link.start()
    time.sleep(1.5)
    mir = link.mirror

    def q_now():
        return [f.abs_angle_deg for f in mir.arm.feedback]

    def go_home():
        try:
            link.stop_ik().result(1.0)
        except Exception:
            pass
        for i in range(7):
            link.set_host_target(i, 0.0, True).result(1.0)
        end = time.monotonic() + 15
        while time.monotonic() < end:
            if max(abs(v) for v in q_now()) < 0.4:
                break
            time.sleep(0.1)
        time.sleep(0.5)

    def settle(max_s):
        """等位姿不再变化（0.5 s 内变化 < 0.1 mm / 0.05°）或超时"""
        end = time.monotonic() + max_s
        prev = None
        while time.monotonic() < end:
            time.sleep(0.5)
            p = mir.posture
            cur = (p.x, p.y, p.z, p.roll, p.pitch, p.yaw)
            if prev is not None and math.dist(cur[:3], prev[:3]) < 1e-4 and max(abs(a - b) for a, b in zip(cur[3:], prev[3:])) < 0.05:
                break
            prev = cur
        return time.monotonic() - end + max_s

    print(f"{'目标':<4}{'方式':<8}{'结果':<22}{'位置误差mm':>11}{'姿态误差°':>10}{'用时s':>7}  备注")
    for tag, tgt in targets:
        for name, method, auto in (("解析解", IkMethod.ANALYTIC, True), ("数值解", IkMethod.NUMERIC, False)):
            go_home()
            br = mir.posture.branch
            t0 = time.monotonic()
            try:
                link.set_ik_target(br, *tgt, 0.0, auto, (1.0,) * 7, method).result(1.0)
                res = "ACK"
            except Exception as e:
                res = f"拒绝({str(e)[:14]})"
            if res != "ACK":
                print(f"{tag:<4}{name:<8}{res:<22}{'-':>11}{'-':>10}{'-':>7}")
                continue
            t_settle = settle(14 if method == IkMethod.NUMERIC else 8)
            p = mir.posture
            dp, dr = pose_err(p, tgt)
            st = (p.flags >> 4) & 3
            note = f"状态位={st}" + (" OutOfReach" if st == 1 else "")
            lim = [(-90, 90), (-20, 175), (-175, 175), (-110, 110), (-175, 175), (-120, 70), (-175, 175)]
            if not all(lo - 0.6 <= v <= hi + 0.6 for v, (lo, hi) in zip(q_now(), lim)):
                note += " 越限!"
            print(f"{tag:<4}{name:<8}{res:<22}{dp:>11.2f}{dr:>10.3f}{t_settle:>7.1f}  {note}")
    go_home()
finally:
    sim.terminate()
    try:
        sim.communicate(timeout=3)
    except Exception:
        sim.kill()
