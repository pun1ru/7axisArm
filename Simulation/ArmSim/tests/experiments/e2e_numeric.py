"""端到端：真实 armsim（--host，伪串口）+ Host 的 ArmLink，走协议发数值解目标。
覆盖：数值解进入并收敛、更新目标、回原位、不可达目标的 OutOfReach 状态位、数值解切回解析解。
用法：python3 -I e2e_numeric.py   （先编译 armsim：cmake --build ../../build --target armsim）"""
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ARMSIM_BIN = str(HERE.parents[1] / "build" / "armsim")
sys.path.insert(0, str(HERE.parents[3] / "Host"))

from armhost.comm import ArmLink, SerialConfig, SerialTransport  # noqa: E402
from armhost.protocol.enums import IkMethod, PostureFlag  # noqa: E402

LIMITS = [(-90, 90), (-20, 175), (-175, 175), (-110, 110), (-175, 175), (-120, 70), (-175, 175)]

sim = subprocess.Popen([ARMSIM_BIN, "--host", "--no-viewer", "--no-arena"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
try:
    time.sleep(2.0)
    tr = SerialTransport(SerialConfig(port="/tmp/armsim-host", baudrate=921600))
    link = ArmLink(tr)
    link.start()
    assert tr.wait_connected(3.0), "cannot open pty"
    time.sleep(1.0)
    p0 = link.mirror.posture
    print("state", link.mirror.arm.state, "posture", p0)
    assert p0 is not None and p0.valid

    def snap():
        return link.mirror.posture, [f.abs_angle_deg for f in link.mirror.arm.feedback]

    def wait(sec):
        time.sleep(sec)
        return snap()

    def in_limits(q):
        return all(lo - 0.5 <= v <= hi + 0.5 for v, (lo, hi) in zip(q, LIMITS))

    W = (1.0,) * 7
    # 1. 数值解：从当前位姿沿 +z 抬 0.06 m、沿 +x 移 0.04 m
    tx, ty, tz = p0.x + 0.04, p0.y, p0.z + 0.06
    r = link.set_ik_target(p0.branch, tx, ty, tz, p0.roll, p0.pitch, p0.yaw, 0.0, False, W,
                           IkMethod.NUMERIC).result(1.0)
    print("ack numeric", r)
    for k in range(8):
        p, q = wait(0.5)
        err = ((p.x - tx) ** 2 + (p.y - ty) ** 2 + (p.z - tz) ** 2) ** 0.5
        print(f"t={0.5 * (k + 1):.1f}s err={err * 1000:.2f} mm flags={PostureFlag(p.flags)!r} "
              f"q={[round(v, 1) for v in q]}")
        assert in_limits(q), "limit violated"
    assert PostureFlag.NUMERIC_IK in PostureFlag(p.flags), "NUMERIC_IK flag missing"
    assert err < 0.003, f"numeric ik did not converge: {err}"

    # 2. 更新目标（保持数值解）：回到原位
    link.set_ik_target(p0.branch, p0.x, p0.y, p0.z, p0.roll, p0.pitch, p0.yaw, 0.0, False, W,
                       IkMethod.NUMERIC).result(1.0)
    p, q = wait(4.0)
    err = ((p.x - p0.x) ** 2 + (p.y - p0.y) ** 2 + (p.z - p0.z) ** 2) ** 0.5
    print(f"return err={err * 1000:.2f} mm q={[round(v, 1) for v in q]}")
    assert err < 0.003

    # 3. 不可达目标：约 0.5 s 没动就应上报 OutOfReach（flags bit4~5 = 1），且不越限
    link.set_ik_target(p0.branch, 3.0, 0.0, 0.5, 0, 0, 0, 0.0, False, W, IkMethod.NUMERIC).result(1.0)
    p, q = wait(10.0)
    print("unreachable flags", PostureFlag(p.flags), "status bits", (p.flags >> 4) & 3, [round(v, 1) for v in q])
    assert in_limits(q)
    assert (p.flags >> 4) & 3 == 1, "stall not reported as OutOfReach"

    # 4. 数值解切回解析解（解析解要求分支与当前构型一致）
    link.set_ik_target(p0.branch, p0.x, p0.y, p0.z, p0.roll, p0.pitch, p0.yaw, p0.psi_deg, False, W,
                       IkMethod.NUMERIC).result(1.0)
    wait(3.0)
    p, q = snap()
    r = link.set_ik_target(p.branch, p.x, p.y, p.z, p.roll, p.pitch, p.yaw, p.psi_deg, False, W,
                           IkMethod.ANALYTIC).result(1.0)
    print("ack analytic switch", r)
    p, q = wait(1.0)
    print("after analytic flags", PostureFlag(p.flags))
    assert PostureFlag.NUMERIC_IK not in PostureFlag(p.flags)

    # 5. 非法方式字节：Host 编码阶段就拒绝（固件侧 BAD_PARAM 分支没有走线测试）
    try:
        link.set_ik_target(p.branch, p.x, p.y, p.z, 0, 0, 0, 0, False, W, 7).result(1.0)
        print("method=7 unexpectedly accepted")
    except Exception as e:
        print("method=7 rejected:", type(e).__name__, e)

    link.stop_ik().result(1.0)
    print("E2E OK")
finally:
    sim.terminate()
    try:
        out = sim.communicate(timeout=3)[0]
        print("--- sim log tail ---")
        print("\n".join(out.splitlines()[-4:]))
    except Exception:
        sim.kill()
