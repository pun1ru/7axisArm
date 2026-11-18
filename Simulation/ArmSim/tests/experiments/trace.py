import math, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ARMSIM_BIN = str(HERE.parents[1] / "build" / "armsim")
HOST = HERE.parents[3] / "Host"

sys.path.insert(0, str(HOST))
from armhost.comm import ArmLink, SerialConfig, SerialTransport
from armhost.protocol.enums import IkMethod
tgt = tuple(map(float, sys.argv[1:7]))
sim = subprocess.Popen([ARMSIM_BIN, "--host", "--no-viewer", "--no-arena"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
try:
    time.sleep(2)
    link = ArmLink(SerialTransport(SerialConfig(port="/tmp/armsim-host", baudrate=921600))); link.start(); time.sleep(1.5)
    m = link.mirror
    link.set_ik_target(m.posture.branch, *tgt, 0.0, sys.argv[7]=='A', (1.0,) * 7, (IkMethod.NUMERIC if sys.argv[7]=='N' else IkMethod.ANALYTIC)).result(1.0)
    for k in range(10):
        time.sleep(0.5)
        a = m.arm
        q = [f.abs_angle_deg for f in a.feedback]; t = [c.target_deg for c in a.joint_ctrl]
        lag = max(abs(x - y) for x, y in zip(q, t))
        p = m.posture
        per = " ".join(f"{x-y:+5.1f}" for x, y in zip(t, q))
        print(f"      cmd-obs per joint: {per}") if k in (9,) else None
        if k == 9: print("      torque out:", " ".join(f"{c.output:6.1f}" for c in a.joint_ctrl)); print("      obs q:", " ".join(f"{v:6.1f}" for v in q))
        print(f"t={0.5*(k+1):4.1f} pos_err={math.dist((p.x,p.y,p.z),tgt[:3])*1000:7.2f}mm max|cmd-obs|={lag:5.2f}° cmd=" + " ".join(f"{v:6.1f}" for v in t))
finally:
    sim.terminate()
