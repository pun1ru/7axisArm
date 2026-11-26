import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from test_c_gravity import _find_vcvars  # noqa: E402

FW = ROOT.parent / "Real" / "DIYcontroller" / "LibSE"
STUB = ROOT / "tests" / "c_stub"
MOTOR_STUB = ROOT / "tests" / "c_motor_stub"
DRIVER = ROOT / "tests" / "c_motor_status_driver.c"
SRC = [FW / "Drivers" / "Motor" / "Motor.c", DRIVER]
INC = [MOTOR_STUB, STUB, FW / "Drivers" / "Motor"]


def build(out: Path) -> bool:
    for cc in ("gcc", "clang"):
        exe = shutil.which(cc)
        if exe:
            cmd = [exe, "-std=gnu99", "-O1", *[f"-I{i}" for i in INC],
                   *map(str, SRC), "-o", str(out)]
            return subprocess.run(cmd, capture_output=True).returncode == 0

    vc = _find_vcvars()
    if vc is None:
        return False
    flags = " ".join(f'/I"{i}"' for i in INC)
    files = " ".join(f'"{s}"' for s in SRC)
    bat = (f'call "{vc}" >nul && cl /nologo /O1 /W3 /std:c11 '
           f'/D_CRT_SECURE_NO_WARNINGS {flags} {files} '
           f'/Fe:"{out}" /Fo:"{out.parent}\\\\"')
    result = subprocess.run(bat, shell=True, capture_output=True, text=True,
                            encoding="utf-8", errors="replace", cwd=out.parent)
    if result.returncode != 0:
        print(result.stdout[-3000:], result.stderr[-1000:])
    return result.returncode == 0


class TestMotorFeedbackTimeout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.tmp.name) / "motor_status.exe"
        if not build(cls.exe):
            raise unittest.SkipTest("no host C compiler")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_commands(self, *commands):
        result = subprocess.run(
            [str(self.exe)],
            input="\n".join(commands) + "\n",
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout.strip().splitlines()

    def test_feedback_timestamp_updates_liveness(self):
        out = self.run_commands(
            "config 6.5 0.2 1",
            "tick 1",
            "update",
            "dump",
            "tick 200",
            "update",
            "dump",
        )
        live = [float(v) for v in out[0].split()]
        timeout = [float(v) for v in out[1].split()]
        self.assertEqual(live[3], 1)
        self.assertAlmostEqual(live[0], 6.5, places=6)
        self.assertEqual(timeout[2], 10)
        self.assertEqual(timeout[3], 0)
        self.assertAlmostEqual(timeout[0], 6.5, places=6)

    def test_feedback_timeout_never_rewrites_multi_turn_angle(self):
        out = self.run_commands(
            "config 6.5 0.2 1",
            "tick 1",
            "update",
            "tick 200",
            "update",
            "dump",
        )
        values = [float(v) for v in out[-1].split()]
        self.assertAlmostEqual(values[0], 6.5, places=6)
        self.assertAlmostEqual(values[1], 0.2, places=6)
        self.assertEqual(values[2], 10)
        self.assertEqual(values[3], 0)


if __name__ == "__main__":
    unittest.main()
