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
DRIVER = ROOT / "tests" / "c_lkmotor_driver.c"
SRC = [FW / "Drivers" / "Motor" / "LKMotor" / "LKMotor.c", DRIVER]
INC = [STUB, FW / "Drivers" / "Motor", FW / "Drivers" / "Motor" / "LKMotor"]

MODE_POSITION = 0
MODE_TORQUE = 2


def build(out: Path) -> bool:
    for cc in ("gcc", "clang"):
        exe = shutil.which(cc)
        if exe:
            cmd = [exe, "-std=gnu99", "-O1", *[f"-I{i}" for i in INC],
                   *map(str, SRC), "-lm", "-o", str(out)]
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


class TestLKMotorPositionMode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.tmp.name) / "lkmotor.exe"
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

    @staticmethod
    def init_line(motor_id=1, zero_point=32768, mode=MODE_POSITION):
        feedback_id = 0x140 + motor_id
        return f"init 2 {motor_id} {feedback_id} {zero_point} 0 {mode}"

    def test_position_feedback_uses_raw_multiturn_minus_zero(self):
        out = self.run_commands(
            self.init_line(),
            "raw 54000",
            "dump",
            "raw -18000",
            "dump",
        )
        positive = [float(v) for v in out[-2].split()]
        negative = [float(v) for v in out[-1].split()]
        self.assertAlmostEqual(positive[0], 360.0, places=5)
        self.assertAlmostEqual(positive[1], 0.0, places=5)
        self.assertAlmostEqual(negative[0], -360.0, places=5)
        self.assertAlmostEqual(negative[1], 0.0, places=5)

    def test_position_mode_does_not_apply_gear_ratio(self):
        out = self.run_commands(
            "init 1 1 385 32768 0 0",
            "clearframes",
            "pos 1.5707963267948966 3.141592653589793",
            "frames",
            "raw 54000",
            "dump",
        )
        end = out.index("END")
        feedback = [float(v) for v in out[end + 1].split()]
        self.assertAlmostEqual(feedback[0], 360.0, places=5)
        self.assertEqual(out[0], "141 A4 00 B4 00 78 69 00 00")

    def test_state2_does_not_fake_position_feedback_before_0x92(self):
        out = self.run_commands(
            self.init_line(),
            "state 156 25 0 0 0",
            "dump",
            "raw 32768",
            "dump",
        )
        before = [float(v) for v in out[-2].split()]
        after = [float(v) for v in out[-1].split()]
        self.assertEqual(before[6], 10)
        self.assertEqual(after[6], 1)

    def test_position_mode_sends_a4_with_raw_absolute_target(self):
        out = self.run_commands(
            self.init_line(),
            "clearframes",
            "pos 0 3.141592653589793",
            "frames",
        )
        frames = out[: out.index("END")]
        self.assertEqual(frames[0], "141 A4 00 B4 00 50 46 00 00")
        self.assertEqual(frames[1], "141 92 00 00 00 00 00 00 00")

    def test_position_mode_default_speed_is_30_dps(self):
        out = self.run_commands(
            self.init_line(),
            "clearframes",
            "pos 0 0",
            "frames",
        )
        self.assertEqual(out[0], "141 A4 00 1E 00 50 46 00 00")

    def test_position_mode_uses_motor_id_in_command_frame(self):
        out = self.run_commands(
            self.init_line(motor_id=6),
            "clearframes",
            "pos 0 0",
            "frames",
        )
        self.assertTrue(out[0].startswith("146 A4 "))

    def test_driver_uses_140_base_feedback_id(self):
        out = self.run_commands(
            "init 2 6 326 32768 0 0",
            "ids",
        )
        self.assertEqual(out[-1], "326 326 0")

    def test_driver_ignores_180_reply_id_variant(self):
        out = self.run_commands(
            self.init_line(),
            "rawid 385 54000",
            "ids",
            "rawid 321 54000",
            "ids",
            "dump",
        )
        self.assertEqual(out[0], "321 321 0")
        self.assertEqual(out[1], "321 321 321")
        self.assertAlmostEqual(float(out[-1].split()[0]), 360.0, places=5)

    def test_torque_mode_keeps_a1_and_does_not_schedule_position_polling(self):
        out = self.run_commands(
            self.init_line(mode=MODE_TORQUE),
            "clearframes",
            "torque 1.0",
            "frames",
        )
        frames = out[: out.index("END")]
        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0].startswith("141 A1 "))

    def test_torque_mode_keeps_legacy_single_turn_angle_path(self):
        out = self.run_commands(
            self.init_line(mode=MODE_TORQUE),
            "raw 54000",
            "dump",
        )
        feedback = [float(v) for v in out[-1].split()]
        self.assertAlmostEqual(feedback[0], 0.0, places=5)

    def test_position_mode_schedules_angle_state_and_error_polls(self):
        commands = [self.init_line(), "clearframes"]
        commands.extend(["pos 0 3.141592653589793"] * 110)
        commands.append("frames")
        out = self.run_commands(*commands)
        frames = out[: out.index("END")]
        commands_by_code = [int(line.split()[1], 16) for line in frames]
        self.assertEqual(commands_by_code.count(0xA4), 110)
        self.assertEqual(commands_by_code.count(0x92), 110)
        self.assertEqual(commands_by_code.count(0x9C), 3)
        self.assertEqual(commands_by_code.count(0x9A), 2)

    def test_position_feedback_request_runs_maintenance_without_control_frame(self):
        commands = [self.init_line(), "clearframes"]
        commands.extend(["req"] * 110)
        commands.append("frames")
        out = self.run_commands(*commands)
        frames = out[: out.index("END")]
        commands_by_code = [int(line.split()[1], 16) for line in frames]
        self.assertEqual(commands_by_code.count(0xA4), 0)
        self.assertEqual(commands_by_code.count(0x92), 110)
        self.assertEqual(commands_by_code.count(0x9C), 3)
        self.assertEqual(commands_by_code.count(0x9A), 2)


if __name__ == "__main__":
    unittest.main()
