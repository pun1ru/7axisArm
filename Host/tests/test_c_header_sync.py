"""检查 firmware_ref/arm_proto.h 与 Python 协议定义保持同步（不需要 C 编译器）。"""

import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from armhost.protocol import MAX_FRAME, MAX_PAYLOAD, PROTO_VERSION, MsgType  # noqa: E402
from armhost.protocol.enums import (AckResult, EventCode, HostTargetAction, JointFlag, ParamType,  # noqa: E402
                                    StatusFlag)
from armhost.protocol.msg_report import JointRecord, JointState, SystemStatus  # noqa: E402
from armhost.protocol.params import BY_NAME  # noqa: E402

HEADER = (ROOT / "firmware_ref" / "arm_proto.h").read_text(encoding="utf-8")


def c_value(name: str) -> int:
    m = re.search(rf"\b{name}\s*=\s*(0x[0-9A-Fa-f]+|\d+)", HEADER) or \
        re.search(rf"#define\s+{name}\s+\(?(0x[0-9A-Fa-f]+|\d+)U?\)?", HEADER)
    if not m:
        raise AssertionError(f"{name} not found in arm_proto.h")
    return int(m.group(1), 0)


def c_enum_seq(first: str, names: list[str]) -> dict[str, int]:
    """解析 `A = 0, B, C` 这种连续枚举"""
    m = re.search(rf"{first}\s*=\s*(\d+)\s*,([^}}]*)}}", HEADER)
    base = int(m.group(1))
    rest = [s.strip() for s in m.group(2).replace("\n", " ").split(",") if s.strip()]
    return {n: base + i for i, n in enumerate([first] + rest)}


class TestCHeaderSync(unittest.TestCase):
    def test_frame_constants(self):
        self.assertEqual(c_value("ARM_PROTO_VERSION"), PROTO_VERSION)
        self.assertEqual(c_value("ARM_PROTO_MAX_FRAME"), MAX_FRAME)
        self.assertEqual(c_value("ARM_PROTO_MAX_FRAME") - 11, MAX_PAYLOAD)

    def test_msg_types(self):
        for t in MsgType:
            with self.subTest(t.name):
                self.assertEqual(c_value(f"ARM_MSG_{t.name}"), int(t))

    def test_ack_results(self):
        c = c_enum_seq("ARM_ACK_OK", [])
        for r in AckResult:
            self.assertEqual(c[f"ARM_ACK_{r.name}"], int(r))

    def test_flags(self):
        sf = {"RC_ONLINE": "ARM_SF_RC_ONLINE", "HOST_LINK_OK": "ARM_SF_HOST_LINK_OK",
              "ESTOP_LATCHED": "ARM_SF_ESTOP_LATCHED", "HOST_TORQUE_ACTIVE": "ARM_SF_HOST_TORQUE_ACTIVE",
              "CTRL_ALIGN_PENDING": "ARM_SF_CTRL_ALIGN_PENDING"}
        for py, c in sf.items():
            self.assertEqual(c_value(c), int(StatusFlag[py]))
        for f in ("ONLINE", "AT_LIMIT", "HOST_TORQUE_VALID", "TORQUE_CLAMPED",
                  "HOST_TARGET_VALID", "HOST_CLOSED_LOOP"):
            self.assertEqual(c_value(f"ARM_JF_{f}"), int(JointFlag[f]))

    def test_host_target_actions(self):
        c = c_enum_seq("ARM_HOST_ACTION_SET_OPEN", [])
        for a in HostTargetAction:
            self.assertEqual(c[f"ARM_HOST_ACTION_{a.name}"], int(a))

    def test_events(self):
        for e in EventCode:
            with self.subTest(e.name):
                self.assertEqual(c_value(f"ARM_EV_{e.name}"), int(e))

    def test_param_types_and_ids(self):
        c = c_enum_seq("ARM_PT_U8", [])
        for t in ParamType:
            self.assertEqual(c[f"ARM_PT_{t.name}"], int(t))
        for name, cname in [("state_rate_hz", "STATE_RATE_HZ"), ("zero_point", "ZERO_POINT"),
                            ("gain_kp", "GAIN_KP"), ("gain_output_max", "GAIN_OUTPUT_MAX"),
                            ("host_torque_limit_nm", "HOST_TORQUE_LIMIT"), ("dh", "DH")]:
            self.assertEqual(c_value(f"ARM_PARAM_{cname}"), BY_NAME[name].pid)

    def test_struct_sizes_in_static_asserts(self):
        sizes = dict(re.findall(r"sizeof\((\w+)\)\s*==\s*(\d+)", HEADER))
        self.assertEqual(int(sizes["ArmJointRecord_t"]), JointRecord.S.size)
        self.assertEqual(int(sizes["ArmMsgJointStateHead_t"]), JointState.H.size)
        self.assertEqual(int(sizes["ArmMsgSystemStatusHead_t"]), SystemStatus.H.size)


if __name__ == "__main__":
    unittest.main()
