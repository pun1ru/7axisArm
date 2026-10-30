"""生成协议文档里的示例帧（保证 CRC 与实现一致）。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armhost.protocol import (ArmState, Estop, EstopAction, Heartbeat, JointFlag,  # noqa: E402
                              JointRecord, JointState, MotorStatus, ParamRead, SetJointTorque,
                              StatusFlag, TorqueItem, encode_frame, Ack, AckResult)


def show(name, msg, seq, flags=0):
    f = encode_frame(msg.TYPE, seq, msg.encode(), flags)
    print(f"## {name} ({len(f)} B)")
    print(f.hex(" ").upper())
    print()


show("HEARTBEAT", Heartbeat(1000, 42), 0x0001)
show("SET_JOINT_TORQUE J0=+1.5 J6=-0.5 hold=100ms", SetJointTorque(100, [TorqueItem(0, 1.5), TorqueItem(6, -0.5)]), 0x0102, 1)
show("ACK", Ack(0x0102, 0x20, AckResult.OK, 0), 0x0200)
show("ESTOP latch", Estop(EstopAction.LATCH), 0x0103, 1)
show("PARAM_READ gain_kp[J2]", ParamRead(0x0200, 2), 0x0104, 1)
show("JOINT_STATE 1 joint", JointState(1_000_000, ArmState.ENABLE,
     StatusFlag.RC_ONLINE | StatusFlag.HOST_LINK_OK, 20,
     [JointRecord(12.5, -3.0, 0.25, 12.0, 0.3, 0x1234, MotorStatus.ENABLED, 0, 36, JointFlag.ONLINE, 7)]), 0xFFFF)
