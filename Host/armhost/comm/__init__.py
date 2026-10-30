"""通信模块：与 PySide6 无关，可单独使用与测试。"""

from .hexlog import HexLog, HexRecord, hexdump
from .link import AckTimeoutError, ArmLink, CommandRejected, LinkLostError, Received
from .mirror import Arm_t, ArmJointCtrl_t, ArmMirror, ArmObserver_t, Feedback_t
from .mock import FaultConfig, MockSerial, MockSerialFactory, SimDevice, make_mock_transport
from .stats import LinkStats, McuClock, SeqTracker
from .transport import LinkState, PortInfo, SerialConfig, SerialTransport, list_serial_ports
