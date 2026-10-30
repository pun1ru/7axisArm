"""串口协议：CRC、帧、消息、流式解析。纯 Python，不依赖 Qt / PySerial。"""

from .enums import *  # noqa: F401,F403
from .frame import (HEADER_SIZE, MAX_FRAME, MAX_PAYLOAD, OVERHEAD, PROTO_VERSION, SOF0, SOF1,
                    Frame, encode_frame)
from .messages import *  # noqa: F401,F403
from .parser import FrameParser, ParserStats
