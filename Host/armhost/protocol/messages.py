"""消息注册表与统一入口。"""

from __future__ import annotations

from dataclasses import dataclass

from .frame import Frame
from .msg_cmd import (ClearTorque, Estop, ParamRead, ParamValue, ParamWrite, SetJointTorque,
                      TorqueItem)
from .msg_link import DecodeError, DeviceInfo, DeviceInfoReq, Heartbeat
from .msg_report import Ack, CanBusStatus, Event, JointRecord, JointState, SystemStatus

MESSAGE_CLASSES = (Heartbeat, DeviceInfoReq, DeviceInfo, Ack, JointState, SystemStatus, Event,
                   SetJointTorque, ClearTorque, Estop, ParamRead, ParamWrite, ParamValue)
REGISTRY = {int(c.TYPE): c for c in MESSAGE_CLASSES}


@dataclass(slots=True)
class RawMessage:
    """未知类型：原样保留，供 hex 查看"""
    TYPE: int
    payload: bytes

    def encode(self) -> bytes:
        return self.payload


def decode_payload(msg_type: int, payload: bytes):
    cls = REGISTRY.get(msg_type)
    if cls is None:
        return RawMessage(msg_type, payload)
    return cls.decode(payload)


def decode_frame(frame: Frame):
    return decode_payload(frame.msg_type, frame.payload)


__all__ = [
    "Ack", "CanBusStatus", "ClearTorque", "DecodeError", "DeviceInfo", "DeviceInfoReq", "Estop",
    "Event", "Heartbeat", "JointRecord", "JointState", "MESSAGE_CLASSES", "ParamRead",
    "ParamValue", "ParamWrite", "REGISTRY", "RawMessage", "SetJointTorque", "SystemStatus",
    "TorqueItem", "decode_frame", "decode_payload",
]
