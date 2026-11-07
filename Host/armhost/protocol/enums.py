"""协议枚举。凡是标注"同 xxx"的，数值必须与固件 C 枚举保持一致。"""

from __future__ import annotations

from enum import IntEnum, IntFlag


class MsgType(IntEnum):
    # 0x00~0x0F 链路
    HEARTBEAT = 0x01          # 双向
    DEVICE_INFO_REQ = 0x02    # host -> mcu
    DEVICE_INFO = 0x03        # mcu -> host
    ACK = 0x04                # mcu -> host
    # 0x10~0x1F 上报
    JOINT_STATE = 0x10        # mcu -> host 周期
    SYSTEM_STATUS = 0x11      # mcu -> host 周期
    EVENT = 0x12              # mcu -> host 事件
    ARM_POSTURE = 0x13        # mcu -> host 周期，分支 / 臂角 / 末端位姿
    # 0x20~0x2F 控制
    SET_JOINT_TORQUE = 0x20   # host -> mcu
    CLEAR_TORQUE = 0x21       # host -> mcu
    ESTOP = 0x22              # host -> mcu
    SET_HOST_TARGET = 0x23    # host -> mcu
    SET_IK_TARGET = 0x24      # host -> mcu，进入/更新/退出 HOST_IK
    # 0x28~0x2F 参数
    PARAM_READ = 0x28         # host -> mcu
    PARAM_WRITE = 0x29        # host -> mcu
    PARAM_VALUE = 0x2A        # mcu -> host
    # 0x30~0x3F 预留给轨迹 / 仿真同步
    EXCHANGE_COMMAND = 0x30   # host -> ArmSim，兑换站仿真扩展
    EXCHANGE_STATE = 0x31     # ArmSim -> host，兑换站状态


class ArmState(IntEnum):
    """同 arm.h ArmState_t"""
    PROTECTED = 0
    ENABLE = 1
    FEEDBACK = 2
    MASTER = 3
    AUTO = 4
    SERVANT = 5
    HOST = 6
    HOST_IK = 7


class ArmRole(IntEnum):
    """同 arm.h ArmRole_t"""
    MASTER = 0
    SERVANT = 1


class MotorStatus(IntEnum):
    """同 Motor.h MotorStatus_t"""
    DISABLED = 0
    ENABLED = 1
    FAULT = 2
    OVERVOLTAGE = 3
    UNDERVOLTAGE = 4
    OVERCURRENT = 5
    MOS_OVERTEMP = 6
    COIL_OVERTEMP = 7
    COMM_LOST = 8
    OVERLOAD = 9
    NO_FEEDBACK = 10


class FrameFlag(IntFlag):
    NONE = 0
    ACK_REQ = 0x01            # 发送方要求 ACK


class StatusFlag(IntFlag):
    """JOINT_STATE / SYSTEM_STATUS 的 status_flags"""
    NONE = 0
    RC_ONLINE = 0x0001
    HOST_LINK_OK = 0x0002
    ESTOP_LATCHED = 0x0004
    HOST_TORQUE_ACTIVE = 0x0008   # 至少一个关节的上位机力矩设定在有效期内
    CTRL_ALIGN_PENDING = 0x0010
    HOST_CLOSED_LOOP = 0x0020


class JointFlag(IntFlag):
    NONE = 0
    ONLINE = 0x01             # 反馈计数在最近 100 ms 内有增长
    AT_LIMIT = 0x02           # 目标被工作区间截断
    HOST_TORQUE_VALID = 0x04  # 本关节有未过期的上位机力矩设定
    TORQUE_CLAMPED = 0x08     # 设定值被限幅
    HOST_TARGET_VALID = 0x10  # Host 目标已保存
    HOST_CLOSED_LOOP = 0x20   # 该关节 Host 角度闭环已投入


class HostTargetAction(IntEnum):
    """SET_HOST_TARGET 的 action，同 host.h HOST_TARGET_ACTION_*"""
    SET_OPEN = 0              # 只保存目标，该关节不闭环
    SET_CLOSED = 1            # 保存目标，该关节投入闭环
    STOP_ALL = 2              # 全部关节退出闭环，目标保留


class HostIkAction(IntEnum):
    """SET_IK_TARGET 的 action"""
    SET = 0                   # 进入或更新 HOST_IK 位姿 / ψ
    STOP = 1                  # 退出 HOST_IK，回到 HOST


class PostureFlag(IntFlag):
    """ARM_POSTURE.flags"""
    NONE = 0
    VALID = 0x01              # 臂角 ψ 有效
    SHOULDER_SINGULAR = 0x02  # 最近一次 IK 解处于肩奇异
    WRIST_SINGULAR = 0x04     # 最近一次 IK 解处于腕奇异
    IK_ACTIVE = 0x08          # 当前处于 HOST_IK
    AUTO_PSI = 0x40           # 当前 HOST_IK 由固件自动选择 ψ
    # bit4~5: 最近一次 IK 状态 0 OK / 1 OutOfReach / 2 RefDegenerate


class ExchangeAction(IntEnum):
    """EXCHANGE_COMMAND.action；仅 ArmSim 扩展协议使用。"""
    NONE = 0
    RESET = 1
    RANDOMIZE = 2
    MOVE_TO_POSE = 3
    INSERT = 4
    LIFT = 5
    ROTATE_P = 6
    ROTATE_Q = 7
    CONFIRM = 8
    AUTO_DEMO = 9
    STOP_AUTO = 10
    FREE = 11          # 撤掉驱动，关节可手动拖动；RESET 重新接管


class ExchangePhase(IntEnum):
    """EXCHANGE_STATE.phase；与 ArmSim ExchangeStation::Phase 一一对应。"""
    PARKED = 0
    MOVING_POSE = 1
    READY = 2
    INSERTING = 3
    INSERTED = 4
    LIFTING = 5
    LIFTED = 6
    ROTATING_P = 7
    P_ROTATED = 8
    ROTATING_Q = 9
    Q_READY = 10
    COMPLETE = 11
    RESETTING = 12


class ExchangeStateFlag(IntFlag):
    NONE = 0
    AVAILABLE = 0x01
    MOVING = 0x02
    POSE_VALID = 0x04
    AUTO_DEMO = 0x08
    COMPLETE = 0x10
    Q_READY = 0x20
    FREE = 0x40


class AckResult(IntEnum):
    OK = 0
    OK_CLAMPED = 1
    REJECT_ESTOP = 2
    BAD_PARAM = 3
    BAD_LENGTH = 4
    UNSUPPORTED = 5
    READ_ONLY = 6
    BUSY = 7


class EventCode(IntEnum):
    BOOT = 0x01
    ARM_STATE_CHANGED = 0x02      # value = 新状态
    HOST_LINK_LOST = 0x10
    HOST_LINK_RESTORED = 0x11
    TORQUE_EXPIRED = 0x12         # source = 关节号
    ESTOP_LATCHED = 0x13
    ESTOP_RELEASED = 0x14
    MOTOR_FAULT = 0x20            # source = 关节号, value = MotorStatus
    MOTOR_OFFLINE = 0x21          # source = 关节号
    MOTOR_ONLINE = 0x22
    TASK_OFFLINE = 0x30           # source = SystemTaskId_t
    CAN_BUS_OFF = 0x31            # source = CAN 索引
    PROTO_ERROR = 0x40            # value = 错误类别


class Severity(IntEnum):
    INFO = 0
    WARN = 1
    ERROR = 2
    FATAL = 3


class ParamType(IntEnum):
    U8 = 0
    I8 = 1
    U16 = 2
    I16 = 3
    U32 = 4
    I32 = 5
    F32 = 6


class EstopAction(IntEnum):
    LATCH = 1
    RELEASE = 0
