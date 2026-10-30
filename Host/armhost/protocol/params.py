"""参数表。param_id 与固件 arm_proto.h 的 ARM_PARAM_* 宏一一对应。

只把调试时确实会读写的量放进来；数组参数用 index 选元素（关节号 / DH 行号*4+列）。
写参数只改 RAM，掉电恢复 module_init.c 里的编译期值；是否落盘由后续协议版本决定。
"""

from __future__ import annotations

from dataclasses import dataclass

from .enums import ParamType

R, W = 0x01, 0x02


@dataclass(frozen=True, slots=True)
class ParamDef:
    pid: int
    name: str
    ptype: ParamType
    count: int          # 元素个数，标量为 1
    access: int
    unit: str
    note: str


PARAMS: tuple[ParamDef, ...] = (
    # 0x00xx 系统 / 协议
    ParamDef(0x0001, "state_rate_hz", ParamType.U16, 1, R | W, "Hz", "JOINT_STATE 上报频率，固件限 1~500"),
    ParamDef(0x0002, "status_rate_hz", ParamType.U16, 1, R | W, "Hz", "SYSTEM_STATUS 上报频率，固件限 1~50"),
    ParamDef(0x0003, "link_timeout_ms", ParamType.U16, 1, R, "ms", "链路超时，编译期常量"),
    ParamDef(0x0004, "torque_hold_max_ms", ParamType.U16, 1, R, "ms", "力矩设定最长有效期"),
    # 0x01xx 关节标定（arm.h ArmParam_t）
    ParamDef(0x0100, "zero_point", ParamType.U16, 7, R, "raw", "ArmParam_t.zero_point[i]"),
    ParamDef(0x0101, "direction", ParamType.I8, 7, R, "", "ArmParam_t.direction[i]"),
    ParamDef(0x0102, "limit_min_deg", ParamType.F32, 7, R, "deg", "ArmJointLimit_t.min_deg，只读"),
    ParamDef(0x0103, "limit_max_deg", ParamType.F32, 7, R, "deg", "ArmJointLimit_t.max_deg，只读"),
    # 0x02xx 角度环增益（ArmJointGain_t）：上位机只读，调参在固件里改
    ParamDef(0x0200, "gain_kp", ParamType.F32, 7, R, "N*m/deg", ""),
    ParamDef(0x0201, "gain_ki", ParamType.F32, 7, R, "N*m/deg", ""),
    ParamDef(0x0202, "gain_kd", ParamType.F32, 7, R, "N*m/deg", "每周期增量口径"),
    ParamDef(0x0203, "gain_d_alpha", ParamType.F32, 7, R, "", "[0,1)"),
    ParamDef(0x0204, "gain_sum_error_max", ParamType.F32, 7, R, "", ""),
    ParamDef(0x0205, "gain_output_max", ParamType.F32, 7, R, "N*m", "PID 力矩限幅"),
    # 0x03xx 上位机力矩设定
    ParamDef(0x0300, "host_torque_limit_nm", ParamType.F32, 7, R, "N*m",
             "上位机力矩设定限幅，只读；固件编译期常量，设 0 即禁止上位机出力"),
    # 0x04xx DH（只读，供三维显示）：index = 行号*4 + 列(0 alpha_deg,1 a_m,2 d_m,3 theta_offset_deg)
    ParamDef(0x0400, "dh", ParamType.F32, 28, R, "deg/m", "改进 DH 表"),
)

BY_ID = {p.pid: p for p in PARAMS}
BY_NAME = {p.name: p for p in PARAMS}
