# 02 上位机 ↔ STM32 串口协议 v1.0

> 对应 README 第 2 项。Python 实现在 `armhost/protocol/`，C 参考实现在 `firmware_ref/`。
> 偏移表由单元测试核对（`tests/test_protocol.py`），示例帧由 `examples/gen_doc_frames.py` 生成。

## 1. 待确认参数（不擅自假定）

| 参数 | 协议里怎么处理 | 建议值 | 依据 |
|---|---|---|---|
| 关节数 | `DEVICE_INFO` / `JOINT_STATE` 现场上报，上位机不写死 | 7 | `ARM_JOINT_COUNT` |
| 链路 / 波特率 | 协议与链路无关 | UART 921600 8N1 | UART7/2/3/10 已配 921600 且没绑解析器 |
| JOINT_STATE 频率 | 参数 0x0001 可改 | UART 200 Hz，USB 500 Hz | 见第 9 节带宽 |
| SYSTEM_STATUS 频率 | 参数 0x0002 | 10 Hz | 诊断量变化慢 |
| 链路超时 | `DEVICE_INFO.link_timeout_ms` | 300 ms | 3 个心跳周期 |
| 力矩设定有效期上限 | `DEVICE_INFO.torque_hold_max_ms` | 500 ms | |
| 力矩硬上限 | 固件编译期常量，协议改不了 | 按电机额定扭矩 | 安全 |

USB CDC：`Docs/USB核复位卡死导致整机死机-排查记录.md` 里的问题已加防御，但长期稳定性没验证，**首选 UART**。

## 2. 编码约定

| 项 | 约定 |
|---|---|
| 字节序 | 全部小端（STM32 原生） |
| 整数 | 定长 u8/i8/u16/i16/u32/i32，有符号用补码 |
| 浮点 | IEEE-754 binary32 小端；NaN/Inf 视为 BAD_PARAM |
| 角度 | deg，(-180, 180]，**关节口径**：已乘 `ArmParam_t.direction` |
| 角速度 | deg/s，关节口径 |
| 力矩 | N·m，关节口径；正方向 = 使关节 +q 增大 |
| 时间戳 | `mcu_time_us` u32，MCU 采样时刻，µs，约 71.6 min 回绕，上位机展开成 64 位。**上位机接收时间不在帧里**，由上位机自己记 `monotonic_ns` |
| C 结构体 | 一律 packed 或逐字段 memcpy，禁止直接发未打包的结构体 |

> ⚠️ 方向待上板确认：固件里 `Feedback_t.omega_rads / torque_nm` 是电机口径，填 JOINT_STATE 时要和角度一样乘 `direction[i]`（见 `firmware_ref/arm_proto_port.c`）。最近提交 "J2/J6 力矩方向取反" 说明这个口径对力矩指令也成立。请确认：给 +τ 时关节角是否增大。

## 3. 帧格式

```
偏移   长度  字段    说明
 0      1   SOF0    0xA5
 1      1   SOF1    0x5A
 2      1   VER     高 4 位 major，低 4 位 minor；v1.0 = 0x10
 3      1   TYPE    消息类型
 4      2   SEQ     u16，发送方每帧 +1（两个方向各自独立），65535 后回 0
 6      1   LEN     负载长度 0..245
 7      1   FLAGS   bit0 ACK_REQ 要求应答；其余保留填 0
 8      1   HCRC    CRC-8 覆盖偏移 0..7
 9     LEN  PAYLOAD
 9+LEN  2   CRC16   CRC-16 覆盖偏移 0..8+LEN，小端
```

* 开销 11 字节，**最大帧长 256**（= 固件 `UART_DMA_BUFFER_SIZE`），最大负载 245。
* HCRC：CRC-8/SMBUS（poly 0x07，init 0x00，不反射），`"123456789"` → 0xF4。
* CRC16：CRC-16/CCITT-FALSE（poly 0x1021，init 0xFFFF，不反射），`"123456789"` → 0x29B1。
* 为什么要单独的 HCRC：LEN 被噪声改大时，没有 HCRC 的解析器会傻等几百字节；有了 HCRC，坏帧头 1 字节内就被识别出来。

## 4. 消息类型

| TYPE | 名称 | 方向 | 频率 / 触发 | 应答 |
|---|---|---|---|---|
| 0x01 | HEARTBEAT | 双向 | 10 Hz | 无 |
| 0x02 | DEVICE_INFO_REQ | H→M | 连接后 | DEVICE_INFO |
| 0x03 | DEVICE_INFO | M→H | 应答 | — |
| 0x04 | ACK | M→H | 应答 | — |
| 0x10 | JOINT_STATE | M→H | 周期，默认 100 Hz | 无 |
| 0x11 | SYSTEM_STATUS | M→H | 周期，默认 10 Hz | 无 |
| 0x12 | EVENT | M→H | 事件触发 | 无 |
| 0x20 | SET_JOINT_TORQUE | H→M | ≥ 20 Hz 持续刷新 | ACK（ACK_REQ 置位时） |
| 0x21 | CLEAR_TORQUE | H→M | 按需 | ACK |
| 0x22 | ESTOP | H→M | 按需 | ACK |
| 0x23 | SET_HOST_TARGET | H→M | 按需 | ACK |
| 0x28 | PARAM_READ | H→M | 按需 | PARAM_VALUE |
| 0x29 | PARAM_WRITE | H→M | 按需 | PARAM_VALUE |
| 0x2A | PARAM_VALUE | M→H | 应答 | — |
| 0x30~0x3F | 预留：轨迹 / 仿真同步 | | | |

## 5. 负载定义（偏移相对负载起点）

### 0x01 HEARTBEAT（8 B）
| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | u32 | uptime_ms | 发送方运行时间 |
| 4 | u32 | rx_frames_ok | 发送方累计收到的有效帧数，对端据此估计己方发出的帧有没有丢 |

### 0x03 DEVICE_INFO（28 B）
| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u8 | proto_version（同 VER） |
| 1 | u8 | joint_count |
| 2 | u8 | joint_record_size（v1.0 = 28） |
| 3 | u8 | arm_role（ArmRole_t） |
| 4 | u16 | max_frame |
| 6 | u16 | state_rate_hz |
| 8 | u16 | status_rate_hz |
| 10 | u16 | link_timeout_ms |
| 12 | u16 | torque_hold_max_ms |
| 14 | u16 | fw_build |
| 16 | char[12] | fw_name，不足补 0 |

### 0x04 ACK（8 B）
| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | u16 | ack_seq | 被应答帧的 SEQ |
| 2 | u8 | ack_type | 被应答帧的 TYPE |
| 3 | u8 | result | 0 OK，1 OK_CLAMPED，2 REJECT_ESTOP，3 BAD_PARAM，4 BAD_LENGTH，5 UNSUPPORTED，6 READ_ONLY，7 BUSY |
| 4 | u32 | detail | SET_JOINT_TORQUE：被限幅关节的位掩码；其余命令为 0 |

### 0x23 SET_HOST_TARGET（8 B，单关节）
| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | u8 | joint | 目标关节 0~6；STOP_ALL 时忽略 |
| 1 | u8 | action | 0 SET_OPEN：保存目标、该关节不闭环；1 SET_CLOSED：保存目标并闭环；2 STOP_ALL：全部关节退出闭环、目标保留 |
| 2 | u16 | reserved | 0 |
| 4 | f32 | target_deg | 该关节单圈绝对目标角，必须位于固件 `ArmJointLimit_t` 内；STOP_ALL 时忽略 |

一帧只更新一个关节，未列出的关节保持原目标和原闭环状态。只有 `arm_state == HOST`（左上 + 右下拨杆）时接受 SET_OPEN / SET_CLOSED。固件会再检查一次限位，越界或状态不符回 `BAD_PARAM`；成功回 `ACK`。目标命令不自动重试，链路超过 300 ms 没有有效帧时全部关节自动退出闭环。`STOP_ALL` 是安全命令，任何状态都接受。

### 0x10 JOINT_STATE（12 + N×28 B，N=7 时 208 B，整帧 219 B）
| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | u32 | mcu_time_us | 本次观测的采样时刻 |
| 4 | u8 | arm_state | ArmState_t：0 PROTECTED 1 ENABLE 2 FEEDBACK 3 MASTER 4 AUTO 5 SERVANT 6 HOST |
| 5 | u8 | joint_count | N |
| 6 | u8 | record_size | 每条关节记录的字节数，v1.0 = 28；接收方按它步进，兼容以后加长 |
| 7 | u8 | reserved | 0 |
| 8 | u16 | status_flags | bit0 RC_ONLINE，bit1 HOST_LINK_OK，bit2 ESTOP_LATCHED，bit3 HOST_TORQUE_ACTIVE，bit4 CTRL_ALIGN_PENDING，bit5 HOST_CLOSED_LOOP |
| 10 | u16 | host_cmd_age_ms | 距上一条有效上位机控制命令（SET_JOINT_TORQUE / SET_HOST_TARGET）的时间，饱和到 65535（=从未收到） |
| 12 | rec[N] | joints | |

关节记录（28 B），对应固件字段：
| 偏移 | 类型 | 字段 | 固件来源 |
|---|---|---|---|
| 0 | f32 | angle_deg | `observer.joint_deg[i]` |
| 4 | f32 | velocity_dps | `feedback.omega_rads × 57.2958 × direction[i]` |
| 8 | f32 | torque_nm | `feedback.torque_nm × direction[i]` |
| 12 | f32 | target_deg | `joint_ctrl[i].target_deg` |
| 16 | f32 | torque_cmd_nm | 本周期实际写进电机指令的力矩（关节口径） |
| 20 | u16 | encoder | `feedback.encoder` |
| 22 | u8 | motor_status | MotorStatus_t（0 DISABLED … 10 NO_FEEDBACK） |
| 23 | u8 | error_id | `feedback.error_id` |
| 24 | i8 | temperature_c | `feedback.temp` |
| 25 | u8 | joint_flags | bit0 ONLINE，bit1 AT_LIMIT，bit2 HOST_TORQUE_VALID，bit3 TORQUE_CLAMPED，bit4 HOST_TARGET_VALID，bit5 HOST_CLOSED_LOOP（本关节闭环） |
| 26 | u16 | feedback_count | `feedback.timestamp` 低 16 位，不增长说明电机没回包 |

### 0x11 SYSTEM_STATUS（32 + 2T + 12C B）
| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | u32 | mcu_time_us | |
| 4 | u32 | uptime_ms | |
| 8 | u8 | arm_state | |
| 9 | u8 | arm_role | |
| 10 | u8 | rc_switch_left | DT7：1 上，3 中，2 下，0 无效 |
| 11 | u8 | rc_switch_right | 同上 |
| 12 | u16 | status_flags | 同 JOINT_STATE |
| 14 | u8 | task_count | T，= `SYSTEM_TASK_COUNT` |
| 15 | u8 | can_count | C |
| 16 | u32 | link_rx_ok | 固件收到的有效帧 |
| 20 | u32 | link_rx_crc_err | |
| 24 | u32 | link_rx_other_err | 帧头 / 长度 / 版本错误 |
| 28 | u32 | link_tx_drop | 固件因 DMA 忙丢掉的上报帧 |
| 32 | u16[T] | task_offline_count | `Monitor_t.task[i].offline_count` |
| 32+2T | can[C] | 每路 12 B：tx_ok u32，tx_fail u32，tx_err_cnt u8，rx_err_cnt u8，bus_flags u8（bit0 bus_off，bit1 error_passive，bit2 warning），保留 u8 | `CANTxMonitor` |

### 0x12 EVENT（12 B）
| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u32 | mcu_time_us |
| 4 | u16 | code |
| 6 | u8 | severity：0 INFO，1 WARN，2 ERROR，3 FATAL |
| 7 | u8 | source：关节号 / 任务号 / CAN 号 |
| 8 | i32 | value |

| code | 含义 | source | value |
|---|---|---|---|
| 0x01 | BOOT | 0 | 复位原因 |
| 0x02 | ARM_STATE_CHANGED | 0 | 新状态 |
| 0x10 / 0x11 | HOST_LINK_LOST / RESTORED | 0 | 0 |
| 0x12 | TORQUE_EXPIRED | 关节号 | 0 |
| 0x13 / 0x14 | ESTOP_LATCHED / RELEASED | 0 | 0 |
| 0x20 | MOTOR_FAULT | 关节号 | MotorStatus_t |
| 0x21 / 0x22 | MOTOR_OFFLINE / ONLINE | 关节号 | 0 |
| 0x30 | TASK_OFFLINE | SystemTaskId_t | 0 |
| 0x31 | CAN_BUS_OFF | CAN 号 | 0 |
| 0x40 | PROTO_ERROR | 0 | 错误类别 |

### 0x20 SET_JOINT_TORQUE（4 + 5K B）
| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | u16 | hold_ms | 本条设定的有效期；0 = 固件默认 100 ms；超过 torque_hold_max_ms 截断 |
| 2 | u8 | count | K，1..N |
| 3 | u8 | reserved | 0 |
| 4+5k | u8 | joint | 关节号 |
| 5+5k | f32 | torque_nm | 关节口径 |

没列出的关节保持各自原有的设定和有效期。**只是"设定"**：是否真的经 CAN 发出去由固件状态机决定（第 7 节）。

### 0x21 CLEAR_TORQUE（4 B）：`u32 joint_mask`，第 i 位为 1 清掉关节 i 的设定，0xFFFFFFFF 全清。
### 0x22 ESTOP（1 B）：`u8 action`，1 = 锁存急停，0 = 解除。

### 0x28 PARAM_READ（4 B）：`u16 param_id | u8 index | u8 reserved`
### 0x29 PARAM_WRITE（8 B）：`u16 param_id | u8 index | u8 type | 4B value`
### 0x2A PARAM_VALUE（12 B）
| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u16 | req_seq：所应答请求的 SEQ |
| 2 | u16 | param_id |
| 4 | u8 | index |
| 5 | u8 | type：0 U8，1 I8，2 U16，3 I16，4 U32，5 I32，6 F32 |
| 6 | u8 | result（同 ACK） |
| 7 | u8 | access：bit0 可读，bit1 可写 |
| 8 | 4B | value：按 type 解释，不足 4 字节时放低位、高位补 0 |

## 6. 参数表

| id | 名称 | 类型 | 个数 | 读写 | 说明 |
|---|---|---|---|---|---|
| 0x0001 | state_rate_hz | U16 | 1 | RW | 1~500 |
| 0x0002 | status_rate_hz | U16 | 1 | RW | 1~50 |
| 0x0003 | link_timeout_ms | U16 | 1 | R | |
| 0x0004 | torque_hold_max_ms | U16 | 1 | R | |
| 0x0100 | zero_point | U16 | 7 | R | `ArmParam_t.zero_point` |
| 0x0101 | direction | I8 | 7 | R | |
| 0x0102 / 0x0103 | limit_min_deg / limit_max_deg | F32 | 7 | R | `ArmJointLimit_t`，上位机只读 |
| 0x0200~0x0205 | gain_kp / ki / kd / d_alpha / sum_error_max / output_max | F32 | 7 | R | `ArmJointGain_t`，上位机只读，调参在固件里改 |
| 0x0300 | host_torque_limit_nm | F32 | 7 | R | 上位机力矩限幅，固件编译期常量，只读；**填 0 即禁止上位机出力** |
| 0x0400 | dh | F32 | 28 | R | index = 行×4 + 列（alpha_deg, a_m, d_m, theta_offset_deg） |

**目前上位机只有 `state_rate_hz`、`status_rate_hz` 两个参数可写**；PID 增益、关节限位、上位机力矩限幅一律只读，对它们发 PARAM_WRITE 固件回 `READ_ONLY`（6）。可写参数只改 RAM，掉电后恢复编译期值。

## 7. 安全机制（固件必须实现）

1. **力矩只是设定**：固件存 `host_tau[i]` 和 `deadline[i]`。只有 `arm_state == ENABLE`、遥控允许、没急停、没过期时，固件才把它用到电机指令上（替代还是叠加 PID 输出，**待你确定**，参考实现是替代）。
2. **有效期**：每个关节的设定到 deadline 自动清零，并发 `TORQUE_EXPIRED`。上位机要持续以 ≥ 20 Hz 刷新。
3. **链路超时**：超过 `link_timeout_ms` 没收到任何有效帧（包括心跳），清掉全部设定，发 `HOST_LINK_LOST`。所以串口断开后最多 300 ms 就不会再有未经确认的力矩。
4. **限幅**：`|τ| ≤ min(host_torque_limit_nm[i], 编译期硬上限[i])`，超限截断，ACK 返回 OK_CLAMPED 并给出被截断关节的掩码。限幅值是固件编译期常量，上位机不能改；**填 0 则上位机永远出不了力**，要用上位机力矩必须先在固件里给非零值。
5. **急停**：ESTOP(1) 清全部设定并锁存，锁存期间 SET_JOINT_TORQUE 一律回 REJECT_ESTOP；只有 ESTOP(0) 能解除，链路恢复也不会自动解除。遥控的保护档仍然最高优先。
6. **上位机侧**：断线时所有等待应答的命令立刻失败；力矩命令**不重试**、不缓存，重连后不补发。上位机退出前发 ESTOP(1) + CLEAR_TORQUE。
7. **Host 角度目标**：只在 `ARM_HOST` 状态接受，绝对目标逐轴检查 `ArmJointLimit_t`，再映射到当前多圈。`closed_loop=0` 时保持零力矩；闭环投入后若 300 ms 没收到任何有效帧，固件自动退出闭环并清 PID。

## 8. 错误处理

| 情况 | 接收方动作 |
|---|---|
| 粘包 / 一次收到多帧 | 循环解析，直到缓冲里不够一帧 |
| 半包 | 保留，等下一块数据 |
| 丢字节 / 垃圾字节 | 往后找 A5 5A，跳过的字节计入 discarded |
| HCRC 错 | 计 header_error，丢 1 字节重新找同步 |
| LEN > 245 | 计 length_error，丢 1 字节 |
| CRC16 错 | 计 crc_error，**只丢 1 字节**：真正的帧头可能藏在坏帧里 |
| major 版本不同 | 计 version_error，按 LEN 整帧跳过 |
| 半帧 50 ms 没收齐 | 丢弃半帧，计 timeout_drop |
| 负载比定义短 | 回 BAD_LENGTH（命令）或丢弃（上报） |
| SEQ 判断 | `d = (seq − last) mod 65536`：d=1 正常；2 ≤ d < 32768 丢了 d−1 帧；d=0 重复；d ≥ 32768 视为对端重启，重新同步 |
| 重复命令 | 固件缓存最近若干个 SEQ 的应答，SEQ 重复时**只重发应答、不重复执行** |
| 应答超时 | 上位机 150 ms 没收到应答：幂等命令（PARAM_*、ESTOP、CLEAR_TORQUE、DEVICE_INFO_REQ）用同一 SEQ 重发 2 次；SET_JOINT_TORQUE 不重发 |

## 9. 带宽与速率建议

JOINT_STATE（7 关节）整帧 219 B。UART 8N1 每字节 10 bit：

| 链路 | 有效吞吐 | 200 Hz | 500 Hz |
|---|---|---|---|
| 921600 | 92.1 kB/s | 43.8 kB/s（48%） | 109.5 kB/s ✗ |
| 2 Mbps | 200 kB/s | 22% | 55% |
| USB FS CDC | ≥ 500 kB/s | 9% | 22% |

建议：UART 921600 跑 200 Hz，余量留给 SYSTEM_STATUS（10 Hz × 约 70 B）、EVENT 和应答；要 500 Hz（和 2 ms 控制周期同步）就换 USB 或 2 Mbps。固件的发送任务周期是 2 ms，200 Hz 即每 5 个周期发一次。

## 10. 版本兼容

* major 不同：直接丢帧。上位机连上后先发 DEVICE_INFO_REQ，major 不一致时提示升级。
* 同一 major 内只允许在**负载末尾**或**关节记录末尾**追加字段；接收方忽略多出的尾部，负载比定义短才报错。JOINT_STATE 按 `record_size` 步进，所以加长记录不影响旧的上位机。
* 新消息类型：不认识的 TYPE 原样保留（RawMessage）、不报错；固件收到不认识的命令回 UNSUPPORTED。

## 11. 示例帧（`examples/gen_doc_frames.py` 生成）

**HEARTBEAT**，SEQ=1，uptime=1000 ms，rx_frames_ok=42（19 B）
```
A5 5A 10 01 01 00 08 00 09 E8 03 00 00 2A 00 00 00 1C 5F
│  │  │  │  └─┬─┘ │  │  │  └────┬────┘ └────┬────┘ └─┬─┘
│  │  │  │  SEQ=1 │  │  HCRC  uptime=1000 rx_ok=42  CRC16=0x5F1C
SOF0 SOF1 VER TYPE LEN=8 FLAGS=0
```

**SET_JOINT_TORQUE**，SEQ=0x0102，ACK_REQ，hold=100 ms，J0=+1.5，J6=−0.5（25 B）
```
A5 5A 10 20 02 01 0E 01 27 | 64 00 | 02 | 00 | 00 00 00 C0 3F | 06 00 00 00 BF | 88 DF
帧头: TYPE=0x20 SEQ=0x0102 LEN=14 FLAGS=ACK_REQ HCRC=0x27
负载: hold_ms=100, count=2, rsv, {J0, 1.5f=0x3FC00000}, {J6, -0.5f=0xBF000000}; CRC16=0xDF88
```

**ACK**（应答上一条，OK）（19 B）
```
A5 5A 10 04 00 02 08 00 24 | 02 01 | 20 | 00 | 00 00 00 00 | D5 88
ack_seq=0x0102, ack_type=0x20, result=OK, detail=0
```

**ESTOP 锁存**（12 B）：`A5 5A 10 22 03 01 01 01 36 01 49 B4`

**PARAM_READ** gain_kp[J2]（15 B）：`A5 5A 10 28 04 01 04 01 C8 00 02 02 00 EE 47`（param_id=0x0200，index=2）

**JOINT_STATE**（只有 1 个关节，演示格式），SEQ=0xFFFF（回绕前最后一个）（51 B）
```
A5 5A 10 10 FF FF 28 00 1B
40 42 0F 00          mcu_time_us = 1 000 000
01 01 1C 00          arm_state=ENABLE, joint_count=1, record_size=28, rsv
03 00 14 00          status_flags=RC_ONLINE|HOST_LINK_OK, host_cmd_age_ms=20
00 00 48 41          angle_deg = 12.5
00 00 40 C0          velocity_dps = -3.0
00 00 80 3E          torque_nm = 0.25
00 00 40 41          target_deg = 12.0
9A 99 99 3E          torque_cmd_nm = 0.3
34 12 01 00 24 01    encoder=0x1234, status=ENABLED, error_id=0, temp=36, flags=ONLINE
07 00                feedback_count = 7
79 D1                CRC16
```

## 12. 实现

* Python：编解码在 `armhost/protocol/`，单元测试在 `tests/test_protocol.py`。
* STM32 HAL C：打包、流式解包、CRC 的参考实现在 `firmware_ref/arm_proto.{h,c}`；实际固件收发在 `Real/DIYcontroller/LibSE/Module/Host/host.{h,c}`，UART 中断只校验入队，Arm 写入、应答和上报都在发送任务执行。
