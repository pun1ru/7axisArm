/* 上位机串口协议 v1.0 —— STM32 端参考实现
 * 与 Host/armhost/protocol 逐字节一致，改动任何定义都要同步 Python 端与 docs/02_protocol.md。
 * 只依赖 <stdint.h> <string.h>，不依赖 HAL / FreeRTOS，可以直接在 PC 上编译做单测。
 */
#ifndef ARM_PROTO_H
#define ARM_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------帧--------------------------------------*/
#define ARM_PROTO_SOF0            0xA5U
#define ARM_PROTO_SOF1            0x5AU
#define ARM_PROTO_VERSION         0x10U     /* 高 4 位 major，低 4 位 minor */
#define ARM_PROTO_MAJOR           (ARM_PROTO_VERSION >> 4)
#define ARM_PROTO_HEADER_SIZE     9U        /* 含 HCRC */
#define ARM_PROTO_TRAILER_SIZE    2U
#define ARM_PROTO_OVERHEAD        (ARM_PROTO_HEADER_SIZE + ARM_PROTO_TRAILER_SIZE)
#define ARM_PROTO_MAX_FRAME       256U      /* = UART_DMA_BUFFER_SIZE */
#define ARM_PROTO_MAX_PAYLOAD     (ARM_PROTO_MAX_FRAME - ARM_PROTO_OVERHEAD)
#define ARM_PROTO_FLAG_ACK_REQ    0x01U

/*--------------------------------------消息类型--------------------------------------*/
enum {
    ARM_MSG_HEARTBEAT        = 0x01,
    ARM_MSG_DEVICE_INFO_REQ  = 0x02,
    ARM_MSG_DEVICE_INFO      = 0x03,
    ARM_MSG_ACK              = 0x04,
    ARM_MSG_JOINT_STATE      = 0x10,
    ARM_MSG_SYSTEM_STATUS    = 0x11,
    ARM_MSG_EVENT            = 0x12,
    ARM_MSG_SET_JOINT_TORQUE = 0x20,
    ARM_MSG_CLEAR_TORQUE     = 0x21,
    ARM_MSG_ESTOP            = 0x22,
    ARM_MSG_PARAM_READ       = 0x28,
    ARM_MSG_PARAM_WRITE      = 0x29,
    ARM_MSG_PARAM_VALUE      = 0x2A,
};

enum {
    ARM_ACK_OK = 0, ARM_ACK_OK_CLAMPED, ARM_ACK_REJECT_ESTOP, ARM_ACK_BAD_PARAM,
    ARM_ACK_BAD_LENGTH, ARM_ACK_UNSUPPORTED, ARM_ACK_READ_ONLY, ARM_ACK_BUSY,
};

/* status_flags */
#define ARM_SF_RC_ONLINE           0x0001U
#define ARM_SF_HOST_LINK_OK        0x0002U
#define ARM_SF_ESTOP_LATCHED       0x0004U
#define ARM_SF_HOST_TORQUE_ACTIVE  0x0008U
#define ARM_SF_CTRL_ALIGN_PENDING  0x0010U
/* joint_flags */
#define ARM_JF_ONLINE              0x01U
#define ARM_JF_AT_LIMIT            0x02U
#define ARM_JF_HOST_TORQUE_VALID   0x04U
#define ARM_JF_TORQUE_CLAMPED      0x08U

enum {
    ARM_EV_BOOT = 0x01, ARM_EV_ARM_STATE_CHANGED = 0x02,
    ARM_EV_HOST_LINK_LOST = 0x10, ARM_EV_HOST_LINK_RESTORED = 0x11, ARM_EV_TORQUE_EXPIRED = 0x12,
    ARM_EV_ESTOP_LATCHED = 0x13, ARM_EV_ESTOP_RELEASED = 0x14,
    ARM_EV_MOTOR_FAULT = 0x20, ARM_EV_MOTOR_OFFLINE = 0x21, ARM_EV_MOTOR_ONLINE = 0x22,
    ARM_EV_TASK_OFFLINE = 0x30, ARM_EV_CAN_BUS_OFF = 0x31, ARM_EV_PROTO_ERROR = 0x40,
};

enum { ARM_PT_U8 = 0, ARM_PT_I8, ARM_PT_U16, ARM_PT_I16, ARM_PT_U32, ARM_PT_I32, ARM_PT_F32 };

/* 参数 id，与 Host/armhost/protocol/params.py 一一对应 */
#define ARM_PARAM_STATE_RATE_HZ        0x0001U
#define ARM_PARAM_STATUS_RATE_HZ       0x0002U
#define ARM_PARAM_LINK_TIMEOUT_MS      0x0003U
#define ARM_PARAM_TORQUE_HOLD_MAX_MS   0x0004U
#define ARM_PARAM_ZERO_POINT           0x0100U
#define ARM_PARAM_DIRECTION            0x0101U
#define ARM_PARAM_LIMIT_MIN_DEG        0x0102U
#define ARM_PARAM_LIMIT_MAX_DEG        0x0103U
#define ARM_PARAM_GAIN_KP              0x0200U   /* 0x0200..0x0205 依次 kp ki kd d_alpha sum_error_max output_max，上位机只读 */
#define ARM_PARAM_GAIN_OUTPUT_MAX      0x0205U
#define ARM_PARAM_HOST_TORQUE_LIMIT    0x0300U   /* 只读 */
#define ARM_PARAM_DH                   0x0400U

/*--------------------------------------负载结构（packed，小端）--------------------------------------*/
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
#define ARM_PACKED __attribute__((packed))
#else
#define ARM_PACKED
#pragma pack(push, 1)
#endif

typedef struct ARM_PACKED { uint32_t uptime_ms; uint32_t rx_frames_ok; } ArmMsgHeartbeat_t;

typedef struct ARM_PACKED {
    uint8_t proto_version, joint_count, joint_record_size, arm_role;
    uint16_t max_frame, state_rate_hz, status_rate_hz, link_timeout_ms, torque_hold_max_ms, fw_build;
    char fw_name[12];
} ArmMsgDeviceInfo_t;

typedef struct ARM_PACKED {
    uint16_t ack_seq; uint8_t ack_type; uint8_t result; uint32_t detail;
} ArmMsgAck_t;

typedef struct ARM_PACKED {
    float angle_deg, velocity_dps, torque_nm, target_deg, torque_cmd_nm;
    uint16_t encoder;
    uint8_t motor_status, error_id;
    int8_t temperature_c;
    uint8_t joint_flags;
    uint16_t feedback_count;
} ArmJointRecord_t;

typedef struct ARM_PACKED {
    uint32_t mcu_time_us;
    uint8_t arm_state, joint_count, record_size, reserved;
    uint16_t status_flags, host_cmd_age_ms;
    /* 后跟 ArmJointRecord_t[joint_count] */
} ArmMsgJointStateHead_t;

typedef struct ARM_PACKED {
    uint32_t mcu_time_us, uptime_ms;
    uint8_t arm_state, arm_role, rc_switch_left, rc_switch_right;
    uint16_t status_flags;
    uint8_t task_count, can_count;
    uint32_t link_rx_ok, link_rx_crc_err, link_rx_other_err, link_tx_drop;
    /* 后跟 uint16_t task_offline[task_count]，再跟 ArmCanStatus_t[can_count] */
} ArmMsgSystemStatusHead_t;

typedef struct ARM_PACKED {
    uint32_t tx_ok, tx_fail; uint8_t tx_err_cnt, rx_err_cnt, bus_flags, reserved;
} ArmCanStatus_t;

typedef struct ARM_PACKED {
    uint32_t mcu_time_us; uint16_t code; uint8_t severity, source; int32_t value;
} ArmMsgEvent_t;

typedef struct ARM_PACKED { uint16_t hold_ms; uint8_t count, reserved; } ArmMsgSetTorqueHead_t;
typedef struct ARM_PACKED { uint8_t joint; float torque_nm; } ArmTorqueItem_t;
typedef struct ARM_PACKED { uint32_t joint_mask; } ArmMsgClearTorque_t;
typedef struct ARM_PACKED { uint8_t action; } ArmMsgEstop_t;
typedef struct ARM_PACKED { uint16_t param_id; uint8_t index, reserved; } ArmMsgParamRead_t;
typedef struct ARM_PACKED { uint16_t param_id; uint8_t index, type; uint8_t value[4]; } ArmMsgParamWrite_t;
typedef struct ARM_PACKED {
    uint16_t req_seq, param_id; uint8_t index, type, result, access; uint8_t value[4];
} ArmMsgParamValue_t;

#if !(defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__))
#pragma pack(pop)
#endif

/* 编译期检查尺寸，任何一个不对说明打包失效 */
#define ARM_STATIC_ASSERT(c, n) typedef char arm_static_assert_##n[(c) ? 1 : -1]
ARM_STATIC_ASSERT(sizeof(ArmMsgHeartbeat_t) == 8, hb);
ARM_STATIC_ASSERT(sizeof(ArmMsgDeviceInfo_t) == 28, info);
ARM_STATIC_ASSERT(sizeof(ArmMsgAck_t) == 8, ack);
ARM_STATIC_ASSERT(sizeof(ArmJointRecord_t) == 28, jrec);
ARM_STATIC_ASSERT(sizeof(ArmMsgJointStateHead_t) == 12, jhead);
ARM_STATIC_ASSERT(sizeof(ArmMsgSystemStatusHead_t) == 32, shead);
ARM_STATIC_ASSERT(sizeof(ArmCanStatus_t) == 12, can);
ARM_STATIC_ASSERT(sizeof(ArmMsgEvent_t) == 12, ev);
ARM_STATIC_ASSERT(sizeof(ArmTorqueItem_t) == 5, titem);
ARM_STATIC_ASSERT(sizeof(ArmMsgParamValue_t) == 12, pval);

/*--------------------------------------CRC--------------------------------------*/
uint8_t ArmProto_Crc8(const uint8_t *data, uint16_t len);
uint16_t ArmProto_Crc16(const uint8_t *data, uint16_t len);

/*--------------------------------------打包--------------------------------------*/
/* 把负载打成整帧写进 out，返回帧长；out_size 不够或负载过长返回 0 */
uint16_t ArmProto_Pack(uint8_t *out, uint16_t out_size, uint8_t type, uint16_t seq,
                       uint8_t flags, const void *payload, uint16_t len);

/*--------------------------------------流式解包--------------------------------------*/
typedef struct {
    uint32_t frames_ok, bytes_discarded, header_errors, length_errors, crc_errors, version_errors;
} ArmProtoParserStats_t;

typedef void (*ArmProtoFrameHandler_t)(void *context, uint8_t type, uint16_t seq, uint8_t flags,
                                       const uint8_t *payload, uint16_t len);

typedef struct {
    uint8_t buf[ARM_PROTO_MAX_FRAME * 2U];
    uint16_t len;
    ArmProtoFrameHandler_t on_frame;
    void *context;
    ArmProtoParserStats_t stats;
} ArmProtoParser_t;

void ArmProto_ParserInit(ArmProtoParser_t *p, ArmProtoFrameHandler_t on_frame, void *context);
/* 喂任意长度的字节（可以是半帧、多帧、垃圾）；完整且校验正确的帧经 on_frame 回调 */
void ArmProto_ParserFeed(ArmProtoParser_t *p, const uint8_t *data, uint16_t len);
/* 串口出错或半帧超时时调用，丢掉缓冲 */
void ArmProto_ParserReset(ArmProtoParser_t *p);

#ifdef __cplusplus
}
#endif

#endif
