#include "host.h"

#include <string.h>

#define HOST_RAD_TO_DEG         57.29577951f
#define HOST_REC_SIZE           28U     /* 每关节一条记录 */
#define HOST_STATE_HEAD_SIZE    12U
#define HOST_STATE_SIZE         (HOST_STATE_HEAD_SIZE + ARM_JOINT_COUNT * HOST_REC_SIZE)
#define HOST_TASK_PERIOD_MS     2U
#define HOST_ENCODER_CPR        65536.0f
#define HOST_MAX_REQUEST        32U

#define HOST_ACK_OK             0U
#define HOST_ACK_BAD_PARAM      3U
#define HOST_ACK_BAD_LENGTH     4U
#define HOST_ACK_UNSUPPORTED    5U
#define HOST_FLAG_ACK_REQ       0x01U

#define HOST_ACTION_SET_OPEN    0U      /* 保存目标，该关节不闭环 */
#define HOST_ACTION_SET_CLOSED  1U      /* 保存目标，该关节投入闭环 */
#define HOST_ACTION_STOP_ALL    2U      /* 全部关节退出闭环，目标保留 */

/* PARAM_READ 的参数号；类型：U16=2 I8=1 F32=6 */
#define HOST_PARAM_STATE_RATE_HZ    0x0001U
#define HOST_PARAM_LINK_TIMEOUT     0x0003U
#define HOST_PARAM_ZERO_POINT       0x0100U
#define HOST_PARAM_DIRECTION        0x0101U
#define HOST_PARAM_LIMIT_MIN        0x0102U
#define HOST_PARAM_LIMIT_MAX        0x0103U
#define HOST_PARAM_GAIN_KP          0x0200U
#define HOST_PARAM_GAIN_KI          0x0201U
#define HOST_PARAM_GAIN_KD          0x0202U
#define HOST_PARAM_GAIN_D_ALPHA     0x0203U
#define HOST_PARAM_GAIN_SUM_MAX     0x0204U
#define HOST_PARAM_GAIN_OUTPUT_MAX  0x0205U
#define HOST_PARAM_DH               0x0400U

/* status_flags / joint_flags */
#define HOST_FLAG_LINK_OK           0x0002U
#define HOST_FLAG_CTRL_ALIGNING     0x0010U
#define HOST_FLAG_HOST_CLOSED_LOOP  0x0020U
#define HOST_JOINT_ONLINE           0x01U
#define HOST_JOINT_TARGET_VALID     0x10U
#define HOST_JOINT_CLOSED_LOOP      0x20U

uint8_t Host_Crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00U;

    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t Host_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) != 0U ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/*--------------------------------------字节打包（小端）--------------------------------------*/
static void PutU16(uint8_t *out, uint16_t v)
{
    out[0] = (uint8_t)(v & 0xFFU);
    out[1] = (uint8_t)(v >> 8);
}

static void PutU32(uint8_t *out, uint32_t v)
{
    PutU16(out, (uint16_t)(v & 0xFFFFU));
    PutU16(out + 2, (uint16_t)(v >> 16));
}

static void PutFloat(uint8_t *out, float v)
{
    uint32_t bits;

    memcpy(&bits, &v, sizeof(bits));
    PutU32(out, bits);
}

static uint16_t GetU16(const uint8_t *in)
{
    return (uint16_t)(in[0] | (in[1] << 8));
}

static float GetFloat(const uint8_t *in)
{
    const uint32_t bits = (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
                          ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
    float v;

    memcpy(&v, &bits, sizeof(v));
    return v;
}

/*--------------------------------------发送--------------------------------------*/
static void SendFrame(Host_t *host, uint8_t type, const uint8_t *payload, uint8_t length)
{
    uint8_t frame[HOST_FRAME_MAX_SIZE];
    uint16_t crc16;

    frame[0] = HOST_FRAME_SOF0;
    frame[1] = HOST_FRAME_SOF1;
    frame[2] = HOST_FRAME_VERSION;
    frame[3] = type;
    PutU16(frame + 4, host->sequence++);
    frame[6] = length;
    frame[7] = 0U;
    frame[8] = Host_Crc8(frame, 8U);
    memcpy(frame + HOST_FRAME_HEADER_SIZE, payload, length);
    crc16 = Host_Crc16(frame, (uint16_t)(HOST_FRAME_HEADER_SIZE + length));
    PutU16(frame + HOST_FRAME_HEADER_SIZE + length, crc16);
    (void)HostLink_Write(host->config.link, frame, HOST_FRAME_OVERHEAD + length);
}

/* 应答并缓存：上位机重发同一个 (type, seq) 时直接重放，不重复执行 */
static void Reply(Host_t *host, uint8_t request_type, uint16_t seq, uint8_t reply_type,
                  const uint8_t *payload, uint8_t length)
{
    HostReply_t *entry = &host->replies[host->reply_head];

    entry->seq = seq;
    entry->request_type = request_type;
    entry->reply_type = reply_type;
    entry->length = length;
    memcpy(entry->payload, payload, length);
    host->reply_head = (uint8_t)((host->reply_head + 1U) % 8U);
    if (host->reply_count < 8U)
    {
        host->reply_count++;
    }
    SendFrame(host, reply_type, payload, length);
}

static void Ack(Host_t *host, uint8_t flags, uint16_t seq, uint8_t type, uint8_t result)
{
    uint8_t payload[8] = {};

    if ((flags & HOST_FLAG_ACK_REQ) == 0U)
    {
        return;
    }
    PutU16(payload, seq);
    payload[2] = type;
    payload[3] = result;
    Reply(host, type, seq, HOST_MSG_ACK, payload, sizeof(payload));
}

/*--------------------------------------JOINT_STATE 上报--------------------------------------*/
static uint16_t StateRateHz(const Host_t *host)
{
    return (uint16_t)(1000U / (HOST_TASK_PERIOD_MS * host->config.state_divider));
}

static bool LinkOk(const Host_t *host, uint32_t now_ms)
{
    return host->ever_rx != 0U && (uint32_t)(now_ms - host->last_rx_ms) <= HOST_LINK_TIMEOUT_MS;
}

static void PackJoint(const Arm_t *arm, uint8_t joint, uint8_t *out)
{
    const Feedback_t *fb = &arm->config.motor.joint[joint]->feedback;
    const float direction = arm->config.param.direction[joint] < 0 ? -1.0f : 1.0f;
    const uint8_t bit = (uint8_t)(1U << joint);
    uint8_t flags = fb->status != MOTOR_NO_FEEDBACK ? HOST_JOINT_ONLINE : 0U;

    if (arm->state == ARM_HOST && (arm->host_target_valid_mask & bit) != 0U)
    {
        flags |= HOST_JOINT_TARGET_VALID;
    }
    if (arm->state == ARM_HOST && (arm->host_closed_loop_mask & bit) != 0U)
    {
        flags |= HOST_JOINT_CLOSED_LOOP;
    }

    PutFloat(out + 0, arm->observer.joint_deg[joint]);
    PutFloat(out + 4, fb->omega_rads * HOST_RAD_TO_DEG * direction);
    PutFloat(out + 8, fb->torque_nm * direction);
    PutFloat(out + 12, arm->joint_ctrl[joint].target_deg);
    PutFloat(out + 16, arm->joint_ctrl[joint].output);
    PutU16(out + 20, fb->encoder);
    out[22] = (uint8_t)fb->status;
    out[23] = fb->error_id;
    out[24] = (uint8_t)(int8_t)fb->temp;
    out[25] = flags;
    PutU16(out + 26, (uint16_t)(fb->timestamp & 0xFFFFU));
}

static void SendJointState(Host_t *host, double sim_time_s, uint32_t now_ms)
{
    const Arm_t *arm = host->arm;
    uint8_t payload[HOST_STATE_SIZE] = {};
    uint16_t flags = 0U;
    uint16_t age = 0xFFFFU;   /* 还没收到过有效命令 */

    if (LinkOk(host, now_ms))
    {
        flags |= HOST_FLAG_LINK_OK;
    }
    if (arm->ctrl_align_pending != 0U || arm->host_align_pending != 0U)
    {
        flags |= HOST_FLAG_CTRL_ALIGNING;
    }
    if (arm->state == ARM_HOST && arm->host_closed_loop_mask != 0U)
    {
        flags |= HOST_FLAG_HOST_CLOSED_LOOP;
    }
    if (host->ever_cmd != 0U)
    {
        const uint32_t elapsed = now_ms - host->last_cmd_ms;
        age = elapsed > 0xFFFFU ? 0xFFFFU : (uint16_t)elapsed;
    }

    PutU32(payload + 0, (uint32_t)(sim_time_s * 1.0e6));
    payload[4] = (uint8_t)arm->state;
    payload[5] = ARM_JOINT_COUNT;
    payload[6] = HOST_REC_SIZE;
    PutU16(payload + 8, flags);
    PutU16(payload + 10, age);
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        PackJoint(arm, i, payload + HOST_STATE_HEAD_SIZE + i * HOST_REC_SIZE);
    }
    SendFrame(host, HOST_MSG_JOINT_STATE, payload, sizeof(payload));
}

/*--------------------------------------参数读取--------------------------------------*/
/* 返回参数类型和元素个数；未知参数返回 false */
static bool ParamInfo(uint16_t id, uint8_t *type, uint8_t *count)
{
    switch (id)
    {
    case HOST_PARAM_STATE_RATE_HZ:
    case HOST_PARAM_LINK_TIMEOUT:
        *type = 2U; *count = 1U; return true;
    case HOST_PARAM_ZERO_POINT:
        *type = 2U; *count = ARM_JOINT_COUNT; return true;
    case HOST_PARAM_DIRECTION:
        *type = 1U; *count = ARM_JOINT_COUNT; return true;
    case HOST_PARAM_LIMIT_MIN:
    case HOST_PARAM_LIMIT_MAX:
    case HOST_PARAM_GAIN_KP:
    case HOST_PARAM_GAIN_KI:
    case HOST_PARAM_GAIN_KD:
    case HOST_PARAM_GAIN_D_ALPHA:
    case HOST_PARAM_GAIN_SUM_MAX:
    case HOST_PARAM_GAIN_OUTPUT_MAX:
        *type = 6U; *count = ARM_JOINT_COUNT; return true;
    case HOST_PARAM_DH:
        *type = 6U; *count = ARM_JOINT_COUNT * 4U; return true;
    default:
        return false;
    }
}

static void ParamValue(const Host_t *host, uint16_t id, uint8_t index, uint8_t *value)
{
    const ArmParam_t *p = &host->arm->config.param;

    switch (id)
    {
    case HOST_PARAM_STATE_RATE_HZ:   PutU16(value, StateRateHz(host)); break;
    case HOST_PARAM_LINK_TIMEOUT:    PutU16(value, HOST_LINK_TIMEOUT_MS); break;
    case HOST_PARAM_ZERO_POINT:
    {   /* 实机是编码器计数，仿真零点是 rad，按 16 bit 编码器折回去 */
        const float turns = p->zero_point_rad[index] / MOTOR_TWO_PI;
        PutU16(value, (uint16_t)((int32_t)(turns * HOST_ENCODER_CPR) & 0xFFFF));
        break;
    }
    case HOST_PARAM_DIRECTION:       value[0] = (uint8_t)p->direction[index]; break;
    case HOST_PARAM_LIMIT_MIN:       PutFloat(value, p->limit[index].min_deg); break;
    case HOST_PARAM_LIMIT_MAX:       PutFloat(value, p->limit[index].max_deg); break;
    case HOST_PARAM_GAIN_KP:         PutFloat(value, p->gain[index].kp); break;
    case HOST_PARAM_GAIN_KI:         PutFloat(value, p->gain[index].ki); break;
    case HOST_PARAM_GAIN_KD:         PutFloat(value, p->gain[index].kd); break;
    case HOST_PARAM_GAIN_D_ALPHA:    PutFloat(value, p->gain[index].d_alpha); break;
    case HOST_PARAM_GAIN_SUM_MAX:    PutFloat(value, p->gain[index].sum_error_max); break;
    case HOST_PARAM_GAIN_OUTPUT_MAX: PutFloat(value, p->gain[index].output_max); break;
    case HOST_PARAM_DH:
    {   /* index = 行 * 4 + 列，列顺序 alpha / a / d / theta_offset */
        const DH_Param_t *row = &p->dh[index / 4U];
        const float cols[4] = {row->alpha_deg, row->a_m, row->d_m, row->theta_offset_deg};
        PutFloat(value, cols[index % 4U]);
        break;
    }
    default: break;
    }
}

/*--------------------------------------命令处理--------------------------------------*/
typedef struct {
    uint16_t seq;
    uint8_t type;
    uint8_t flags;
    uint8_t length;
    const uint8_t *payload;
} HostRequest_t;

static void ProcessParamRead(Host_t *host, const HostRequest_t *req)
{
    uint8_t payload[12] = {};
    uint8_t type = 0U, count = 0U;
    uint16_t id = 0U;
    uint8_t index = 0U;
    uint8_t result = HOST_ACK_BAD_LENGTH;

    if (req->length >= 3U)
    {
        id = GetU16(req->payload);
        index = req->payload[2];
        if (!ParamInfo(id, &type, &count))
        {
            result = HOST_ACK_UNSUPPORTED;
        }
        else if (index >= count)
        {
            result = HOST_ACK_BAD_PARAM;
        }
        else
        {
            result = HOST_ACK_OK;
            ParamValue(host, id, index, payload + 8);
        }
    }
    PutU16(payload, req->seq);
    PutU16(payload + 2, id);
    payload[4] = index;
    payload[5] = type;
    payload[6] = result;
    payload[7] = result == HOST_ACK_OK ? 1U : 0U;   /* access：只读 */
    Reply(host, req->type, req->seq, HOST_MSG_PARAM_VALUE, payload, sizeof(payload));
}

static void ProcessDeviceInfo(Host_t *host, const HostRequest_t *req)
{
    uint8_t payload[28] = {};

    payload[0] = HOST_FRAME_VERSION;
    payload[1] = ARM_JOINT_COUNT;
    payload[2] = HOST_REC_SIZE;
    payload[3] = (uint8_t)host->arm->config.state.role;
    PutU16(payload + 4, HOST_FRAME_MAX_SIZE);
    PutU16(payload + 6, StateRateHz(host));
    PutU16(payload + 10, HOST_LINK_TIMEOUT_MS);
    PutU16(payload + 14, 1U);
    memcpy(payload + 16, "ArmSim", 6U);
    Reply(host, req->type, req->seq, HOST_MSG_DEVICE_INFO, payload, sizeof(payload));
}

/* payload: joint(1) action(1) reserved(2) target_deg(4) */
static void ProcessSetTarget(Host_t *host, const HostRequest_t *req, uint32_t now_ms)
{
    uint8_t result = HOST_ACK_OK;

    if (req->length != 8U)
    {
        result = HOST_ACK_BAD_LENGTH;
    }
    else if (req->payload[1] == HOST_ACTION_STOP_ALL)
    {
        Arm_StopHostControl(host->arm);
    }
    else if (req->payload[1] == HOST_ACTION_SET_OPEN || req->payload[1] == HOST_ACTION_SET_CLOSED)
    {
        const bool closed = req->payload[1] == HOST_ACTION_SET_CLOSED;

        if (Arm_SetHostTarget(host->arm, req->payload[0], GetFloat(req->payload + 4), closed) !=
            ARM_HOST_TARGET_OK)
        {
            result = HOST_ACK_BAD_PARAM;
        }
    }
    else
    {
        result = HOST_ACK_BAD_PARAM;
    }

    if (result == HOST_ACK_OK)
    {
        host->last_cmd_ms = now_ms;
        host->ever_cmd = 1U;
    }
    Ack(host, req->flags, req->seq, req->type, result);
}

static void ProcessRequest(Host_t *host, const HostRequest_t *req, uint32_t now_ms)
{
    for (uint8_t i = 0U; i < host->reply_count; ++i)
    {
        const HostReply_t *cached = &host->replies[i];

        if (cached->seq == req->seq && cached->request_type == req->type)
        {
            SendFrame(host, cached->reply_type, cached->payload, cached->length);
            return;
        }
    }

    switch (req->type)
    {
    case HOST_MSG_HEARTBEAT:
        break;
    case HOST_MSG_DEVICE_INFO_REQ:
        ProcessDeviceInfo(host, req);
        break;
    case HOST_MSG_PARAM_READ:
        ProcessParamRead(host, req);
        break;
    case HOST_MSG_SET_HOST_TARGET:
        ProcessSetTarget(host, req, now_ms);
        break;
    default:
        Ack(host, req->flags, req->seq, req->type, HOST_ACK_UNSUPPORTED);
        break;
    }
}

/*--------------------------------------收帧--------------------------------------*/
static void DropRx(Host_t *host, uint16_t count)
{
    if (count >= host->rx_length)
    {
        host->rx_length = 0U;
        return;
    }
    memmove(host->rx_buffer, host->rx_buffer + count, host->rx_length - count);
    host->rx_length = (uint16_t)(host->rx_length - count);
}

/* 在缓冲里找完整且校验通过的帧，坏帧逐字节丢弃重新同步 */
static void ParseRx(Host_t *host, uint32_t now_ms)
{
    for (;;)
    {
        const uint8_t *buf = host->rx_buffer;
        uint16_t skip = 0U;
        uint16_t total;
        HostRequest_t req;

        while (skip + 1U < host->rx_length &&
               !(buf[skip] == HOST_FRAME_SOF0 && buf[skip + 1U] == HOST_FRAME_SOF1))
        {
            ++skip;
        }
        DropRx(host, skip);
        if (host->rx_length < HOST_FRAME_HEADER_SIZE)
        {
            return;
        }
        if (Host_Crc8(buf, 8U) != buf[8])
        {
            DropRx(host, 1U);
            continue;
        }
        total = (uint16_t)(HOST_FRAME_OVERHEAD + buf[6]);
        if (host->rx_length < total)
        {
            return;
        }
        if (Host_Crc16(buf, (uint16_t)(total - 2U)) != GetU16(buf + total - 2U))
        {
            DropRx(host, 1U);
            continue;
        }

        host->last_rx_ms = now_ms;
        host->ever_rx = 1U;
        if ((buf[2] >> 4) == (HOST_FRAME_VERSION >> 4))
        {
            req.type = buf[3];
            req.seq = GetU16(buf + 4);
            req.length = buf[6];
            req.flags = buf[7];
            req.payload = buf + HOST_FRAME_HEADER_SIZE;
            if (req.length > HOST_MAX_REQUEST)
            {
                Ack(host, req.flags, req.seq, req.type, HOST_ACK_BAD_LENGTH);
            }
            else
            {
                ProcessRequest(host, &req, now_ms);
            }
        }
        DropRx(host, total);
    }
}

/*--------------------------------------对外接口--------------------------------------*/
void Host_Init(Host_t *host, const HostConfig_t *config, Arm_t *arm)
{
    memset(host, 0, sizeof(*host));
    host->config = *config;
    host->arm = arm;
    if (host->config.state_divider == 0U)
    {
        host->config.state_divider = 1U;
    }
}

void Host_Run(Host_t *host, double sim_time_s, uint32_t wall_ms)
{
    uint8_t chunk[HOST_FRAME_MAX_SIZE];
    const int n = HostLink_Read(host->config.link, chunk, sizeof(chunk));

    if (n > 0)
    {
        const int overflow = (int)host->rx_length + n - (int)sizeof(host->rx_buffer);

        if (overflow > 0)
        {
            DropRx(host, (uint16_t)overflow);
        }
        memcpy(host->rx_buffer + host->rx_length, chunk, (size_t)n);
        host->rx_length = (uint16_t)(host->rx_length + n);
        ParseRx(host, wall_ms);
    }

    /* 上位机掉线：退出所有闭环，关节回到只受重力前馈托住 */
    if (host->arm->state == ARM_HOST && host->arm->host_closed_loop_mask != 0U &&
        !LinkOk(host, wall_ms))
    {
        Arm_StopHostControl(host->arm);
    }

    if (++host->divider_count >= host->config.state_divider)
    {
        host->divider_count = 0U;
        SendJointState(host, sim_time_s, wall_ms);
    }
}
