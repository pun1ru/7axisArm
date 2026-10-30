/* 固件对接示例：怎么把 arm_proto 挂到 DIYcontroller 现有框架上。
 * 本文件只是参考，没有放进 Real/，所有接口都按现有工程的真实签名写：
 *   UART_BindRx / UART_BindTx / UART_TransmitDMAEx   (LibSE/BSP/UART/uart_driver.h)
 *   Arm_t / ArmParam_t / Feedback_t                  (LibSE/Module/Arm/arm.h, Drivers/Motor/Motor.h)
 *
 * 线程模型与现有工程一致：
 *   接收：UART 中断回调 -> ArmProto_ParserFeed -> HostLink_OnFrame（在中断里，只改小状态，不调非 FromISR 的 RTOS 接口）
 *   发送：UartTransmitTask 每 2 ms 经 UART_DispatchTx 调 HostLink_Transmit
 *   控制：ArmTask 每 2 ms 调 HostLink_Tick 做超时检查，再用 HostLink_GetTorque 取有效设定
 *
 * 集成要点（需要你在 Real 里决定的）：
 *   1. 选一个空闲 UART（UART7/USART2/3/10，均已 921600），在 comm_init.c 里调 HostLink_BindUart
 *   2. 上位机力矩在什么状态下生效、替代还是叠加 PID 输出（下面 Arm_ApplyHostTorque 示例是 ENABLE 下替代）
 *   3. 急停锁存时是否强制 Arm_SetState(ARM_PROTECTED)
 */
#include "arm_proto.h"

#include <math.h>
#include <string.h>

#include "arm.h"
#include "uart_driver.h"

#define HOSTLINK_LINK_TIMEOUT_MS   300U
#define HOSTLINK_HOLD_DEFAULT_MS   100U
#define HOSTLINK_HOLD_MAX_MS       500U
#define HOSTLINK_STATE_DIV         5U      /* 2 ms 任务 × 5 = 200 Hz */
#define HOSTLINK_STATUS_DIV        50U     /* 10 Hz */
#define HOSTLINK_ACK_CACHE         8U
#define RAD_TO_DEG                 57.29577951f

/* 编译期硬上限，协议改不了（按电机额定扭矩填，下面是占位） */
static const float kHardTorqueLimitNm[ARM_JOINT_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

typedef struct {
    UartInstance_t *uart;
    Arm_t *arm;
    ArmProtoParser_t parser;
    uint32_t (*now_ms)(void);               /* 传 xTaskGetTickCount 换算或 HAL_GetTick */
    uint32_t (*now_us)(void);               /* DWT 微秒计数 */

    /* 中断写、任务读：单字 32 位读写在 M7 上是原子的，float 也是 */
    volatile float host_tau[ARM_JOINT_COUNT];
    volatile uint32_t deadline_ms[ARM_JOINT_COUNT];
    volatile float host_limit[ARM_JOINT_COUNT];
    volatile uint32_t last_rx_ms;
    volatile uint32_t last_cmd_ms;
    volatile uint8_t estop;
    volatile uint8_t link_ok;
    volatile uint8_t clamped_mask;

    /* 中断里生成的应答先放这里，由发送任务发出（中断里不能等 DMA） */
    uint8_t reply[ARM_PROTO_MAX_FRAME];
    volatile uint16_t reply_len;
    uint16_t ack_seq_cache[HOSTLINK_ACK_CACHE];
    uint8_t ack_seq_valid[HOSTLINK_ACK_CACHE];
    uint8_t ack_cache_pos;

    uint16_t tx_seq;
    uint16_t divider;
    uint32_t tx_drop;
    uint8_t tx_frame[ARM_PROTO_MAX_FRAME];
} HostLink_t;

static HostLink_t hostLink;

/*--------------------------------------接收（中断上下文）--------------------------------------*/
static void HostLink_QueueReply(HostLink_t *h, uint8_t type, const void *payload, uint16_t len)
{
    /* 只缓存一个应答：上位机同一时间最多几个未决命令，发送任务 2 ms 就取走；
     * 被覆盖的应答上位机会超时重试（幂等命令）或放弃（力矩命令） */
    h->reply_len = 0U;
    h->reply_len = ArmProto_Pack(h->reply, sizeof(h->reply), type, h->tx_seq++, 0U, payload, len);
}

static void HostLink_Ack(HostLink_t *h, uint16_t seq, uint8_t type, uint8_t result, uint32_t detail)
{
    ArmMsgAck_t ack = {seq, type, result, detail};
    HostLink_QueueReply(h, ARM_MSG_ACK, &ack, sizeof(ack));
}

static uint8_t HostLink_IsDuplicate(HostLink_t *h, uint16_t seq)
{
    for (uint8_t i = 0U; i < HOSTLINK_ACK_CACHE; ++i)
    {
        if (h->ack_seq_valid[i] && h->ack_seq_cache[i] == seq)
        {
            return 1U;
        }
    }
    h->ack_seq_cache[h->ack_cache_pos] = seq;
    h->ack_seq_valid[h->ack_cache_pos] = 1U;
    h->ack_cache_pos = (uint8_t)((h->ack_cache_pos + 1U) % HOSTLINK_ACK_CACHE);
    return 0U;
}

static void HostLink_ClearTorque(HostLink_t *h, uint32_t mask)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        if ((mask >> i) & 1U)
        {
            h->deadline_ms[i] = 0U;
            h->host_tau[i] = 0.0f;
        }
    }
}

static void HostLink_OnSetTorque(HostLink_t *h, uint16_t seq, const uint8_t *p, uint16_t len)
{
    ArmMsgSetTorqueHead_t head;
    uint32_t now = h->now_ms();
    uint32_t mask = 0U;
    uint16_t hold;

    if (len < sizeof(head)) { HostLink_Ack(h, seq, ARM_MSG_SET_JOINT_TORQUE, ARM_ACK_BAD_LENGTH, 0U); return; }
    memcpy(&head, p, sizeof(head));
    if (len < sizeof(head) + (uint16_t)head.count * sizeof(ArmTorqueItem_t))
    {
        HostLink_Ack(h, seq, ARM_MSG_SET_JOINT_TORQUE, ARM_ACK_BAD_LENGTH, 0U);
        return;
    }
    if (h->estop) { HostLink_Ack(h, seq, ARM_MSG_SET_JOINT_TORQUE, ARM_ACK_REJECT_ESTOP, 0U); return; }

    /* 先整体校验，任何一项非法就整条拒绝，不做部分生效 */
    for (uint8_t k = 0U; k < head.count; ++k)
    {
        ArmTorqueItem_t it;
        memcpy(&it, p + sizeof(head) + k * sizeof(it), sizeof(it));
        if (it.joint >= ARM_JOINT_COUNT || !isfinite(it.torque_nm))
        {
            HostLink_Ack(h, seq, ARM_MSG_SET_JOINT_TORQUE, ARM_ACK_BAD_PARAM, 0U);
            return;
        }
    }
    hold = head.hold_ms == 0U ? HOSTLINK_HOLD_DEFAULT_MS : head.hold_ms;
    if (hold > HOSTLINK_HOLD_MAX_MS) { hold = HOSTLINK_HOLD_MAX_MS; }

    for (uint8_t k = 0U; k < head.count; ++k)
    {
        ArmTorqueItem_t it;
        float lim, tau;
        memcpy(&it, p + sizeof(head) + k * sizeof(it), sizeof(it));
        lim = fminf(h->host_limit[it.joint], kHardTorqueLimitNm[it.joint]);
        tau = fmaxf(-lim, fminf(lim, it.torque_nm));
        if (tau != it.torque_nm) { mask |= 1UL << it.joint; }
        h->host_tau[it.joint] = tau;
        h->deadline_ms[it.joint] = now + hold;     /* 先写值再写期限，任务侧按期限判有效 */
    }
    h->clamped_mask = (uint8_t)mask;
    h->last_cmd_ms = now;
    HostLink_Ack(h, seq, ARM_MSG_SET_JOINT_TORQUE, mask ? ARM_ACK_OK_CLAMPED : ARM_ACK_OK, mask);
}

static void HostLink_OnFrame(void *ctx, uint8_t type, uint16_t seq, uint8_t flags,
                             const uint8_t *p, uint16_t len)
{
    HostLink_t *h = (HostLink_t *)ctx;
    uint8_t want_ack = (flags & ARM_PROTO_FLAG_ACK_REQ) != 0U;

    h->last_rx_ms = h->now_ms();
    if (type == ARM_MSG_HEARTBEAT)
    {
        return;
    }
    /* 重复 SEQ（上位机重试）：重新应答但不重复执行；力矩命令上位机不会重试 */
    if (want_ack && type != ARM_MSG_SET_JOINT_TORQUE && HostLink_IsDuplicate(h, seq))
    {
        HostLink_Ack(h, seq, type, ARM_ACK_OK, 0U);
        return;
    }
    switch (type)
    {
    case ARM_MSG_DEVICE_INFO_REQ:
    {
        ArmMsgDeviceInfo_t info = {ARM_PROTO_VERSION, ARM_JOINT_COUNT, sizeof(ArmJointRecord_t),
                                   (uint8_t)h->arm->config.state.role, ARM_PROTO_MAX_FRAME,
                                   1000U / (2U * HOSTLINK_STATE_DIV), 1000U / (2U * HOSTLINK_STATUS_DIV),
                                   HOSTLINK_LINK_TIMEOUT_MS, HOSTLINK_HOLD_MAX_MS, 1U, "DIYctrl"};
        HostLink_QueueReply(h, ARM_MSG_DEVICE_INFO, &info, sizeof(info));
        break;
    }
    case ARM_MSG_SET_JOINT_TORQUE:
        HostLink_OnSetTorque(h, seq, p, len);
        if (!want_ack) { h->reply_len = 0U; }     /* 不要应答就丢掉，省带宽 */
        break;
    case ARM_MSG_CLEAR_TORQUE:
    {
        ArmMsgClearTorque_t m;
        if (len < sizeof(m)) { HostLink_Ack(h, seq, type, ARM_ACK_BAD_LENGTH, 0U); break; }
        memcpy(&m, p, sizeof(m));
        HostLink_ClearTorque(h, m.joint_mask);
        HostLink_Ack(h, seq, type, ARM_ACK_OK, 0U);
        break;
    }
    case ARM_MSG_ESTOP:
        if (len < 1U) { HostLink_Ack(h, seq, type, ARM_ACK_BAD_LENGTH, 0U); break; }
        if (p[0] != 0U) { HostLink_ClearTorque(h, 0xFFFFFFFFUL); }
        h->estop = p[0] != 0U;
        HostLink_Ack(h, seq, type, ARM_ACK_OK, 0U);
        break;
    /* PARAM_READ / PARAM_WRITE：按 docs/02_protocol.md 第 6 节查表读写 arm->config.param，
     * 其中 PID 增益、关节限位、上位机力矩限幅只读：PARAM_WRITE 一律回 ARM_ACK_READ_ONLY；
     * 目前只有 STATE_RATE / STATUS_RATE 可写 */
    default:
        if (want_ack) { HostLink_Ack(h, seq, type, ARM_ACK_UNSUPPORTED, 0U); }
        break;
    }
}

static void HostLink_OnUartRx(void *ctx, const uint8_t *data, uint16_t size)
{
    ArmProto_ParserFeed(&((HostLink_t *)ctx)->parser, data, size);
}

static void HostLink_OnUartError(void *ctx)
{
    ArmProto_ParserReset(&((HostLink_t *)ctx)->parser);
}

/*--------------------------------------控制侧（ArmTask）--------------------------------------*/
/* 每个控制周期调一次：处理链路超时与各关节有效期 */
void HostLink_Tick(void)
{
    HostLink_t *h = &hostLink;
    uint32_t now = h->now_ms();

    h->link_ok = (uint32_t)(now - h->last_rx_ms) <= HOSTLINK_LINK_TIMEOUT_MS;
    if (!h->link_ok)
    {
        HostLink_ClearTorque(h, 0xFFFFFFFFUL);
    }
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        if (h->deadline_ms[i] != 0U && (int32_t)(now - h->deadline_ms[i]) > 0)
        {
            h->deadline_ms[i] = 0U;
            h->host_tau[i] = 0.0f;
        }
    }
}

/* 关节 i 有有效设定时返回 true 并给出力矩（关节口径） */
bool HostLink_GetTorque(uint8_t i, float *tau)
{
    HostLink_t *h = &hostLink;
    if (h->estop || !h->link_ok || h->deadline_ms[i] == 0U)
    {
        return false;
    }
    *tau = h->host_tau[i];
    return true;
}

/* 示例：在 Arm_JointControl 之后、Arm_SendJointCommand 之前，ENABLE 下用上位机设定替代 PID 输出。
 * 实际放在哪里、替代还是叠加，由你在 Real 里决定 */
void Arm_ApplyHostTorque(Arm_t *arm)
{
    float tau;
    if (arm->state != ARM_ENABLE)
    {
        return;
    }
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        if (HostLink_GetTorque(i, &tau))
        {
            arm->joint_ctrl[i].output = tau;
        }
    }
}

/*--------------------------------------发送（UartTransmitTask）--------------------------------------*/
static void HostLink_FillJointState(HostLink_t *h, uint8_t *payload, uint16_t *len)
{
    const Arm_t *arm = h->arm;
    ArmMsgJointStateHead_t head;
    uint32_t now = h->now_ms();
    uint16_t flags = 0U;
    uint8_t any_host = 0U;

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        const Motor_t *m = arm->config.motor.joint[i];
        const Feedback_t *fb = &m->feedback;
        float dir = (float)arm->config.param.direction[i];
        uint8_t valid = h->deadline_ms[i] != 0U && h->link_ok && !h->estop;
        ArmJointRecord_t r;

        r.angle_deg = arm->observer.joint_deg[i];
        r.velocity_dps = fb->omega_rads * RAD_TO_DEG * dir;   /* 电机口径 -> 关节口径 */
        r.torque_nm = fb->torque_nm * dir;
        r.target_deg = arm->joint_ctrl[i].target_deg;
        r.torque_cmd_nm = arm->state == ARM_ENABLE ? arm->joint_ctrl[i].output : 0.0f;
        r.encoder = fb->encoder;
        r.motor_status = (uint8_t)fb->status;
        r.error_id = fb->error_id;
        r.temperature_c = (int8_t)fb->temp;
        r.joint_flags = (uint8_t)((fb->status == MOTOR_ENABLED ? ARM_JF_ONLINE : 0U) |
                                  (valid ? ARM_JF_HOST_TORQUE_VALID : 0U) |
                                  (((h->clamped_mask >> i) & 1U) ? ARM_JF_TORQUE_CLAMPED : 0U));
        r.feedback_count = (uint16_t)fb->timestamp;
        memcpy(payload + sizeof(head) + i * sizeof(r), &r, sizeof(r));
        any_host |= valid;
    }
    if (h->link_ok) { flags |= ARM_SF_HOST_LINK_OK; }
    if (h->estop) { flags |= ARM_SF_ESTOP_LATCHED; }
    if (any_host) { flags |= ARM_SF_HOST_TORQUE_ACTIVE; }
    if (arm->ctrl_align_pending) { flags |= ARM_SF_CTRL_ALIGN_PENDING; }
    /* RC_ONLINE 由调用方按 RC_Remote_IsOnline 补上 */

    head.mcu_time_us = h->now_us();
    head.arm_state = (uint8_t)arm->state;
    head.joint_count = ARM_JOINT_COUNT;
    head.record_size = sizeof(ArmJointRecord_t);
    head.reserved = 0U;
    head.status_flags = flags;
    head.host_cmd_age_ms = h->last_cmd_ms == 0U ? 0xFFFFU :
                           (uint16_t)((now - h->last_cmd_ms) > 0xFFFFU ? 0xFFFFU : (now - h->last_cmd_ms));
    memcpy(payload, &head, sizeof(head));
    *len = (uint16_t)(sizeof(head) + ARM_JOINT_COUNT * sizeof(ArmJointRecord_t));
}

static void HostLink_Transmit(void *ctx, UartInstance_t *uart)
{
    HostLink_t *h = (HostLink_t *)ctx;
    uint8_t payload[ARM_PROTO_MAX_PAYLOAD];
    uint16_t len = 0U;
    uint16_t n = 0U;

    /* 应答优先 */
    if (h->reply_len != 0U)
    {
        if (UART_TransmitDMAEx(uart, h->reply, h->reply_len) == HAL_OK) { h->reply_len = 0U; }
        return;
    }
    if (++h->divider < HOSTLINK_STATE_DIV)
    {
        return;
    }
    h->divider = 0U;
    HostLink_FillJointState(h, payload, &len);
    n = ArmProto_Pack(h->tx_frame, sizeof(h->tx_frame), ARM_MSG_JOINT_STATE, h->tx_seq++, 0U, payload, len);
    /* DMA 忙就丢这一帧（序号已递增，上位机能统计到丢帧），下个周期带新数据再发 */
    if (n == 0U || UART_TransmitDMAEx(uart, h->tx_frame, n) != HAL_OK) { h->tx_drop++; }
    /* SYSTEM_STATUS / EVENT / HEARTBEAT 按同样方式分频发送，略 */
}

/*--------------------------------------初始化--------------------------------------*/
/* 在 comm_init.c 的 Comm_UartRegister 里、UART_StartReceiveEx 之前调用 */
BaseType_t HostLink_BindUart(UartInstance_t *uart, Arm_t *arm,
                             uint32_t (*now_ms)(void), uint32_t (*now_us)(void))
{
    memset(&hostLink, 0, sizeof(hostLink));
    hostLink.uart = uart;
    hostLink.arm = arm;
    hostLink.now_ms = now_ms;
    hostLink.now_us = now_us;
    /* host_limit 是只读的编译期常量（上位机不能改）：这里按 0 初始化 = 禁止上位机出力，
     * 要用上位机力矩，先在这里给各关节填非零限幅（≤ kHardTorqueLimitNm） */
    ArmProto_ParserInit(&hostLink.parser, HostLink_OnFrame, &hostLink);
    if (UART_BindRx(uart, HostLink_OnUartRx, HostLink_OnUartError, &hostLink) != pdPASS)
    {
        return pdFAIL;
    }
    return UART_BindTx(uart, HostLink_Transmit, &hostLink);
}
