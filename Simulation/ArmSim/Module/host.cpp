#include "host.h"

#include <cstring>

namespace {

constexpr uint8_t kSof0 = 0xA5;
constexpr uint8_t kSof1 = 0x5A;
constexpr uint8_t kFrameVersion = 0x10;
constexpr size_t kHeaderSize = 9;
constexpr size_t kFrameOverhead = 11;
constexpr uint8_t kMaxRequest = 32;

constexpr uint8_t kMsgHeartbeat = 0x01;
constexpr uint8_t kMsgDeviceInfoReq = 0x02;
constexpr uint8_t kMsgDeviceInfo = 0x03;
constexpr uint8_t kMsgAck = 0x04;
constexpr uint8_t kMsgJointState = 0x10;
constexpr uint8_t kMsgSetHostTarget = 0x23;
constexpr uint8_t kMsgParamRead = 0x28;
constexpr uint8_t kMsgParamValue = 0x2A;

constexpr uint32_t kLinkTimeoutMs = 300;   // 超时无帧则退出上位机闭环
constexpr uint32_t kTaskPeriodMs = 2;
constexpr float kEncoderCpr = 65536.0f;

constexpr size_t kRecSize = 28;            // 每关节一条记录
constexpr size_t kStateHeadSize = 12;
constexpr size_t kStateSize = kStateHeadSize + kJointCount * kRecSize;

constexpr uint8_t kAckOk = 0;
constexpr uint8_t kAckBadParam = 3;
constexpr uint8_t kAckBadLength = 4;
constexpr uint8_t kAckUnsupported = 5;
constexpr uint8_t kFlagAckReq = 0x01;

constexpr uint8_t kActionSetOpen = 0;      // 保存目标，该关节不闭环
constexpr uint8_t kActionSetClosed = 1;    // 保存目标，该关节投入闭环
constexpr uint8_t kActionStopAll = 2;      // 全部关节退出闭环，目标保留

// PARAM_READ 的参数号；类型：U16=2 I8=1 F32=6
constexpr uint16_t kParamStateRateHz = 0x0001;
constexpr uint16_t kParamLinkTimeout = 0x0003;
constexpr uint16_t kParamZeroPoint = 0x0100;
constexpr uint16_t kParamDirection = 0x0101;
constexpr uint16_t kParamLimitMin = 0x0102;
constexpr uint16_t kParamLimitMax = 0x0103;
constexpr uint16_t kParamGainKp = 0x0200;
constexpr uint16_t kParamGainKi = 0x0201;
constexpr uint16_t kParamGainKd = 0x0202;
constexpr uint16_t kParamGainDAlpha = 0x0203;
constexpr uint16_t kParamGainSumMax = 0x0204;
constexpr uint16_t kParamGainOutputMax = 0x0205;
constexpr uint16_t kParamDh = 0x0400;

// status_flags / joint_flags
constexpr uint16_t kFlagLinkOk = 0x0002;
constexpr uint16_t kFlagCtrlAligning = 0x0010;
constexpr uint16_t kFlagHostClosedLoop = 0x0020;
constexpr uint8_t kJointOnline = 0x01;
constexpr uint8_t kJointTargetValid = 0x10;
constexpr uint8_t kJointClosedLoop = 0x20;

uint8_t Crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x80U) != 0U ? static_cast<uint8_t>((crc << 1) ^ 0x07U)
                                      : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8);
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000U) != 0U ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                        : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// 字节打包（小端）
void PutU16(uint8_t *out, uint16_t v)
{
    out[0] = static_cast<uint8_t>(v & 0xFFU);
    out[1] = static_cast<uint8_t>(v >> 8);
}

void PutU32(uint8_t *out, uint32_t v)
{
    PutU16(out, static_cast<uint16_t>(v & 0xFFFFU));
    PutU16(out + 2, static_cast<uint16_t>(v >> 16));
}

void PutFloat(uint8_t *out, float v)
{
    uint32_t bits;

    std::memcpy(&bits, &v, sizeof(bits));
    PutU32(out, bits);
}

uint16_t GetU16(const uint8_t *in)
{
    return static_cast<uint16_t>(in[0] | (in[1] << 8));
}

float GetFloat(const uint8_t *in)
{
    const uint32_t bits = static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
                          (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
    float v;

    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

/* 返回参数类型和元素个数；未知参数返回 false */
bool ParamInfo(uint16_t id, uint8_t &type, uint8_t &count)
{
    switch (id)
    {
    case kParamStateRateHz:
    case kParamLinkTimeout:
        type = 2; count = 1; return true;
    case kParamZeroPoint:
        type = 2; count = kJointCount; return true;
    case kParamDirection:
        type = 1; count = kJointCount; return true;
    case kParamLimitMin:
    case kParamLimitMax:
    case kParamGainKp:
    case kParamGainKi:
    case kParamGainKd:
    case kParamGainDAlpha:
    case kParamGainSumMax:
    case kParamGainOutputMax:
        type = 6; count = kJointCount; return true;
    case kParamDh:
        type = 6; count = kJointCount * 4; return true;
    default:
        return false;
    }
}

}  // namespace

Host::Host(Arm &arm, HostLink &link, uint16_t state_divider)
    : arm_(arm), link_(link), state_divider_(state_divider == 0 ? 1 : state_divider)
{
}

/*--------------------------------------发送--------------------------------------*/
void Host::SendFrame(uint8_t type, const uint8_t *payload, uint8_t length)
{
    uint8_t frame[kFrameMaxSize];

    frame[0] = kSof0;
    frame[1] = kSof1;
    frame[2] = kFrameVersion;
    frame[3] = type;
    PutU16(frame + 4, sequence_++);
    frame[6] = length;
    frame[7] = 0;
    frame[8] = Crc8(frame, 8);
    std::memcpy(frame + kHeaderSize, payload, length);
    PutU16(frame + kHeaderSize + length, Crc16(frame, kHeaderSize + length));
    link_.Write(frame, kFrameOverhead + length);
}

/* 应答并缓存：上位机重发同一个 (type, seq) 时直接重放，不重复执行 */
void Host::SendReply(const Request &req, uint8_t reply_type, const uint8_t *payload, uint8_t length)
{
    Reply &entry = replies_[reply_head_];

    entry.seq = req.seq;
    entry.request_type = req.type;
    entry.reply_type = reply_type;
    entry.length = length;
    std::memcpy(entry.payload.data(), payload, length);
    reply_head_ = (reply_head_ + 1) % kReplyCacheSize;
    if (reply_count_ < kReplyCacheSize)
    {
        ++reply_count_;
    }
    SendFrame(reply_type, payload, length);
}

void Host::Ack(const Request &req, uint8_t result)
{
    uint8_t payload[8] = {};

    if ((req.flags & kFlagAckReq) == 0U)
    {
        return;
    }
    PutU16(payload, req.seq);
    payload[2] = req.type;
    payload[3] = result;
    SendReply(req, kMsgAck, payload, sizeof(payload));
}

/*--------------------------------------JOINT_STATE 上报--------------------------------------*/
uint16_t Host::StateRateHz() const
{
    return static_cast<uint16_t>(1000U / (kTaskPeriodMs * state_divider_));
}

bool Host::LinkOk(uint32_t now_ms) const
{
    return ever_rx_ && static_cast<uint32_t>(now_ms - last_rx_ms_) <= kLinkTimeoutMs;
}

void Host::PackJoint(int joint, uint8_t *out) const
{
    const Arm::Observer &obs = arm_.observer();
    const Arm::JointCtrl &ctrl = arm_.joint(joint);
    const uint8_t bit = static_cast<uint8_t>(1U << joint);
    uint8_t flags = kJointOnline;

    if (arm_.state() == Arm::State::Host && (arm_.host_target_valid_mask() & bit) != 0U)
    {
        flags |= kJointTargetValid;
    }
    if (arm_.state() == Arm::State::Host && (arm_.host_closed_loop_mask() & bit) != 0U)
    {
        flags |= kJointClosedLoop;
    }

    PutFloat(out + 0, obs.joint_deg[joint]);
    PutFloat(out + 4, obs.joint_vel_dps[joint]);
    PutFloat(out + 8, obs.joint_torque_nm[joint]);
    PutFloat(out + 12, ctrl.target_deg);
    PutFloat(out + 16, ctrl.output);
    PutU16(out + 20, obs.encoder[joint]);
    out[22] = arm_.enabled() ? 1U : 0U;   // 电机状态：1 = 使能，0 = 失能
    out[23] = 0;                          // error_id：仿真没有电机故障
    out[24] = 0;                          // 温度：仿真没有温度模型
    out[25] = flags;
    PutU16(out + 26, static_cast<uint16_t>(arm_.cycle_count() & 0xFFFFU));
}

void Host::SendJointState(double sim_time_s, uint32_t now_ms)
{
    uint8_t payload[kStateSize] = {};
    uint16_t flags = 0;
    uint16_t age = 0xFFFF;   // 还没收到过有效命令

    if (LinkOk(now_ms))
    {
        flags |= kFlagLinkOk;
    }
    if (arm_.aligning())
    {
        flags |= kFlagCtrlAligning;
    }
    if (arm_.state() == Arm::State::Host && arm_.host_closed_loop_mask() != 0U)
    {
        flags |= kFlagHostClosedLoop;
    }
    if (ever_cmd_)
    {
        const uint32_t elapsed = now_ms - last_cmd_ms_;
        age = elapsed > 0xFFFFU ? 0xFFFF : static_cast<uint16_t>(elapsed);
    }

    PutU32(payload + 0, static_cast<uint32_t>(sim_time_s * 1.0e6));
    payload[4] = static_cast<uint8_t>(arm_.state());
    payload[5] = kJointCount;
    payload[6] = kRecSize;
    PutU16(payload + 8, flags);
    PutU16(payload + 10, age);
    for (int i = 0; i < kJointCount; ++i)
    {
        PackJoint(i, payload + kStateHeadSize + i * kRecSize);
    }
    SendFrame(kMsgJointState, payload, sizeof(payload));
}

/*--------------------------------------参数读取--------------------------------------*/
void Host::ParamValue(uint16_t id, uint8_t index, uint8_t *value) const
{
    const Arm::Config &c = arm_.config();

    switch (id)
    {
    case kParamStateRateHz:   PutU16(value, StateRateHz()); break;
    case kParamLinkTimeout:   PutU16(value, kLinkTimeoutMs); break;
    case kParamZeroPoint:
    {   // 实机是编码器计数，仿真零点是 rad，按 16 bit 编码器折回去
        const float turns = c.zero_point_rad[index] / kTwoPi;
        PutU16(value, static_cast<uint16_t>(static_cast<int32_t>(turns * kEncoderCpr) & 0xFFFF));
        break;
    }
    case kParamDirection:       value[0] = static_cast<uint8_t>(c.direction[index]); break;
    case kParamLimitMin:        PutFloat(value, c.limit[index].min_deg); break;
    case kParamLimitMax:        PutFloat(value, c.limit[index].max_deg); break;
    case kParamGainKp:          PutFloat(value, c.gain[index].kp); break;
    case kParamGainKi:          PutFloat(value, c.gain[index].ki); break;
    case kParamGainKd:          PutFloat(value, c.gain[index].kd); break;
    case kParamGainDAlpha:      PutFloat(value, c.gain[index].d_alpha); break;
    case kParamGainSumMax:      PutFloat(value, c.gain[index].sum_error_max); break;
    case kParamGainOutputMax:   PutFloat(value, c.gain[index].output_max); break;
    case kParamDh:
    {   // index = 行 * 4 + 列，列顺序 alpha / a / d / theta_offset
        const DhParam &row = c.dh[index / 4];
        const float cols[4] = {row.alpha_deg, row.a_m, row.d_m, row.theta_offset_deg};
        PutFloat(value, cols[index % 4]);
        break;
    }
    default: break;
    }
}

/*--------------------------------------命令处理--------------------------------------*/
void Host::ProcessParamRead(const Request &req)
{
    uint8_t payload[12] = {};
    uint8_t type = 0, count = 0;
    uint16_t id = 0;
    uint8_t index = 0;
    uint8_t result = kAckBadLength;

    if (req.length >= 3)
    {
        id = GetU16(req.payload);
        index = req.payload[2];
        if (!ParamInfo(id, type, count))
        {
            result = kAckUnsupported;
        }
        else if (index >= count)
        {
            result = kAckBadParam;
        }
        else
        {
            result = kAckOk;
            ParamValue(id, index, payload + 8);
        }
    }
    PutU16(payload, req.seq);
    PutU16(payload + 2, id);
    payload[4] = index;
    payload[5] = type;
    payload[6] = result;
    payload[7] = result == kAckOk ? 1U : 0U;   // access：只读
    SendReply(req, kMsgParamValue, payload, sizeof(payload));
}

void Host::ProcessDeviceInfo(const Request &req)
{
    uint8_t payload[28] = {};

    payload[0] = kFrameVersion;
    payload[1] = kJointCount;
    payload[2] = kRecSize;
    payload[3] = static_cast<uint8_t>(arm_.config().role);
    PutU16(payload + 4, static_cast<uint16_t>(256));
    PutU16(payload + 6, StateRateHz());
    PutU16(payload + 10, kLinkTimeoutMs);
    PutU16(payload + 14, 1);
    std::memcpy(payload + 16, "ArmSim", 6);
    SendReply(req, kMsgDeviceInfo, payload, sizeof(payload));
}

/* payload: joint(1) action(1) reserved(2) target_deg(4) */
void Host::ProcessSetTarget(const Request &req, uint32_t now_ms)
{
    uint8_t result = kAckOk;

    if (req.length != 8)
    {
        result = kAckBadLength;
    }
    else if (req.payload[1] == kActionStopAll)
    {
        arm_.StopHostControl();
    }
    else if (req.payload[1] == kActionSetOpen || req.payload[1] == kActionSetClosed)
    {
        const bool closed = req.payload[1] == kActionSetClosed;

        if (arm_.SetHostTarget(req.payload[0], GetFloat(req.payload + 4), closed) !=
            Arm::HostTargetResult::Ok)
        {
            result = kAckBadParam;
        }
    }
    else
    {
        result = kAckBadParam;
    }

    if (result == kAckOk)
    {
        last_cmd_ms_ = now_ms;
        ever_cmd_ = true;
    }
    Ack(req, result);
}

void Host::ProcessRequest(const Request &req, uint32_t now_ms)
{
    for (size_t i = 0; i < reply_count_; ++i)
    {
        const Reply &cached = replies_[i];

        if (cached.seq == req.seq && cached.request_type == req.type)
        {
            SendFrame(cached.reply_type, cached.payload.data(), cached.length);
            return;
        }
    }

    switch (req.type)
    {
    case kMsgHeartbeat:
        break;
    case kMsgDeviceInfoReq:
        ProcessDeviceInfo(req);
        break;
    case kMsgParamRead:
        ProcessParamRead(req);
        break;
    case kMsgSetHostTarget:
        ProcessSetTarget(req, now_ms);
        break;
    default:
        Ack(req, kAckUnsupported);
        break;
    }
}

/*--------------------------------------收帧--------------------------------------*/
void Host::DropRx(size_t count)
{
    if (count >= rx_length_)
    {
        rx_length_ = 0;
        return;
    }
    std::memmove(rx_buffer_.data(), rx_buffer_.data() + count, rx_length_ - count);
    rx_length_ -= count;
}

/* 在缓冲里找完整且校验通过的帧，坏帧逐字节丢弃重新同步 */
void Host::ParseRx(uint32_t now_ms)
{
    for (;;)
    {
        const uint8_t *buf = rx_buffer_.data();
        size_t skip = 0;

        while (skip + 1 < rx_length_ && !(buf[skip] == kSof0 && buf[skip + 1] == kSof1))
        {
            ++skip;
        }
        DropRx(skip);
        if (rx_length_ < kHeaderSize)
        {
            return;
        }
        if (Crc8(buf, 8) != buf[8])
        {
            DropRx(1);
            continue;
        }
        const size_t total = kFrameOverhead + buf[6];
        if (rx_length_ < total)
        {
            return;
        }
        if (Crc16(buf, total - 2) != GetU16(buf + total - 2))
        {
            DropRx(1);
            continue;
        }

        last_rx_ms_ = now_ms;
        ever_rx_ = true;
        if ((buf[2] >> 4) == (kFrameVersion >> 4))
        {
            const Request req{GetU16(buf + 4), buf[3], buf[7], buf[6], buf + kHeaderSize};

            if (req.length > kMaxRequest)
            {
                Ack(req, kAckBadLength);
            }
            else
            {
                ProcessRequest(req, now_ms);
            }
        }
        DropRx(total);
    }
}

/*--------------------------------------对外接口--------------------------------------*/
void Host::Step(double sim_time_s, uint32_t wall_ms)
{
    uint8_t chunk[kFrameMaxSize];
    const int n = link_.Read(chunk, sizeof(chunk));

    if (n > 0)
    {
        const int overflow = static_cast<int>(rx_length_) + n - static_cast<int>(rx_buffer_.size());

        if (overflow > 0)
        {
            DropRx(static_cast<size_t>(overflow));
        }
        std::memcpy(rx_buffer_.data() + rx_length_, chunk, static_cast<size_t>(n));
        rx_length_ += static_cast<size_t>(n);
        ParseRx(wall_ms);
    }

    /* 上位机掉线：退出所有闭环，关节回到只受重力前馈托住 */
    if (arm_.state() == Arm::State::Host && arm_.host_closed_loop_mask() != 0U && !LinkOk(wall_ms))
    {
        arm_.StopHostControl();
    }

    if (++divider_count_ >= state_divider_)
    {
        divider_count_ = 0;
        SendJointState(sim_time_s, wall_ms);
    }
}
