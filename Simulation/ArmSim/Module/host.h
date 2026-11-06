#pragma once

#include <array>
#include <cstdint>

#include "arm.h"
#include "host_link.h"

/* 上位机协议桥，帧格式、消息号、参数号与实机
 * Real/DIYcontroller/LibSE/Module/Host/host.c 相同，PC 端 Host/armhost 无需修改。
 *
 *   帧   A5 5A | ver | type | seq(2) | len | flags | crc8 | payload | crc16     小端
 *   上行 JOINT_STATE 周期上报，另有 ACK / PARAM_VALUE / DEVICE_INFO 应答
 *   下行 HEARTBEAT / DEVICE_INFO_REQ / PARAM_READ / SET_HOST_TARGET
 *
 * 与实机的差别：没有 UART 中断，Step 在循环里轮询链路；应答直接写链路，不排队。
 * 已实现的消息就这几条，其余（力矩直控、急停、参数写）回 UNSUPPORTED。 */
class Host {
public:
    /* state_divider：每 N 次 Step 上报一次 JOINT_STATE */
    Host(Arm &arm, HostLink &link, uint16_t state_divider);

    /* 每个控制周期调一次：收帧 -> 执行命令并应答 -> 掉线保护 -> 周期上报。
     * sim_time_s 是仿真时间（写进上报帧）；wall_ms 是墙钟毫秒（判链路超时，
     * 因为上位机是真实时间，--fast 下仿真时间不能拿来判超时） */
    void Step(double sim_time_s, uint32_t wall_ms);

private:
    static constexpr size_t kFrameMaxSize = 256;
    static constexpr size_t kReplyCacheSize = 8;

    struct Request {
        uint16_t seq;
        uint8_t type;
        uint8_t flags;
        uint8_t length;
        const uint8_t *payload;
    };

    struct Reply {
        uint16_t seq;
        uint8_t request_type;
        uint8_t reply_type;
        uint8_t length;
        std::array<uint8_t, 28> payload;
    };

    // 发送
    void SendFrame(uint8_t type, const uint8_t *payload, uint8_t length);
    void SendReply(const Request &req, uint8_t reply_type, const uint8_t *payload, uint8_t length);
    void Ack(const Request &req, uint8_t result);
    void SendJointState(double sim_time_s, uint32_t now_ms);
    void PackJoint(int joint, uint8_t *out) const;

    // 命令
    void ProcessRequest(const Request &req, uint32_t now_ms);
    void ProcessParamRead(const Request &req);
    void ProcessDeviceInfo(const Request &req);
    void ProcessSetTarget(const Request &req, uint32_t now_ms);
    void ParamValue(uint16_t id, uint8_t index, uint8_t *value) const;

    // 收帧
    void DropRx(size_t count);
    void ParseRx(uint32_t now_ms);

    uint16_t StateRateHz() const;
    bool LinkOk(uint32_t now_ms) const;

    Arm &arm_;
    HostLink &link_;
    uint16_t state_divider_;
    uint16_t divider_count_ = 0;
    uint16_t sequence_ = 0;
    uint32_t last_rx_ms_ = 0;
    uint32_t last_cmd_ms_ = 0;
    bool ever_rx_ = false;
    bool ever_cmd_ = false;
    std::array<uint8_t, 2 * kFrameMaxSize> rx_buffer_{};
    size_t rx_length_ = 0;
    std::array<Reply, kReplyCacheSize> replies_{};   // 重复请求（同 type + seq）重放上次应答
    size_t reply_head_ = 0;
    size_t reply_count_ = 0;
};
