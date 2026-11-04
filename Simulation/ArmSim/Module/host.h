#ifndef HOST_H
#define HOST_H

#include <stdbool.h>
#include <stdint.h>

#include "arm.h"
#include "host_link.h"

/* Module 层：上位机协议桥，帧格式、消息号、参数号与实机
 * Real/DIYcontroller/LibSE/Module/Host/host.c 相同，PC 端 Host/armhost 无需修改。
 *
 *   帧   A5 5A | ver | type | seq(2) | len | flags | crc8 | payload | crc16     小端
 *   上行 JOINT_STATE 周期上报，另有 ACK / PARAM_VALUE / DEVICE_INFO 应答
 *   下行 HEARTBEAT / DEVICE_INFO_REQ / PARAM_READ / SET_HOST_TARGET
 *
 * 与实机的差别：没有 UART 中断，Host_Run 在任务里轮询链路；应答直接写链路，
 * 不排队。已实现的消息就这几条，其余（力矩直控、急停、参数写）回 UNSUPPORTED。 */
#define HOST_FRAME_SOF0             0xA5U
#define HOST_FRAME_SOF1             0x5AU
#define HOST_FRAME_VERSION          0x10U
#define HOST_FRAME_HEADER_SIZE      9U
#define HOST_FRAME_OVERHEAD         11U
#define HOST_FRAME_MAX_SIZE         256U

#define HOST_MSG_HEARTBEAT          0x01U
#define HOST_MSG_DEVICE_INFO_REQ    0x02U
#define HOST_MSG_DEVICE_INFO        0x03U
#define HOST_MSG_ACK                0x04U
#define HOST_MSG_JOINT_STATE        0x10U
#define HOST_MSG_SET_HOST_TARGET    0x23U
#define HOST_MSG_PARAM_READ         0x28U
#define HOST_MSG_PARAM_VALUE        0x2AU

#define HOST_LINK_TIMEOUT_MS        300U    /* 超时无帧则退出上位机闭环 */

typedef struct {
    HostLink_t *link;
    uint16_t state_divider;        /* 每 N 个 Host_Run 上报一次 JOINT_STATE */
} HostConfig_t;

typedef struct {
    uint16_t seq;
    uint8_t request_type;
    uint8_t reply_type;
    uint8_t length;
    uint8_t payload[28];
} HostReply_t;

typedef struct {
    HostConfig_t config;
    Arm_t *arm;
    uint16_t divider_count;
    uint16_t sequence;
    uint32_t last_rx_ms;
    uint32_t last_cmd_ms;
    uint8_t ever_rx;
    uint8_t ever_cmd;
    uint8_t rx_buffer[2U * HOST_FRAME_MAX_SIZE];
    uint16_t rx_length;
    HostReply_t replies[8];        /* 重复请求（同 type + seq）重放上次应答 */
    uint8_t reply_head;
    uint8_t reply_count;
} Host_t;

void Host_Init(Host_t *host, const HostConfig_t *config, Arm_t *arm);
/* 每个任务周期调一次：收帧 -> 执行命令并应答 -> 掉线保护 -> 周期上报。
 * sim_time_s 是仿真时间（写进上报帧）；wall_ms 是墙钟毫秒（判链路超时，
 * 因为上位机是真实时间，--fast 下仿真时间不能拿来判超时） */
void Host_Run(Host_t *host, double sim_time_s, uint32_t wall_ms);

uint8_t Host_Crc8(const uint8_t *data, uint16_t length);
uint16_t Host_Crc16(const uint8_t *data, uint16_t length);

#endif
