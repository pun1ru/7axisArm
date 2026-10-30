#include "arm_proto.h"

#include <string.h>

/*--------------------------------------CRC--------------------------------------*/
/* CRC-8/SMBUS：poly 0x07 init 0x00，check("123456789") = 0xF4 */
uint8_t ArmProto_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;
    for (uint16_t i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; ++b)
        {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* CRC-16/CCITT-FALSE：poly 0x1021 init 0xFFFF，check = 0x29B1
 * 逐位算法每字节约 8 次移位，219 字节帧在 480 MHz H7 上不到 10 us；需要再快可换查表 */
uint16_t ArmProto_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0U; b < 8U; ++b)
        {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/*--------------------------------------打包--------------------------------------*/
uint16_t ArmProto_Pack(uint8_t *out, uint16_t out_size, uint8_t type, uint16_t seq,
                       uint8_t flags, const void *payload, uint16_t len)
{
    uint16_t total = (uint16_t)(ARM_PROTO_OVERHEAD + len);
    uint16_t crc;

    if (len > ARM_PROTO_MAX_PAYLOAD || out_size < total)
    {
        return 0U;
    }
    out[0] = ARM_PROTO_SOF0;
    out[1] = ARM_PROTO_SOF1;
    out[2] = ARM_PROTO_VERSION;
    out[3] = type;
    out[4] = (uint8_t)(seq & 0xFFU);
    out[5] = (uint8_t)(seq >> 8);
    out[6] = (uint8_t)len;
    out[7] = flags;
    out[8] = ArmProto_Crc8(out, 8U);
    if (len != 0U)
    {
        memcpy(&out[ARM_PROTO_HEADER_SIZE], payload, len);
    }
    crc = ArmProto_Crc16(out, (uint16_t)(ARM_PROTO_HEADER_SIZE + len));
    out[total - 2U] = (uint8_t)(crc & 0xFFU);
    out[total - 1U] = (uint8_t)(crc >> 8);
    return total;
}

/*--------------------------------------解包--------------------------------------*/
void ArmProto_ParserInit(ArmProtoParser_t *p, ArmProtoFrameHandler_t on_frame, void *context)
{
    memset(p, 0, sizeof(*p));
    p->on_frame = on_frame;
    p->context = context;
}

void ArmProto_ParserReset(ArmProtoParser_t *p)
{
    p->stats.bytes_discarded += p->len;
    p->len = 0U;
}

static void Parser_Drop(ArmProtoParser_t *p, uint16_t n)
{
    if (n >= p->len)
    {
        p->stats.bytes_discarded += p->len;
        p->len = 0U;
        return;
    }
    memmove(p->buf, &p->buf[n], (size_t)(p->len - n));
    p->len = (uint16_t)(p->len - n);
    p->stats.bytes_discarded += n;
}

/* 处理缓冲里能处理的所有帧，规则与 Python FrameParser 相同 */
static void Parser_Drain(ArmProtoParser_t *p)
{
    for (;;)
    {
        uint16_t i = 0U;
        uint16_t length;
        uint16_t total;
        uint16_t rx_crc;

        /* 1. 找 A5 5A */
        while (i + 1U < p->len && !(p->buf[i] == ARM_PROTO_SOF0 && p->buf[i + 1U] == ARM_PROTO_SOF1))
        {
            ++i;
        }
        if (i + 1U >= p->len)
        {
            /* 没找到：最后一个字节若是 A5 留着，可能是下一帧的开头 */
            uint16_t keep = (p->len != 0U && p->buf[p->len - 1U] == ARM_PROTO_SOF0) ? 1U : 0U;
            Parser_Drop(p, (uint16_t)(p->len - keep));
            return;
        }
        if (i != 0U)
        {
            Parser_Drop(p, i);
        }
        if (p->len < ARM_PROTO_HEADER_SIZE)
        {
            return;
        }
        /* 2. 帧头 */
        if (ArmProto_Crc8(p->buf, 8U) != p->buf[8])
        {
            p->stats.header_errors++;
            Parser_Drop(p, 1U);
            continue;
        }
        length = p->buf[6];
        if (length > ARM_PROTO_MAX_PAYLOAD)
        {
            p->stats.length_errors++;
            Parser_Drop(p, 1U);
            continue;
        }
        total = (uint16_t)(ARM_PROTO_OVERHEAD + length);
        if (p->len < total)
        {
            return;
        }
        /* 3. 整帧 CRC：失败只丢 1 字节，真正的帧头可能藏在坏帧里 */
        rx_crc = (uint16_t)(p->buf[total - 2U] | ((uint16_t)p->buf[total - 1U] << 8));
        if (ArmProto_Crc16(p->buf, (uint16_t)(total - 2U)) != rx_crc)
        {
            p->stats.crc_errors++;
            Parser_Drop(p, 1U);
            continue;
        }
        if ((p->buf[2] >> 4) != ARM_PROTO_MAJOR)
        {
            p->stats.version_errors++;
            Parser_Drop(p, total);
            continue;
        }
        p->stats.frames_ok++;
        if (p->on_frame != NULL)
        {
            p->on_frame(p->context, p->buf[3], (uint16_t)(p->buf[4] | ((uint16_t)p->buf[5] << 8)),
                        p->buf[7], &p->buf[ARM_PROTO_HEADER_SIZE], length);
        }
        /* 回调返回后才移走这一帧，回调里拿到的 payload 指针只在回调期间有效 */
        memmove(p->buf, &p->buf[total], (size_t)(p->len - total));
        p->len = (uint16_t)(p->len - total);
    }
}

void ArmProto_ParserFeed(ArmProtoParser_t *p, const uint8_t *data, uint16_t len)
{
    while (len != 0U)
    {
        uint16_t room = (uint16_t)(sizeof(p->buf) - p->len);
        uint16_t n = len < room ? len : room;

        memcpy(&p->buf[p->len], data, n);
        p->len = (uint16_t)(p->len + n);
        data += n;
        len = (uint16_t)(len - n);
        Parser_Drain(p);
        if (p->len == sizeof(p->buf))
        {
            /* 缓冲满还解不出帧：理论上不会发生（帧长 <= 缓冲的一半），防御性清掉 */
            ArmProto_ParserReset(p);
        }
    }
}
