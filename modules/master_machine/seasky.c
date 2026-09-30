/**
 * @file seasky.c
 * @brief Seasky V1 视觉协议实现
 */
#include "seasky.h"
#include <string.h>

uint16_t seasky_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

int16_t seasky_pop_frame(uint8_t *buf, uint16_t *len,
                         uint8_t *payload, uint16_t *payload_len)
{
    while (*len >= 6)
    {
        /* 找帧头 */
        if (buf[0] != SEASKY_SOF)
        {
            memmove(buf, buf + 1, --(*len));
            continue;
        }

        uint16_t plen = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
        if (plen > SEASKY_MAX_PAYLOAD)
        {
            /* 假帧头, 丢弃继续搜 */
            memmove(buf, buf + 1, --(*len));
            continue;
        }

        uint16_t total = 6 + plen;
        if (*len < total)
            return -1;   /* 半帧, 等更多数据 */

        uint16_t crc = (uint16_t)buf[4 + plen] | ((uint16_t)buf[5 + plen] << 8);
        if (seasky_crc16(buf, 4 + plen) != crc)
        {
            memmove(buf, buf + 1, --(*len));
            continue;
        }

        uint8_t id = buf[3];
        if (payload != NULL && plen > 0)
            memcpy(payload, &buf[4], plen);
        if (payload_len != NULL)
            *payload_len = plen;

        /* 消费该帧 */
        *len -= total;
        memmove(buf, buf + total, *len);
        return id;
    }
    return -1;
}
