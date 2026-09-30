/**
 * @file seasky.h
 * @brief Seasky V1 视觉协议(帧解析)
 *
 * 帧结构: 0xA5 | Len(2B LE) | MsgID(1B) | Payload | CRC16(2B LE)
 * CRC-16/Modbus 覆盖前 4+Len 字节(不含 CRC 本身)。
 */
#ifndef INF_SEASKY_H
#define INF_SEASKY_H

#include <stdint.h>

#define SEASKY_SOF           0xA5
#define SEASKY_MAX_PAYLOAD   64

/* 下行(视觉→电控)消息 ID */
#define MSG_ID_FIX_CTRL      0x05

uint16_t seasky_crc16(const uint8_t *data, uint16_t len);

/* 从缓冲区弹出并校验一帧: 成功返回 msg_id(>0), 半帧返回 -1, 假帧头/CRC错丢弃后继续 */
int16_t seasky_pop_frame(uint8_t *buf, uint16_t *len,
                         uint8_t *payload, uint16_t *payload_len);

#endif /* INF_SEASKY_H */
