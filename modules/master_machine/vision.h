/**
 * @file vision.h
 * @brief 视觉通信(Seasky): 接收解析 → 刷新 g_input.vision
 */
#ifndef INF_VISION_H
#define INF_VISION_H

#include <stdint.h>

void VisionInit(void);

/* 由 UART/VCP 接收回调调用, 喂字节流 */
void VisionPushBytes(const uint8_t *data, uint16_t len);

/* 周期调用(100Hz): 解析帧 + 刷新 g_input.vision(含在线检测) */
void VisionTask(void);

#endif /* INF_VISION_H */
