/**
 * @file vision.c
 * @brief 视觉通信实现
 */
#include "vision.h"
#include "robot_def.h"
#include "seasky.h"
#include "bsp_dwt.h"
#include <string.h>

#pragma pack(1)
typedef struct
{
    uint8_t flags;   /* bit0=锁敌 */
    float yaw;
    float pitch;
    uint8_t fire;
    float vx, vy, vz;
} Vision_Fix_Ctrl_s;
#pragma pack()

#define VISION_RX_BUF_SIZE (SEASKY_MAX_PAYLOAD + 32)
#define VISION_ONLINE_TIMEOUT_MS 500

static uint8_t rx_buf[VISION_RX_BUF_SIZE];
static uint16_t rx_len = 0;
static uint32_t last_frame_ms = 0;

void VisionInit(void)
{
    rx_len = 0;
    last_frame_ms = 0;
    memset(&g_input.vision, 0, sizeof(g_input.vision));
}

void VisionPushBytes(const uint8_t *data, uint16_t len)
{
    if (rx_len + len > VISION_RX_BUF_SIZE)
        rx_len = 0;   /* 溢出丢弃, 重新同步 */
    memcpy(&rx_buf[rx_len], data, len);
    rx_len += len;
}

void VisionTask(void)
{
    uint8_t payload[SEASKY_MAX_PAYLOAD];
    uint16_t plen;
    int16_t id;

    while ((id = seasky_pop_frame(rx_buf, &rx_len, payload, &plen)) > 0)
    {
        if (id == MSG_ID_FIX_CTRL && plen >= sizeof(Vision_Fix_Ctrl_s))
        {
            Vision_Fix_Ctrl_s *fix = (Vision_Fix_Ctrl_s *)payload;
            g_input.vision.target_state = (fix->flags & 0x01) ? VISION_READY : VISION_NO_TARGET;
            g_input.vision.yaw = fix->yaw;
            g_input.vision.pitch = fix->pitch;
            g_input.vision.shoot = fix->fire;
        }
        last_frame_ms = DWT_GetTimeline_ms();
    }

    g_input.vision.online =
        (DWT_GetTimeline_ms() - last_frame_ms) < VISION_ONLINE_TIMEOUT_MS;
}
