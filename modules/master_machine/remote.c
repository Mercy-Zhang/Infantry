/**
 * @file remote.c
 * @brief 遥控器适配层实现
 *
 * 底层用搬来的 remote_control(SBUS/DBUS 解析), 这里把 rc_data 映射到
 * 应用层统一使用的 g_input.remote, 并做约定转换:
 *   摇杆 -660~660 → -1000~1000; 开关 RC_SW_*(1/2/3) → 0下/1中/2上。
 */
#include "remote.h"
#include "remote_control.h"
#include "robot_def.h"

static RC_ctrl_t *rc_data;

void RemoteInit(void)
{
    /* SBUS/DBUS 接收机接 UART5(与哨兵一致), 内部自动启动 DMA+IDLE 接收 */
    rc_data = RemoteControlInit(&huart5);
}

/* RC_SW_UP(1)/MID(3)/DOWN(2) → 我方 0下/1中/2上 (仅 SBUS/天地飞 使用) */
#ifdef SBUS
static uint8_t map_switch(uint8_t rc_sw)
{
    if (rc_sw == RC_SW_UP)   return 2;
    if (rc_sw == RC_SW_DOWN) return 0;
    return 1; /* 中 */
}
#endif

/* -660~660 → -1000~1000 */
static int16_t map_joy(int16_t v)
{
    return (int16_t)((int32_t)v * 1000 / 660);
}

void RemoteTask(void)
{
    if (rc_data == NULL)
        return;

    // 遥控器通用    
    /* 右手 → 云台(yaw/pitch), 左手 → 底盘(前后/横移) */
    g_input.remote.gimbal_yaw   = map_joy(rc_data[0].rc.rocker_r_);   /* 右手水平 ch[0] */
    g_input.remote.gimbal_pitch = map_joy(rc_data[0].rc.rocker_r1);   /* 右手竖直 ch[1] */
    g_input.remote.chassis_x    = map_joy(rc_data[0].rc.rocker_l1);   /* 左手竖直 ch[2] */
    g_input.remote.chassis_y    = map_joy(rc_data[0].rc.rocker_l_);   /* 左手水平 ch[3] */
    g_input.remote.wheel        = map_joy(rc_data[0].rc.dial);

    /* 两个物理开关直接透传 */
    g_input.remote.switch_left  = rc_data[0].rc.switch_left;
    g_input.remote.switch_right = rc_data[0].rc.switch_right;

#ifdef SBUS
    /* 天地飞(SBUS): 6 开关 → 逻辑开关字段 sw_*。
     * DT7(DBUS) 不用这些 sw_* 字段, 底盘/云台/发射直接读上面的 switch_left/switch_right/wheel。 */
    g_input.remote.sw_estop    = map_switch(rc_data[0].rc.switch_estop);
    g_input.remote.sw_mode     = map_switch(rc_data[0].rc.switch_mode);
    g_input.remote.sw_friction = map_switch(rc_data[0].rc.switch_friction);
    g_input.remote.sw_loader   = map_switch(rc_data[0].rc.switch_loader);
#endif

    g_input.remote.online = RemoteControlIsOnline();
}
