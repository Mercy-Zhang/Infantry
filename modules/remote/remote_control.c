#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

// 遥控器接收的 buffer 大小(按协议区分)
#ifdef SBUS
#define REMOTE_CONTROL_FRAME_SIZE SBUS_FRAME_SIZE // 25 字节, SBUS 一帧
#else
#define REMOTE_CONTROL_FRAME_SIZE 18u // 18 字节, DBUS 一帧
#endif

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

#ifdef DBUS
/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}
#endif

#ifdef SBUS
/* 每个比例通道的实测标定值, 按 ch[] 下标索引(即 ch[0]=CH1 ... ch[15]=CH16).
 * 四个摇杆实测范围各不相同, 所以每个通道单独填三档:
 *   center = 中立原始值(通常都是 1024)
 *   min    = 推到一端的原始值
 *   max    = 推到另一端的原始值
 * ch[0..3] 为 Ozone 实测值; 其余通道未使用, 按占位值保留. */
typedef struct
{
    uint16_t center;
    uint16_t min;
    uint16_t max;
} sbus_ch_calib_t;

static const sbus_ch_calib_t sbus_ch_calib[SBUS_CHANNEL_NUM] = {
    {1021, 118, 1927}, // ch[0]  = CH1 右摇杆水平 (实测)
    {1024, 118, 1929}, // ch[1]  = CH2 右摇杆竖直 (实测)
    {1024, 17, 2030},  // ch[2]  = CH3 左摇杆竖直/油门 (实测: 行程明显比其他杆大, 接近 11bit 满量程)
    {1024, 118, 1929}, // ch[3]  = CH4 左摇杆水平 (实测)
    {1024, 18, 1919}, // ch[4]  = CH5 SG 两档杆
    {1024, 18, 1919}, // ch[5]  = CH6
    {1024, 18, 1919}, // ch[6]  = CH7
    {1024, 18, 1919}, // ch[7]  = CH8
    {1024, 18, 1919}, // ch[8]  = CH9
    {1024, 18, 1919}, // ch[9]  = CH10
    {1024, 18, 1919}, // ch[10] = CH11
    {1024, 18, 1919}, // ch[11] = CH12
    {1024, 18, 1919}, // ch[12] = CH13
    {1024, 18, 1919}, // ch[13] = CH14
    {1024, 18, 1919}, // ch[14] = CH15
    {1024, 18, 1919}, // ch[15] = CH16
};

/**
 * @brief SBUS 比例通道 → 摇杆值, 映射到 ±660
 * @note SBUS_JOY_UNIFORM=1 时: 中心用各通道实测值, 有效半行程统一,
 *       四个摇杆输出曲线完全一致且对称; 超出有效范围的原始值钳位到 ±660.
 *       SBUS_JOY_UNIFORM=0 时: 按各通道自己的 min/max 满行程映射.
 */
static int16_t sbus_joystick(uint16_t raw, const sbus_ch_calib_t *cal)
{
    int32_t v = (int32_t)raw - cal->center;
    int32_t denom;

    if (v == 0)
        return 0; // 中立(或未分配通道恒为 center)直接返回 0, 避免 0/0
#if SBUS_JOY_UNIFORM
    /* 统一对称模式: 所有比例通道共用同一有效半行程, 增益完全一致 */
    denom = (int32_t)SBUS_JOY_UNIFORM_HALF_RANGE;
    v = v * SBUS_JOY_MAX / denom;
#else
    if (v > 0)
    {
        denom = (int32_t)cal->max - cal->center;
        if (denom <= 0)
            return 0;
        v = v * SBUS_JOY_MAX / denom;
    }
    else
    {
        denom = (int32_t)cal->center - cal->min;
        if (denom <= 0)
            return 0;
        v = v * SBUS_JOY_MAX / denom;
    }
#endif
    if (v > SBUS_JOY_MAX)
        v = SBUS_JOY_MAX;
    if (v < -SBUS_JOY_MAX)
        v = -SBUS_JOY_MAX;
    // 死区: 归一化后 |v| 在死区范围内一律置 0(消除回中抖动/量化噪声, 防漂移)
    if (v >= -SBUS_JOY_DEADBAND && v <= SBUS_JOY_DEADBAND)
        return 0;
    return (int16_t)v;
}

/**
 * @brief SBUS 三段开关 → 上/中/下
 * @note 实测方向: 上=2030(高) / 中=1024 / 下=17(低),
 *       所以 17 判为 DOWN、2030 判为 UP.
 *
 */
static uint8_t sbus_switch3(uint16_t raw)
{
    if (raw < RC_CH_VALUE_OFFSET - SBUS_SW_THRESHOLD) // 17 → 下
        return RC_SW_DOWN;
    if (raw > RC_CH_VALUE_OFFSET + SBUS_SW_THRESHOLD) // 2030 → 上
        return RC_SW_UP;
    return RC_SW_MID;
}

/**
 * @brief SBUS 两段开关 → 上/下 (急停开关用)
 * @note 实测方向: 上=2030(高) / 下=17(低). raw > 1024 判为 UP.
 *
 */
static uint8_t sbus_switch2(uint16_t raw)
{
    return (raw > RC_CH_VALUE_OFFSET) ? RC_SW_UP : RC_SW_DOWN;
}

#ifdef SBUS_CALIB_ENABLE
/* ==================== 摇杆自动标定(死区+范围) ====================
 * 使用流程:
 *   1) 确保 RemoteControlInit 已调用、接收机正常出数据;
 *   2) 调用 RemoteControlStartCalib() 启动(可临时放在 RobotInit 或调试器里调用);
 *   3) 按 RTT 提示: 四个摇杆回中保持 1 秒 → 满行程来回推拉 2 秒;
 *   4) 结束后打印 {center, min, max} 表格, 直接粘贴进 sbus_ch_calib[].
 * 帧率按 ~70Hz(1000/14)估算, 若接收频率不同请调整下方 *_FRAMES. */
typedef enum
{
    RC_CALIB_IDLE = 0,
    RC_CALIB_CENTER, /* 回中采集 center */
    RC_CALIB_RANGE,  /* 满行程采集 min/max */
    RC_CALIB_DONE,
} rc_calib_state_e;

#define RC_CALIB_CENTER_FRAMES 70u  /* 回中保持时间 ≈ 1s @70Hz */
#define RC_CALIB_RANGE_FRAMES 140u  /* 满行程推拉时间 ≈ 2s @70Hz */
#define RC_CALIB_CH_NUM 4u          /* 只标定 4 个摇杆通道 ch[0..3] */

static rc_calib_state_e rc_calib_state = RC_CALIB_IDLE;
static uint32_t rc_calib_cnt = 0;
static uint32_t rc_calib_center_sum[RC_CALIB_CH_NUM];
static uint16_t rc_calib_min[RC_CALIB_CH_NUM];
static uint16_t rc_calib_max[RC_CALIB_CH_NUM];

void RemoteControlStartCalib()
{
    rc_calib_state = RC_CALIB_CENTER;
    rc_calib_cnt = 0;
    for (uint8_t i = 0; i < RC_CALIB_CH_NUM; i++)
        rc_calib_center_sum[i] = 0;
    /* 标定属于显式诊断模式, 直接走 RTT, 不受 DISABLE_LOG_SYSTEM 影响 */
    SEGGER_RTT_printf(BUFFER_INDEX, "[rc] 标定开始: 四个摇杆回中并保持 1 秒...\r\n");
}

static void RCCalibUpdate(const uint16_t *ch)
{
    if (rc_calib_state == RC_CALIB_IDLE || rc_calib_state == RC_CALIB_DONE)
        return;
    rc_calib_cnt++;
    if (rc_calib_state == RC_CALIB_CENTER)
    {
        for (uint8_t i = 0; i < RC_CALIB_CH_NUM; i++)
            rc_calib_center_sum[i] += ch[i];
        if (rc_calib_cnt >= RC_CALIB_CENTER_FRAMES)
        {
            for (uint8_t i = 0; i < RC_CALIB_CH_NUM; i++)
            {
                rc_calib_min[i] = 0xFFFF;
                rc_calib_max[i] = 0;
            }
            SEGGER_RTT_printf(BUFFER_INDEX, "[rc] 回中采集完成, 请四个摇杆满行程来回推拉 2 秒...\r\n");
            rc_calib_state = RC_CALIB_RANGE;
            rc_calib_cnt = 0;
        }
    }
    else if (rc_calib_state == RC_CALIB_RANGE)
    {
        for (uint8_t i = 0; i < RC_CALIB_CH_NUM; i++)
        {
            if (ch[i] < rc_calib_min[i])
                rc_calib_min[i] = ch[i];
            if (ch[i] > rc_calib_max[i])
                rc_calib_max[i] = ch[i];
        }
        if (rc_calib_cnt >= RC_CALIB_RANGE_FRAMES)
        {
            static const char *const ch_name[RC_CALIB_CH_NUM] = {
                "右摇杆水平", "右摇杆竖直", "左摇杆竖直", "左摇杆水平"};
            SEGGER_RTT_printf(BUFFER_INDEX,
                              "[rc] 标定完成! 将以下表格粘贴到 remote_control.c 的 sbus_ch_calib[]:\r\n");
            for (uint8_t i = 0; i < RC_CALIB_CH_NUM; i++)
            {
                uint16_t center = (uint16_t)(rc_calib_center_sum[i] / RC_CALIB_CENTER_FRAMES);
                SEGGER_RTT_printf(BUFFER_INDEX, "    {%u, %u, %u}, // ch[%u] = CH%u %s\r\n",
                                  center, rc_calib_min[i], rc_calib_max[i], i, i + 1, ch_name[i]);
            }
            rc_calib_state = RC_CALIB_DONE;
        }
    }
}
#endif /* SBUS_CALIB_ENABLE */
#endif

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
#ifdef DBUS // DBUS通信协议
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
#endif

#ifdef SBUS // SBUS通信协议 (天地飞 ET16S 的 W.BUS, 兼容 Futaba S.BUS)

    // 帧校验: 帧头 0x0F + 帧尾 0x00. 上电首帧或干扰可能导致 DMA 未对齐到帧头,
    // 此时直接丢弃这一帧, 等待下一帧重新对齐即可.
    if (sbus_buf[0] != SBUS_CH_START || sbus_buf[SBUS_FRAME_SIZE - 1] != SBUS_CH_END)
        return;

    // 1. 通用解码: 22 字节连续比特流 → 16 通道 × 11bit, 原始值 0~2047
    rc_ctrl[TEMP].rc.ch[0]  = ((sbus_buf[1]  | (sbus_buf[2] << 8))                             & 0x07ff);
    rc_ctrl[TEMP].rc.ch[1]  = (((sbus_buf[2] >> 3) | (sbus_buf[3] << 5))                       & 0x07ff);
    rc_ctrl[TEMP].rc.ch[2]  = (((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) & 0x07ff);
    rc_ctrl[TEMP].rc.ch[3]  = (((sbus_buf[5] >> 1) | (sbus_buf[6] << 7))                       & 0x07ff);
    rc_ctrl[TEMP].rc.ch[4]  = (((sbus_buf[6] >> 4) | (sbus_buf[7] << 4))                       & 0x07ff);
    rc_ctrl[TEMP].rc.ch[5]  = (((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9))  & 0x07ff);
    rc_ctrl[TEMP].rc.ch[6]  = (((sbus_buf[9] >> 2) | (sbus_buf[10] << 6))                      & 0x07ff);
    rc_ctrl[TEMP].rc.ch[7]  = (((sbus_buf[10] >> 5) | (sbus_buf[11] << 3))                     & 0x07ff);
    rc_ctrl[TEMP].rc.ch[8]  = ((sbus_buf[12] | (sbus_buf[13] << 8))                            & 0x07ff);
    rc_ctrl[TEMP].rc.ch[9]  = (((sbus_buf[13] >> 3) | (sbus_buf[14] << 5))                     & 0x07ff);
    rc_ctrl[TEMP].rc.ch[10] = (((sbus_buf[14] >> 6) | (sbus_buf[15] << 2) | (sbus_buf[16] << 10)) & 0x07ff);
    rc_ctrl[TEMP].rc.ch[11] = (((sbus_buf[16] >> 1) | (sbus_buf[17] << 7))                     & 0x07ff);
    rc_ctrl[TEMP].rc.ch[12] = (((sbus_buf[17] >> 4) | (sbus_buf[18] << 4))                     & 0x07ff);
    rc_ctrl[TEMP].rc.ch[13] = (((sbus_buf[18] >> 7) | (sbus_buf[19] << 1) | (sbus_buf[20] << 9)) & 0x07ff);
    rc_ctrl[TEMP].rc.ch[14] = (((sbus_buf[20] >> 2) | (sbus_buf[21] << 6))                     & 0x07ff);
    rc_ctrl[TEMP].rc.ch[15] = (((sbus_buf[21] >> 5) | (sbus_buf[22] << 3))                     & 0x07ff);

    // 2. 映射到逻辑字段, 通道号在 remote_control.h 里的 SBUS_CH_* 宏中修改
    rc_ctrl[TEMP].rc.rocker_l_   = sbus_joystick(rc_ctrl[TEMP].rc.ch[SBUS_CH_ROCKER_L_],  &sbus_ch_calib[SBUS_CH_ROCKER_L_]);
    rc_ctrl[TEMP].rc.rocker_l1   = sbus_joystick(rc_ctrl[TEMP].rc.ch[SBUS_CH_ROCKER_L1], &sbus_ch_calib[SBUS_CH_ROCKER_L1]);
    rc_ctrl[TEMP].rc.rocker_r_   = sbus_joystick(rc_ctrl[TEMP].rc.ch[SBUS_CH_ROCKER_R_],  &sbus_ch_calib[SBUS_CH_ROCKER_R_]);
    rc_ctrl[TEMP].rc.rocker_r1   = sbus_joystick(rc_ctrl[TEMP].rc.ch[SBUS_CH_ROCKER_R1], &sbus_ch_calib[SBUS_CH_ROCKER_R1]);
    rc_ctrl[TEMP].rc.dial        = sbus_joystick(rc_ctrl[TEMP].rc.ch[SBUS_CH_DIAL],      &sbus_ch_calib[SBUS_CH_DIAL]);
    // 急停=两段开关, 模式/摩擦轮/拨盘=三段开关
    rc_ctrl[TEMP].rc.switch_estop    = sbus_switch2(rc_ctrl[TEMP].rc.ch[SBUS_CH_SW_ESTOP]);
    rc_ctrl[TEMP].rc.switch_mode     = sbus_switch3(rc_ctrl[TEMP].rc.ch[SBUS_CH_SW_MODE]);
    rc_ctrl[TEMP].rc.switch_friction = sbus_switch3(rc_ctrl[TEMP].rc.ch[SBUS_CH_SW_FRICTION]);
    rc_ctrl[TEMP].rc.switch_loader   = sbus_switch3(rc_ctrl[TEMP].rc.ch[SBUS_CH_SW_LOADER]);

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据

#ifdef SBUS_CALIB_ENABLE
    RCCalibUpdate(rc_ctrl[TEMP].rc.ch); // 摇杆自动标定(仅在启动后工作)
#endif
#endif
}
/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}