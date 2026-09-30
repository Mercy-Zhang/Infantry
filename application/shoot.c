/**
 * @file shoot.c
 * @brief 发射控制(移植自哨兵 shoot.c)
 *
 * 两段式发射: 先开摩擦轮, 起转到位(friction_ready)后才允许拨盘动,
 * 防止摩擦轮没起速就推弹导致卡弹/打不远。
 *
 * 控制源:
 *   摩擦轮: SA上(sw_friction==2) 或 视觉在线(自瞄常转) → 摩擦轮起转
 *   拨盘  : SB下=待机, SB中=单发(进入"中"的上升沿拨一发), SB上=连发
 *   卡弹  : 拨盘扭矩超阈值 → 反转退弹, 时间到恢复
 */
#include "shoot.h"
#include "dji_motor.h"
#include "robot_def.h"
#include "bsp_dwt.h"

static DJIMotorInstance *friction_l, *friction_r;
static DJIMotorInstance *loader;   /* M2006 */

/* 卡弹反转参数(需实测标定) */
#define JAM_CURRENT_THRESHOLD  6000.0f  /* 卡弹电流阈值(M2006 原始 int16) */
#define REVERSE_SPEED    -800.0f       /* 反转速度 °/s(转子, 负=反转) */
#define REVERSE_TIME     500.0f        /* 反转持续时间 ms */

/* 拨盘发弹参数(Ozone 可调, 需按机械实测标定) */
volatile float loader_fire_speed = 12000.0f;  /* 拨盘转速 °/s(转子, M2006) */
volatile float single_shot_ms = 80.0f;       /* 单发一发拨盘转动时间 ms */
volatile float friction_spinup_ms = 150.0f;  /* 摩擦轮起转到全速时间 ms */

/* 拨盘状态机 */
static enum {
    LOADER_NORMAL,
    LOADER_REVERSING,
} loader_state = LOADER_NORMAL;
static float reverse_start_time = 0;

/* 单发状态机: 每次触发上升沿只拨一发 */
static uint8_t single_shot_pending = 0;  /* 已登记一发, 等摩擦轮到位 */
static uint8_t single_shot_active = 0;   /* 单发进行中 */
static float   single_shot_start = 0;

/* 两段式: 先开摩擦轮, 到位后再拨弹 */
static uint8_t friction_was_on = 0;
static float   friction_on_start = 0;

/* 触发上升沿检测 */
static uint8_t loader_was_single = 0;   /* SB 上一拍是否处于"中" */
static uint8_t last_shoot = 0;          /* 视觉 shoot 脉冲上一拍 */

/* ============================ 初始化 ============================ */

void ShootInit(void)
{
    Motor_Init_Config_s friction_config = {
        .can_init_config = {.can_handle = &hcan2},
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20.0f, .Ki = 0.0f, .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000.0f,
                .MaxOut = 30000.0f, .MaxOut_ = -30000.0f,
            },
            .current_PID = {
                .Kp = 1.1f, .Ki = 0.0f, .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000.0f,
                .MaxOut = 30000.0f, .MaxOut_ = -30000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    friction_config.can_init_config.tx_id = CFG_FRICTION_L_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = CFG_FRICTION_R_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    /* 拨盘 M2006 — DJI 速度环(哨兵注释里的 M2006 配置) */
    Motor_Init_Config_s loader_config = {
        .can_init_config = {.can_handle = &hcan1, .tx_id = CFG_LOADER_M2006_ID},
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10.0f, .Ki = 0.0f, .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f,
                .MaxOut = 9000.0f, .MaxOut_ = -9000.0f,
            },
            .current_PID = {
                .Kp = 0.7f, .Ki = 0.0f, .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f,
                .MaxOut = 9000.0f, .MaxOut_ = -9000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,   /* 上机标定 */
        },
        .motor_type = M2006,
    };
    loader = DJIMotorInit(&loader_config);
    DJIMotorSetRef(loader, 0);
}

/* ============================ 控制原语 ============================ */

/* 摩擦轮是否应开启:
 *   SA上(手动开摩擦轮), 或 视觉模式(SE中)+视觉在线(自瞄常转, 避免 150ms 起转延时打不准) */
#ifndef DBUS
static uint8_t FrictionOn(void)
{
    return (g_input.remote.sw_friction == 2) ||
           (g_input.remote.sw_mode == 1 && g_input.vision.online);
}
#endif

/* ============================ 任务入口 ============================ */

void ShootTask(void)
{
    /* ====== 输入映射: DT7(DBUS) 与 天地飞(SBUS) 的开关来源不同 ====== */
#ifdef DBUS
    /* DT7(DBUS): S1上=急停; 拨轮向上=开摩擦轮+连发, 向下=急停 */
    uint8_t estop_active = (g_input.remote.switch_left == 1) || (g_input.remote.wheel < -450);
    uint8_t friction_on  = (g_input.remote.wheel > 450);
    uint8_t loader_mode  = friction_on ? 2 : 0;   /* 2=连发, 0=待机 */
#else
    uint8_t estop_active = (g_input.remote.sw_estop == 2);
    uint8_t friction_on  = FrictionOn();
    uint8_t loader_mode  = g_input.remote.sw_loader;   /* 0下=待机 1中=单发 2上=连发 */
#endif

    /* ====== 急停: 摩擦轮+拨盘全停 ====== */
    if (estop_active)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorSetRef(loader, 0);
        single_shot_pending = 0;
        single_shot_active = 0;
        loader_state = LOADER_NORMAL;
        friction_was_on = 0;
        return;
    }

    /* ====== 单发触发锁存 ======
     * SB进入"中"的上升沿 → 登记一发; 视觉 shoot 脉冲上升沿 → 登记一发。
     * 锁存后在摩擦轮到位前保持, 不丢失。 */
    uint8_t loader_single = (loader_mode == 1);
    if (loader_single && !loader_was_single)
        single_shot_pending = 1;
    loader_was_single = loader_single;

    if (g_input.vision.shoot && !last_shoot)
        single_shot_pending = 1;
    last_shoot = g_input.vision.shoot;

    /* ====== 卡弹回转保护 ====== */
    if (loader_state == LOADER_REVERSING)
    {
        DJIMotorSetRef(loader, REVERSE_SPEED);
        if (DWT_GetTimeline_ms() - reverse_start_time > REVERSE_TIME)
            loader_state = LOADER_NORMAL;
        return;
    }
    if (loader->measure.real_current > JAM_CURRENT_THRESHOLD)
    {
        loader_state = LOADER_REVERSING;
        reverse_start_time = DWT_GetTimeline_ms();
        DJIMotorSetRef(loader, REVERSE_SPEED);
        return;
    }

    /* ====== 摩擦轮: 记录起转时间(两段式用) ====== */
    if (friction_on && !friction_was_on)
        friction_on_start = DWT_GetTimeline_ms();
    friction_was_on = friction_on;

    if (friction_on)
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorSetRef(friction_l, CFG_FRICTION_SPEED);
        DJIMotorSetRef(friction_r, CFG_FRICTION_SPEED);
    }
    else
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
    }

    /* 摩擦轮是否已起转到位(两段式延时) */
    uint8_t friction_ready = friction_on &&
        (DWT_GetTimeline_ms() - friction_on_start >= friction_spinup_ms);

    /* ====== 拨盘: 摩擦轮到位后才允许拨弹 ====== */
    uint8_t loader_run = 0;

    if (friction_on)
    {
        if (loader_mode == 2)   /* SB上 = 连发 */
        {
            loader_run = friction_ready;
            single_shot_pending = 0;
            single_shot_active = 0;
        }
        else                    /* SB中/下 = 单发/待机 */
        {
            if (single_shot_pending && !single_shot_active && friction_ready)
            {
                single_shot_pending = 0;
                single_shot_active = 1;
                single_shot_start = DWT_GetTimeline_ms();
            }

            if (single_shot_active)
            {
                if (DWT_GetTimeline_ms() - single_shot_start < single_shot_ms)
                    loader_run = 1;
                else
                    single_shot_active = 0;
            }
        }
    }
    else
    {
        /* 摩擦轮关: 清掉单发请求, 拨盘不动(开拨盘先开摩擦轮的互锁) */
        single_shot_pending = 0;
        single_shot_active = 0;
    }
    DJIMotorSetRef(loader, loader_run ? loader_fire_speed : 0);
}
