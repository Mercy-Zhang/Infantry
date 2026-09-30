/**
 * @file gimbal.c
 * @brief 云台控制(单 yaw + 单 pitch)
 *
 * yaw  : GM6020 — 内置串级 PID(角度环 ANGLE_LOOP + 速度环 SPEED_LOOP),
 *        反馈走 IMU OTHER_FEED: 角度=YawTotalAngle(°), 速度=Gyro[2](rad/s),
 *        参数移植自 Lao_Hero。
 * pitch: DM4310 POSVEL, 原生位置速度模式, 直接发位置指令。
 *
 * 模式(拨杆 + 急停, 两态活跃):
 *   DISABLE 急停: 两轴停
 *   FOLLOW  遥控: yaw 摇杆 → 累加目标角(角度环跟随+松杆自稳); pitch 摇杆 → 位置
 *
 * 视觉自瞄目前未接: 枚举 GIMBAL_STATE_VISION 与 Vision_t / CFG_VISION_* 配置
 * 作为骨架保留, handler 尚未实现, VISION 档暂时退化为 FOLLOW(见 GimbalTask)。
 */
#include "gimbal.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "robot_def.h"
#include "general_def.h"
#include "daemon.h"

static DJIMotorInstance *yaw_motor;    /* GM6020, 内置角度+速度环(IMU 反馈) */
static DMMotorInstance *pitch_motor;   /* DM4310, POSVEL */
static attitude_t *imu;

static gimbal_state_e state = GIMBAL_STATE_DISABLE;

/* FOLLOW 摇杆 → 目标角增量: 满偏 ≈ YAW_FOLLOW_MAX_RATE °/s(RobotTask 100Hz) */
#define YAW_FOLLOW_MAX_RATE   800.0f
#define YAW_FOLLOW_ANGLE_STEP (YAW_FOLLOW_MAX_RATE / 100.0f)

/* 枪口目标角(°, 世界系): 使能时锁存当前角, 之后由摇杆累加 */
static float gun_yaw_ref = 0.0f;
static uint8_t gun_yaw_ref_locked = 0;

/* 视觉锁敌子状态(视觉未接, 当前恒 0, 待接视觉时启用) */
static uint8_t vision_ever_locked = 0;  /* 本次视觉会话是否锁过敌 */

/* ============================底盘云台角度偏移量函数============================ */

/* 底盘 yaw(°) = 云台 IMU yaw - yaw 电机相对底盘角度, 供底盘航向保持 */
float GimbalGetChassisYawDeg(void)
{
    if (imu == NULL || yaw_motor == NULL)
        return 0.0f;
    return imu->YawTotalAngle - yaw_motor->measure.total_angle;   /* 方向需上机标定 */
}

/* 云台 yaw 电机相对底盘的角度(°), 即 offset_angle, 供底盘跟随云台/坐标旋转 */
float GimbalGetYawOffsetDeg(void)
{
    if (yaw_motor == NULL)
        return 0.0f;
    return yaw_motor->measure.total_angle;   /* 方向需上机标定 */
}

/* ============================ 初始化 ============================ */

void GimbalInit(void)
{
    /* 先取 IMU 静态指针(幂等), 供 yaw 反馈指针使用 */
    imu = INS_Init();

    /* yaw GM6020 — 内置串级 PID(角度+速度环), 反馈走 IMU OTHER_FEED,
     * 参数移植自 Lao_Hero; 底盘反旋前馈暂关 */
    Motor_Init_Config_s yaw_cfg = {
        .can_init_config = {.can_handle = &hcan1, .tx_id = CFG_YAW_GM6020_ID},
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 60.0f, .Ki = 0.0f, .Kd = 2.0f,
                .DeadBand = 0.1f,
                .Derivative_LPF_RC = 0.001f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
                         | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .IntegralLimit = 100.0f,
                .MaxOut = 900.0f, .MaxOut_ = -900.0f,
            },
            .speed_PID = {
                .Kp = 180.0f, .Ki = 5.0f, .Kd = 0.0f,
                .Output_LPF_RC = 0.005f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
                         | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 5000.0f,
                .MaxOut = 30000.0f, .MaxOut_ = -30000.0f,
            },
            .other_angle_feedback_ptr = &imu->YawTotalAngle,
            .other_speed_feedback_ptr = &imu->Gyro[2],
            /* speed_feedforward_ptr 不设 —— 前馈关 */
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,  /* 与 Lao_Hero 一致, 上机标定 */
            .feedforward_flag = FEEDFORWARD_NONE,          /* 底盘反旋前馈暂关 */
        },
        .motor_type = GM6020,
    };
    yaw_motor = DJIMotorInit(&yaw_cfg);

    /* pitch DM4310 — POSVEL 原生位置速度模式 */
    Motor_Init_Config_s pitch_cfg = {
        .can_init_config = {.can_handle = &hcan2,
                            .tx_id = CFG_PITCH_DM_ID,
                            .rx_id = 0x10 + CFG_PITCH_DM_ID},
    };
    pitch_motor = DMMotorInit(&pitch_cfg);
    pitch_motor->p_max = 3.14159265f;
    pitch_motor->v_max = 30.0f;
    pitch_motor->t_max = 18.0f;
    pitch_motor->native_mode = DM_NATIVE_MODE_POSVEL;
    DMMotorSetRef(pitch_motor, CFG_PITCH_HORIZON_RAD);

    gun_yaw_ref_locked = 0;
}

/* ============================ 状态迁移 ============================ */

/* 拨杆 → 控制模式 单一真值源
 * 视觉档(GIMBAL_STATE_VISION)当前退化为 FOLLOW, 见 GimbalTask。 */
static gimbal_state_e GimbalDecode(void)
{
#ifdef SBUS
    if (g_input.remote.sw_estop == 2)     return GIMBAL_STATE_DISABLE;   /* 急停(下) */
    if (g_input.remote.sw_mode == 1)      return GIMBAL_STATE_VISION;    /* 中=视觉档 */
#endif
#ifdef DBUS
    if (g_input.remote.switch_left == 1)  return GIMBAL_STATE_DISABLE;   /* 急停(上) */
    // if (g_input.remote.switch_left == 3)  return GIMBAL_STATE_VISION;    /* 视觉档 */
    if (g_input.remote.switch_left == 3)  return GIMBAL_STATE_FOLLOW;    /* 中=遥控 */
    if (g_input.remote.switch_left == 2)  return GIMBAL_STATE_FOLLOW;    /* 下=遥控 */
#endif
    return GIMBAL_STATE_FOLLOW;
}

/* ============================ 各状态处理 ============================ */

/* 状态 FOLLOW: 摇杆累加目标角 → 内置角度环; 松杆角度环自稳锁最后目标角 */
static void GimbalHandleFollow(void)
{
    if (imu == NULL)
        return;

    /* 摇杆线性映射: -1~1, 中位死区防漂移 */
    float stick = -(float)g_input.remote.gimbal_yaw / 1000.0f;  /* 负号方向上机标定 */
    if (stick > -0.03f && stick < 0.03f) stick = 0.0f;

    /* 使能时锁存当前角, 防止目标跳变 */
    if (!gun_yaw_ref_locked)
    {
        gun_yaw_ref = imu->YawTotalAngle;
        gun_yaw_ref_locked = 1;
    }
    gun_yaw_ref += YAW_FOLLOW_ANGLE_STEP * stick;   /* 摇杆累加(°), 角度环跟随 */
    DJIMotorSetRef(yaw_motor, gun_yaw_ref);
}

/* Pitch 轴: POSVEL 原生位置速度模式, 摇杆/视觉映射到角度(独立于 yaw 状态) */
static void GimbalHandlePitch(uint8_t vision_mode, uint8_t vision_locked)
{
    if (pitch_motor == NULL)
        return;

    static uint16_t startup_cnt = 0;
    static float    pos_ref = 0.0f;

    /* 上电/掉线: 保持当前位置, 防止 POSVEL 乱跳 */
    if (DaemonIsOnline(pitch_motor->motor_daemon) == 0)
    {
        startup_cnt = 0;
        pos_ref = pitch_motor->measure.position;
        DMMotorSetRef(pitch_motor, pos_ref);
        return;
    }
    startup_cnt++;
    if (startup_cnt < 50)  /* 上电后前 50 拍保持当前位 */
    {
        pos_ref = pitch_motor->measure.position;
        DMMotorSetRef(pitch_motor, pos_ref);
        return;
    }

    if (vision_mode)
    {
        if (vision_locked)
        {
            /* 锁敌: 跟随视觉 pitch(deg→rad), 异常数据保持 */
            float vp = g_input.vision.pitch;
            if (vp > -90.0f && vp < 90.0f)
                pos_ref = CFG_VISION_PITCH_HORIZON_RAD + vp * DEGREE_2_RAD * CFG_VISION_PITCH_DIR;
            else
                pos_ref = pitch_motor->measure.position;
        }
        else
        {
            /* 未锁敌: 首次进视觉回水平, 锁后丢失保持当前位置, 防跳变 */
            pos_ref = vision_ever_locked ? pitch_motor->measure.position : CFG_VISION_PITCH_HORIZON_RAD;
        }
    }
    else
    {
        /* 摇杆线性映射到机械范围 */
        float ratio = ((float)g_input.remote.gimbal_pitch + 1000.0f) / 2000.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < 0.0f) ratio = 0.0f;
        pos_ref = CFG_PITCH_MIN_RAD + ratio * (CFG_PITCH_MAX_RAD - CFG_PITCH_MIN_RAD);
    }

    /* 机械限位 */
    if (pos_ref > CFG_PITCH_MAX_RAD) pos_ref = CFG_PITCH_MAX_RAD;
    if (pos_ref < CFG_PITCH_MIN_RAD) pos_ref = CFG_PITCH_MIN_RAD;

    DMMotorSetRef(pitch_motor, pos_ref);
}

/* ============================ 任务入口 ============================ */

void GimbalTask(void)
{
    state = GimbalDecode();  /* ① 拨杆 → 状态 */

    /* ② 失能态: yaw 停, pitch POSVEL 保持(不动作), 立即返回 */
    if (state == GIMBAL_STATE_DISABLE)
    {
        DJIMotorStop(yaw_motor);
        gun_yaw_ref_locked = 0;  /* 重新使能后重锁当前枪口角, 防猛转回旧 ref */
        return;
    }
    DJIMotorEnable(yaw_motor);   /* 使能(幂等) */

    /* ③ 视觉锁敌子状态: 视觉未接, 恒未锁敌(接视觉后在此按 g_input.vision 刷新) */
    uint8_t vision_locked = 0;
    vision_ever_locked = 0;

    /* ④ Yaw 轴: 目前只有手动 FOLLOW; 视觉未接, VISION 档暂退化到 FOLLOW */
    switch (state)
    {
        case GIMBAL_STATE_VISION:
        default:                 /* GIMBAL_STATE_FOLLOW (DISABLE 已在上面提前返回) */
            GimbalHandleFollow();
            break;
    }

    /* ⑤ Pitch 轴独立控制: 视觉模式走视觉 pitch, 否则摇杆 */
    GimbalHandlePitch(state == GIMBAL_STATE_VISION, vision_locked);
}
