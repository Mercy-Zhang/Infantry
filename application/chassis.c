/**
 * @file chassis.c
 * @brief 底盘控制(移植自哨兵 chassis.c)
 *
 * 4×M3508 速度环+电流环 + 麦轮运动学(含云台偏移)。
 * 模式由拨杆 + 急停决定:
 *   ZERO_FORCE 急停: 电机失能(电流零输入)
 *   NO_FOLLOW  遥控: 全向平移(航向保持暂关)
 *   ROTATE     小陀螺: 固定旋转 + 全向平移
 *
 * 速度单位: 转子 °/s(与哨兵一致)。
 * 坐标系: 任意模式下摇杆输入都以"云台朝向"为前向, 按 offset_angle 旋转到底盘系后再解算,
 *         所以底盘不需要"跟随云台"模式 —— 云台转到哪, 平移方向就跟着转到哪。
 */
#include "chassis.h"
#include "dji_motor.h"
#include "robot_def.h"
#include "general_def.h"
#include "gimbal.h"
#include <math.h>

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;

static PIDInstance chassis_yaw_hold_pid;       /* 航向保持 */

static chassis_state_e state = CHASSIS_STATE_ZERO_FORCE;

/* ============================ 参数(与哨兵一致) ============================ */

/* 平移最大速度: 哨兵 vx/vy = -26*rocker(±660) = ±17160 °/s; 步兵摇杆已映射到 ±1000 */
#define CHASSIS_MAX_TRANSLATE_SPEED 17160.0f
/* 小陀螺固定转速(哨兵 ROTATE 默认 wz) */
#define CHASSIS_ROTATE_WZ 2000.0f
/* 航向保持: 平移低于此值不锁航向, 避免摇杆噪声触发 */
#define CHASSIS_YAW_HOLD_MOVE_THRESHOLD 500.0f
/* 航向修正最大旋转量 */
#define CHASSIS_YAW_HOLD_MAX_WZ 3000.0f

/* 麦轮中心距(含云台偏移, rad), 与哨兵一致 */
#define HALF_WHEEL_BASE  (CFG_WHEEL_BASE / 2.0f)
#define HALF_TRACK_WIDTH (CFG_TRACK_WIDTH / 2.0f)
#define LF_CENTER ((HALF_TRACK_WIDTH + CFG_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CFG_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CFG_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CFG_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CFG_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CFG_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CFG_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CFG_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/* 航向保持状态 */
static uint8_t yaw_hold_locked = 0;
static float   yaw_hold_target = 0.0f;

/* ============================ 工具 ============================ */

/* 航向误差最短路径, 处理 ±180° 环绕 */
static float ChassisWrapYawError(float e)
{
    while (e > 180.0f)  e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

/* 清空航向保持 PID 全部状态 */
static void ResetChassisYawHold(void)
{
    chassis_yaw_hold_pid.Iout = 0.0f;
    chassis_yaw_hold_pid.ITerm = 0.0f;
    chassis_yaw_hold_pid.Last_Err = 0.0f;
    chassis_yaw_hold_pid.Last_Measure = 0.0f;
    chassis_yaw_hold_pid.Last_Output = 0.0f;
    chassis_yaw_hold_pid.Last_Dout = 0.0f;
    chassis_yaw_hold_pid.Output = 0.0f;
    yaw_hold_locked = 0;
}

/* 普通平移时锁定底盘当前航向, 输出 wz 修正(哨兵 UpdateChassisYawHold) */
static void UpdateChassisYawHold(float vx, float vy, float *wz)
{
    float mvx = vx < 0.0f ? -vx : vx;
    float mvy = vy < 0.0f ? -vy : vy;
    if (mvx < CHASSIS_YAW_HOLD_MOVE_THRESHOLD && mvy < CHASSIS_YAW_HOLD_MOVE_THRESHOLD)
    {
        ResetChassisYawHold();
        return;
    }

    float current_yaw = GimbalGetChassisYawDeg();
    //初始锁定，锁定一个初始 yaw_hold_target 角度
    if (!yaw_hold_locked)
    {
        //防止上电跳变
        yaw_hold_target = current_yaw;
        yaw_hold_locked = 1;
        /* 目标角刚锁定, 清微分/积分历史, 避免沿用上一次运动的量 */
        chassis_yaw_hold_pid.Iout = 0.0f;
        chassis_yaw_hold_pid.ITerm = 0.0f;
        chassis_yaw_hold_pid.Last_Err = 0.0f;
        chassis_yaw_hold_pid.Last_Measure = current_yaw;
        chassis_yaw_hold_pid.Last_Output = 0.0f;
        chassis_yaw_hold_pid.Last_Dout = 0.0f;
    }

    float err = ChassisWrapYawError(yaw_hold_target - current_yaw);
    /* 误差折回当前角附近, 既处理 ±180° 环绕, 也保留角度微分阻尼 */
    float target_yaw = current_yaw + err;
    *wz = PIDCalculate(&chassis_yaw_hold_pid, current_yaw, target_yaw);
}


//===============================姿态解算=======================================//

/* 麦轮逆运动学: vx(横移)/vy(前进)/wz(旋转, 转子 °/s) → 四轮速度。
 * 前进=四轮同向; 横移=对角异向(LF/RB 与 RF/LB 反向); 旋转=左右异向。 */
static void MecanumCalculate(float vx, float vy, float wz, float out[4])
{
    out[0] = +vx + vy + wz * LF_CENTER;   /* LF */
    out[1] = +vx - vy + wz * RF_CENTER;   /* RF */
    out[2] = -vx + vy + wz * LB_CENTER;   /* LB */
    out[3] = -vx - vy + wz * RB_CENTER;   /* RB */
}

static void SetWheelRef(float v[4])
{
    DJIMotorSetRef(motor_lf, v[0]);   /* LF */
    DJIMotorSetRef(motor_rf, v[1]);   /* RF */
    DJIMotorSetRef(motor_lb, v[2]);   /* LB */
    DJIMotorSetRef(motor_rb, v[3]);   /* RB */
}

/* ============================ 初始化 ============================ */

void ChassisInit(void)
{
    /* 四个轮子参数一样, 只改 tx_id 和反转标志(哨兵同款) */
    Motor_Init_Config_s cfg = {
        .can_init_config = {.can_handle = &hcan1},
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10.0f, .Ki = 0.0f, .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000.0f, .MaxOut_ = -12000.0f,
            },
            .current_PID = {
                .Kp = 0.5f, .Ki = 0.0f, .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 3000.0f, .MaxOut_ = -3000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,   /* 哨兵全 REVERSE, 上机标定 */
        },
        .motor_type = M3508,
    };

    cfg.can_init_config.tx_id = CFG_CHASSIS_LF_ID;
    motor_lf = DJIMotorInit(&cfg);
    cfg.can_init_config.tx_id = CFG_CHASSIS_RF_ID;
    motor_rf = DJIMotorInit(&cfg);
    cfg.can_init_config.tx_id = CFG_CHASSIS_LB_ID;
    motor_lb = DJIMotorInit(&cfg);
    cfg.can_init_config.tx_id = CFG_CHASSIS_RB_ID;
    motor_rb = DJIMotorInit(&cfg);

    /* 航向保持 PID */
    PID_Init_Config_s hold_cfg = {
        .Kp = 70.0f, .Ki = 0.0f, .Kd = 3.0f,
        .Derivative_LPF_RC = 0.05f,
        .IntegralLimit = 0.0f,
        .Improve = PID_Derivative_On_Measurement | PID_DerivativeFilter,
        .MaxOut = CHASSIS_YAW_HOLD_MAX_WZ, .MaxOut_ = -CHASSIS_YAW_HOLD_MAX_WZ,
        .DeadBand = 0.5f,
    };
    PIDInit(&chassis_yaw_hold_pid, &hold_cfg);
}

/* ============================ 状态迁移 ============================ */

static chassis_state_e ChassisDecode(void)
{
#ifdef SBUS
    /* 中=视觉档: 视觉未接, 暂 stop(与 DBUS 的 switch_right 中位行为一致) */
    if (g_input.remote.sw_estop == 2)      return CHASSIS_STATE_ZERO_FORCE; /* 急停(下) */
    if (g_input.remote.sw_mode == 2)       return CHASSIS_STATE_ROTATE;     /* 上=小陀螺 */
    if (g_input.remote.sw_mode == 1)       return CHASSIS_STATE_ZERO_FORCE; /* 中=视觉档: 暂 stop */
#endif
#ifdef DBUS
    if (g_input.remote.switch_left == 1)   return CHASSIS_STATE_ZERO_FORCE; /* 急停(上) */
    if (g_input.remote.switch_right == 1)  return CHASSIS_STATE_ROTATE;     /* 上=小陀螺 */
#endif
    return CHASSIS_STATE_NO_FOLLOW;                                        /* 下=遥控 */
}

/* ============================ 任务入口 ============================ */

void ChassisTask(void)
{
    state = ChassisDecode();

    /* 电机使能/失能仅在模式切换时做, 避免每周期重复设置 */
    static chassis_state_e prev_state = CHASSIS_STATE_ZERO_FORCE;
    if (state == CHASSIS_STATE_ZERO_FORCE)
    {
        // 刚刚进入失能状态
        if (prev_state != CHASSIS_STATE_ZERO_FORCE)
        {
            DJIMotorStop(motor_lf);
            DJIMotorStop(motor_rf);
            DJIMotorStop(motor_lb);
            DJIMotorStop(motor_rb);
        }
    }
    else if (prev_state == CHASSIS_STATE_ZERO_FORCE)
    {
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
    prev_state = state;

    // 摇杆数据
    float vx = -(g_input.remote.chassis_x / 1000.0f) * CHASSIS_MAX_TRANSLATE_SPEED;  // 前后->前后
    float vy = +(g_input.remote.chassis_y / 1000.0f) * CHASSIS_MAX_TRANSLATE_SPEED;  // 左右->左右
    float wz = 0.0f;

    /* 按模式决定旋转速度 */
    switch (state)
    {
    case CHASSIS_STATE_NO_FOLLOW:
        /* 标定阶段: 航向保持暂关(依赖云台 IMU + yaw 电机, 云台未调好), 纯平移(无自转) */
        break;

    case CHASSIS_STATE_ROTATE:
        ResetChassisYawHold();
        wz = CHASSIS_ROTATE_WZ;              /* 小陀螺固定转速 */
        break;

    default: /* ZERO_FORCE */
        ResetChassisYawHold();
        vx = vy = wz = 0.0f;
        break;
    }

    /* 云台系: 摇杆输入以云台朝向为前向, 按 yaw 电机相对底盘的 offset 旋转到底盘系。
     * 零点/方向用 CFG_GIMBAL_OFFSET_ZERO_DEG / CFG_GIMBAL_OFFSET_DIR 标定。 */
    float offset_deg = (GimbalGetYawOffsetDeg() - CFG_GIMBAL_OFFSET_ZERO_DEG) * CFG_GIMBAL_OFFSET_DIR;  //偏转角theta
    float offset_rad = offset_deg * DEGREE_2_RAD;   //转弧度
    float cos_theta = cosf(offset_rad);
    float sin_theta = sinf(offset_rad);
    //云台映射底盘，角速度大小没关系
    // 一致以云台坐标系移动，无论是否有小陀螺模式
    // 把云台坐标系映射到底盘，得到云台坐标系对应的底盘坐标系数据
    float chassis_vx = vx * cos_theta - vy * sin_theta;
    float chassis_vy = vx * sin_theta + vy * cos_theta;

    /* 麦轮解算 + 设定参考 */
    float v[4];
    MecanumCalculate(chassis_vx, chassis_vy, wz, v);
    SetWheelRef(v);
}
