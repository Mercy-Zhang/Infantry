/**
 * @file robot_def.h
 * @brief 步兵整车参数与共享数据结构
 *
 * 机械/接线参数集中在这里; 状态枚举与遥控/视觉结构体是各 FSM 任务的公共契约。
 */
#ifndef INF_ROBOT_DEF_H
#define INF_ROBOT_DEF_H

#include <stdint.h>

/* ============================== 底盘 ============================== */

/* 机械参数, 单位 mm */
#define CFG_WHEEL_BASE       350.0f   /* 纵向轴距(前后轮中心距) */
#define CFG_TRACK_WIDTH      300.0f   /* 横向轮距(左右轮中心距) */
#define CFG_WHEEL_RADIUS      60.0f   /* 轮子半径 */
#define CFG_WHEEL_REDUCTION   19.0f   /* 麦轮电机减速比(3508 内转子:输出轴) */
#define CFG_GIMBAL_OFFSET_X    0.0f   /* 云台旋转中心相对底盘几何中心 前后偏移 */
#define CFG_GIMBAL_OFFSET_Y    0.0f   /* 云台旋转中心相对底盘几何中心 左右偏移 */

/* 底盘 3508 电调 CAN ID (DJI 1~4): 实测 LF=1 LB=2 RB=3 RF=4 */
#define CFG_CHASSIS_LF_ID  1
#define CFG_CHASSIS_RF_ID  4
#define CFG_CHASSIS_LB_ID  2
#define CFG_CHASSIS_RB_ID  3

/* ============================== 云台 ============================== */

/* 云台电机 CAN 地址 */
#define CFG_YAW_GM6020_ID  1   /* GM6020, FDCAN1(与底盘同总线), tx=0x1FF rx=0x205 (拨码 1) */
#define CFG_PITCH_DM_ID    2   /* DM4310, FDCAN2, POSVEL 模式, tx=0x102 rx=0x12 */

/* pitch 水平位置: 云台水平时电机绝对位置(rad), 机械改动需重新标定 */
#define CFG_PITCH_HORIZON_RAD   0.435f   /* 占位=中点, 上车标定: 云台水平时 pitch_motor->measure.position */
/* pitch 机械行程(rad) */
#define CFG_PITCH_MAX_RAD       0.75f
#define CFG_PITCH_MIN_RAD       0.14f

/* 陀螺仪 → 云台轴方向 (1 同向, -1 反向) */
#define CFG_GYRO_DIR_YAW     1
#define CFG_GYRO_DIR_PITCH  -1

/* 云台系(坐标旋转): yaw 电机相对底盘的 offset 用于把摇杆从云台系转到底盘系
 * CFG_GIMBAL_OFFSET_ZERO_DEG: 云台朝底盘正前方时 yaw_motor->measure.total_angle(°), 上机标定
 * CFG_GIMBAL_OFFSET_DIR: 1 同向 / -1 反向, 上机标定 */
#define CFG_GIMBAL_OFFSET_ZERO_DEG  (-28.6546936f)
#define CFG_GIMBAL_OFFSET_DIR       1.0f

/* 视觉自瞄总开关: 0=失能(未接视觉, 视觉档位退化为 FOLLOW 自稳), 1=启用 */
#define CFG_VISION_ENABLE           0

/* 视觉 yaw 零偏: 视觉绝对角零点与 IMU 零点的固定偏置(deg), 上机标定 */
#define CFG_VISION_YAW_OFFSET_DEG   0.0f
/* 视觉 pitch → 电机位置映射 (deg→rad 的零点与方向) */
#define CFG_VISION_PITCH_HORIZON_RAD 0.435f   /* 同上, 水平位置 */
#define CFG_VISION_PITCH_DIR         1.0f   /* 1=视觉向上=位置增大, -1=反向 */

/* ============================== 发射 ============================== */

/* 摩擦轮 3508 CAN ID (FDCAN2, DJI 1~2) */
#define CFG_FRICTION_L_ID  1
#define CFG_FRICTION_R_ID  2
/* 拨盘 M2006 CAN ID (FDCAN1) */
#define CFG_LOADER_M2006_ID  6   /* M2006, DJI 协议: rx=0x206 (拨码 6), 走 0x1FF 命令组 */

/* 摩擦轮目标转速(3508 速度环单位, 实测标定) */
#define CFG_FRICTION_SPEED   80000.0f
/* 拨盘发弹/单发/卡弹/起转时间等可调参数在 shoot.c(volatile, Ozone 可调), 与哨兵一致 */

/* ============================== 状态枚举 ============================== */

typedef enum {
    GIMBAL_STATE_DISABLE = 0,  /* 急停, 两轴停 */
    GIMBAL_STATE_FOLLOW,       /* 手动跟随: 摇杆 → yaw 角速度 + pitch 位置 */
    GIMBAL_STATE_VISION,       /* 视觉自瞄: 绝对 yaw + 视觉 pitch */
} gimbal_state_e;

typedef enum {
    CHASSIS_STATE_ZERO_FORCE = 0,  /* 零力/急停 */
    CHASSIS_STATE_NO_FOLLOW,       /* 全向平移(云台坐标系, 带航向保持) */
    CHASSIS_STATE_ROTATE,          /* 小陀螺 */
} chassis_state_e;

/* ============================== 遥控器(天地飞 SBUS) ============================== */
// #define SBUS
#define DBUS
typedef struct {
    int16_t gimbal_yaw;   /* 云台 yaw 摇杆(左手水平), -1000~1000 */
    int16_t gimbal_pitch; /* 云台 pitch 摇杆(左手竖直), -1000~1000 */
    int16_t chassis_x;    /* 底盘前后摇杆(右手竖直), -1000~1000 */
    int16_t chassis_y;    /* 底盘横移摇杆(右手水平), -1000~1000 */
    uint8_t switch_right; /* DBUS/DT7: 右开关 S2 原始值(1上/3中/2下), 底盘模式 */
    uint8_t switch_left;  /* DBUS/DT7: 左开关 S1 原始值(1上/3中/2下), 云台模式+急停 */
    uint8_t sw_estop;    /* SBUS/天地飞专用: 急停(两段): 0下=急停, 2上=使能 */
    uint8_t sw_mode;     /* SBUS/天地飞专用: 模式(三段): 0下=遥控器, 1中=视觉, 2上=小陀螺 */
    uint8_t sw_friction; /* SBUS/天地飞专用: 摩擦轮开关(SA, 三段): 2上=开摩擦轮, 其余=待机 */
    uint8_t sw_loader;   /* SBUS/天地飞专用: 拨盘开关(SB, 三段): 0下=待机, 1中=单发, 2上=连发 */
    int16_t wheel;       /* DBUS/DT7: 拨轮; SBUS: 旋钮(未用) */
    uint8_t online;      /* 1=在线 */
} Remote_t;

/* ============================== 视觉(Seasky) ============================== */

typedef enum {
    VISION_NO_TARGET = 0,
    VISION_CONVERGING = 1,
    VISION_READY = 2,
} vision_target_state_e;

typedef struct {
    float yaw;               /* 绝对目标 yaw(deg) */
    float pitch;             /* 目标 pitch(deg) */
    uint8_t target_state;    /* vision_target_state_e */
    uint8_t shoot;           /* 开火脉冲 */
    uint8_t online;          /* 1=在线 */
} Vision_t;

/* ============================== 共享输入 ============================== */

/* 各 FSM 任务的输入契约: 由 remote/vision 模块周期性刷新 */
typedef struct {
    Remote_t remote;
    Vision_t vision;
} Robot_Input_t;

extern Robot_Input_t g_input;

#endif /* INF_ROBOT_DEF_H */
