/**
 * @file ins_task.h
 * @brief 姿态解算(IMU): 互补滤波
 *
 * 单位约定: Gyro=rad/s, Accel=m/s^2, 角度(Roll/Pitch/Yaw/YawTotalAngle)=度。
 * 方向修正(CFG_GYRO_DIR_*)由使用者(gimbal/chassis)在读取时施加。
 */
#ifndef INF_INS_TASK_H
#define INF_INS_TASK_H

#include <stdint.h>

typedef struct
{
    float Gyro[3];        /* 角速度 rad/s */
    float Accel[3];       /* 加速度 m/s^2 */
    float Roll;           /* 横滚角 度 */
    float Pitch;          /* 俯仰角 度 */
    float Yaw;            /* 偏航角 度 [-180,180] */
    float YawTotalAngle;  /* 偏航总角度 度 (多圈累加) */
} attitude_t;

extern volatile uint8_t INS_init_done;

attitude_t *INS_Init(void);

/* 1kHz 调用: 读 BMI088 + 更新姿态 */
void INS_Task(void);

void INS_GetAttitude(float *yaw, float *pitch, float *roll);
void INS_GetGyro(float *gx, float *gy, float *gz);

#endif /* INF_INS_TASK_H */
