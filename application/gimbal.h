/**
 * @file gimbal.h
 * @brief 云台控制(单 yaw + 单 pitch)
 */
#ifndef INF_GIMBAL_H
#define INF_GIMBAL_H

#include <stdint.h>

void GimbalInit(void);
void GimbalTask(void);

/* 底盘 yaw(°) = 云台 IMU yaw - yaw电机相对底盘角度, 供底盘航向保持 */
float GimbalGetChassisYawDeg(void);

/* 云台 yaw 电机相对底盘的角度(°), 即 offset_angle, 供底盘坐标旋转(云台系→底盘系) */
float GimbalGetYawOffsetDeg(void);

/* 视觉开火到位判定: yaw/pitch 误差都 ≤ tolerance_deg 返回 1(视觉未接, 实现待接入) */
uint8_t GimbalVisionTargetAligned(float tolerance_deg);

#endif /* INF_GIMBAL_H */
