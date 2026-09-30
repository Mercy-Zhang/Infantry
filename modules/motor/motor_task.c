/**
 * @file motor_task.c
 * @brief 电机控制闭环任务入口实现
 *
 * DM 电机各自在 DMMotorControlInit() 里起了独立任务,
 * 这里只负责 DJI 电机(M3508)的闭环发送。
 */
#include "motor_task.h"
#include "dji_motor.h"

void MotorControlTask(void)
{
    DJIMotorControl();
}
