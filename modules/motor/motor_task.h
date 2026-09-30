/**
 * @file motor_task.h
 * @brief 电机控制闭环任务入口
 */
#ifndef INF_MOTOR_TASK_H
#define INF_MOTOR_TASK_H

/* 在 RTOS 中以 1kHz 调用: 计算所有 DJI 电机闭环并发送 */
void MotorControlTask(void);

#endif /* INF_MOTOR_TASK_H */
