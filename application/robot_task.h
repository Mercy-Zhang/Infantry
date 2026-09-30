/**
 * @file robot_task.h
 * @brief FreeRTOS 任务初始化与各任务入口
 *
 * 仅被 robot.c 包含一次(函数在头文件内定义)。
 */
#ifndef INF_ROBOT_TASK_H
#define INF_ROBOT_TASK_H

#include "cmsis_os.h"
#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "daemon.h"
#include "dmmotor.h"
#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId motorTaskHandle;
osThreadId robotTaskHandle;
osThreadId daemonTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartDAEMONTASK(void const *argument);

/* 创建所有 FreeRTOS 任务(在 RobotInit 里调用, 需在所有 DMMotorInit 之后) */
void OSTaskInit(void)
{
    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL);

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    /* DM 电机各自起独立任务(需在所有 DMMotorInit 之后调用) */
    DMMotorControlInit();
}

void StartINSTASK(void const *argument)
{
    (void)argument;
    LOGINFO("[ins] task start");
    for (;;)
    {
        INS_Task();      /* 1kHz */
        osDelay(1);
    }
}

void StartMOTORTASK(void const *argument)
{
    (void)argument;
    for (;;)
    {
        MotorControlTask();   /* 1kHz: DJI 电机闭环 */
        osDelay(1);
    }
}

void StartROBOTTASK(void const *argument)
{
    (void)argument;
    for (;;)
    {
        RobotTask();          /* 100Hz: gimbal → shoot → chassis */
        osDelay(10);
    }
}

void StartDAEMONTASK(void const *argument)
{
    (void)argument;
    for (;;)
    {
        DaemonTask();         /* 100Hz */
        osDelay(10);
    }
}

#endif /* INF_ROBOT_TASK_H */
