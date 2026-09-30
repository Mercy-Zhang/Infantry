/**
 * @file robot.c
 * @brief 机器人初始化与主控制循环实现
 */
#include "robot.h"
#include "robot_def.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "ins_task.h"
#include "vision.h"
#include "remote.h"
#include "robot_task.h"

/* 全局共享输入: 由 remote/vision 模块周期性刷新 */
Robot_Input_t g_input;

void RobotInit(void)
{
    __disable_irq();

    DWT_Init(SystemCoreClock / 1000000UL);   /* CPU MHz */
    BspCanInit();

    GimbalInit();
    ChassisInit();
    ShootInit();
    INS_Init();
    RemoteInit();
    VisionInit();

    /* 创建 FreeRTOS 任务(需在所有 DMMotorInit 之后) */
    OSTaskInit();

    __enable_irq();
}

void RobotTask(void)
{
    /* 先刷新输入(视觉/遥控), 再跑各子系统 */
    VisionTask();
    RemoteTask();

    GimbalTask();
    ShootTask();
    ChassisTask();
}
