/**
 * @file motor_def.h
 * @brief 电机通用数据结构与闭环定义
 */
#ifndef INF_MOTOR_DEF_H
#define INF_MOTOR_DEF_H

#include "controller.h"
#include "bsp_can.h"
#include <stdint.h>

#define LIMIT_MIN_MAX(x, min, max) \
    (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/* 闭环类型, 需要多环用按位或 */
typedef enum {
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP = 0b0010,
    ANGLE_LOOP = 0b0100,

    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THREE_LOOP = 0b0111,
} Closeloop_Type_e;

typedef enum {
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01,
    SPEED_FEEDFORWARD = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* 反馈来源 */
typedef enum {
    MOTOR_FEED = 0,
    OTHER_FEED,
} Feedback_Source_e;

/* 电机正反转 */
typedef enum {
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1,
} Motor_Reverse_Flag_e;

/* 反馈量正反 */
typedef enum {
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1,
} Feedback_Reverse_Flag_e;

/* 电机工作状态 */
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_ENALBED = 1,
    MOTOR_DEBUG = 2,
    MOTOR_POWER = 3,
} Motor_Working_Type_e;

/* 电机控制设置 */
typedef struct {
    Closeloop_Type_e outer_loop_type;
    Closeloop_Type_e close_loop_type;
    Motor_Reverse_Flag_e motor_reverse_flag;
    Feedback_Reverse_Flag_e feedback_reverse_flag;
    Feedback_Source_e angle_feedback_source;
    Feedback_Source_e speed_feedback_source;
    Feedfoward_Type_e feedforward_flag;
} Motor_Control_Setting_s;

/* 电机控制器(串级 PID + 反馈/前馈指针) */
typedef struct {
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    float pid_ref;
} Motor_Controller_s;

/* 电机类型 */
typedef enum {
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
    GOM8010,
} Motor_Type_e;

/* 电机控制器初始化结构体 */
typedef struct {
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
} Motor_Controller_Init_s;

/* 电机初始化结构体(各类电机通用) */
typedef struct {
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;
    uint8_t debug_flag;
} Motor_Init_Config_s;

#endif /* INF_MOTOR_DEF_H */
