/**
 * @file controller.h
 * @brief 位置式 PID 控制器(独立实现, 接口与既有工程兼容)
 */
#ifndef INF_CONTROLLER_H
#define INF_CONTROLLER_H

#include <stdint.h>

/* 改进项标志位, 按位或组合 */
typedef enum {
    PID_IMPROVE_NONE                 = 0x00,
    PID_Integral_Limit               = 0x01, /* 积分限幅 */
    PID_Derivative_On_Measurement    = 0x02, /* 微分先行(对测量值微分) */
    PID_Trapezoid_Intergral          = 0x04, /* 梯形积分 */
    PID_Proportional_On_Measurement  = 0x08, /* 比例项用测量值 */
    PID_OutputFilter                 = 0x10, /* 输出一阶低通 */
    PID_ChangingIntegrationRate      = 0x20, /* 变速积分 */
    PID_DerivativeFilter             = 0x40, /* 微分一阶低通 */
    PID_ErrorHandle                  = 0x80, /* 堵转检测(占位) */
} PID_Improvement_e;

typedef enum {
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U,
} ErrorType_e;

typedef struct {
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef struct {
    /* ---- 配置 ---- */
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float MaxOut_;
    float DeadBand;
    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA;              /* 变速积分参数 */
    float CoefB;
    float Output_LPF_RC;      /* 输出滤波 RC(秒), 0=关闭 */
    float Derivative_LPF_RC;  /* 微分滤波 RC(秒), 0=关闭 */

    /* ---- 运行状态 ---- */
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;
    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float Output;
    float Last_Output;
    float Last_Dout;
    float Ref;
    uint32_t DWT_CNT;
    float dt;
    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float MaxOut_;
    float DeadBand;
    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA;
    float CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;
} PID_Init_Config_s;

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/* 计算一次 PID, 返回输出; dt 由 DWT 自动测量 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif /* INF_CONTROLLER_H */
