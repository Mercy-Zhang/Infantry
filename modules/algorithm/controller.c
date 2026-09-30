/**
 * @file controller.c
 * @brief 位置式 PID 控制器实现
 */
#include "controller.h"
#include "bsp_dwt.h"
#include <string.h>

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    memset(pid, 0, sizeof(PIDInstance));
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut;
    pid->MaxOut_ = config->MaxOut_;
    pid->DeadBand = config->DeadBand;
    pid->Improve = config->Improve;
    pid->IntegralLimit = config->IntegralLimit;
    pid->CoefA = config->CoefA;
    pid->CoefB = config->CoefB;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
}

/* 一阶低通: y = a*x + (1-a)*y_prev, a = dt/(rc+dt) */
static float lpf(float *state, float x, float rc, float dt)
{
    if (rc <= 0.0f)
        return x;
    float a = dt / (rc + dt);
    *state = a * x + (1.0f - a) * (*state);
    return *state;
}

float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    float dt = DWT_GetDeltaT(&pid->DWT_CNT);
    if (dt <= 0.0f)
        dt = 0.001f;
    if (dt > 0.05f)
        dt = 0.05f; /* 首次调用或任务卡顿保护, 防微分爆炸 */

    pid->Measure = measure;
    pid->Ref = ref;

    float err = ref - measure;
    if (err > -pid->DeadBand && err < pid->DeadBand)
        err = 0.0f;
    pid->Err = err;

    /* ---- 比例项 ---- */
    float pout;
    if (pid->Improve & PID_Proportional_On_Measurement)
        pout = -pid->Kp * measure;
    else
        pout = pid->Kp * err;

    /* ---- 积分项 ---- */
    float i_gain = pid->Ki;
    if (pid->Improve & PID_ChangingIntegrationRate)
    {
        float e = err < 0.0f ? -err : err;
        if (e > pid->CoefA + pid->CoefB)
            i_gain = 0.0f;
        else if (e > pid->CoefA && pid->CoefB > 1e-6f)
            i_gain *= (pid->CoefA + pid->CoefB - e) / pid->CoefB;
    }

    float err_int = (pid->Improve & PID_Trapezoid_Intergral)
                        ? 0.5f * (err + pid->Last_Err)
                        : err;
    pid->ITerm += i_gain * err_int * dt;

    if (pid->Improve & PID_Integral_Limit)
    {
        if (pid->ITerm > pid->IntegralLimit)  pid->ITerm = pid->IntegralLimit;
        if (pid->ITerm < -pid->IntegralLimit) pid->ITerm = -pid->IntegralLimit;
    }

    /* ---- 微分项 ---- */
    float dout;
    if (pid->Improve & PID_Derivative_On_Measurement)
        dout = -pid->Kd * (measure - pid->Last_Measure) / dt;
    else
        dout = pid->Kd * (err - pid->Last_Err) / dt;

    if (pid->Improve & PID_DerivativeFilter)
        dout = lpf(&pid->Last_Dout, dout, pid->Derivative_LPF_RC, dt);

    /* ---- 求和 + 限幅 ---- */
    float out = pout + pid->ITerm + dout;
    if (out > pid->MaxOut)  out = pid->MaxOut;
    if (out < pid->MaxOut_) out = pid->MaxOut_;

    /* ---- 输出滤波 ---- */
    if (pid->Improve & PID_OutputFilter)
        out = lpf(&pid->Last_Output, out, pid->Output_LPF_RC, dt);

    pid->Pout = pout;
    pid->Iout = pid->ITerm;
    pid->Dout = dout;
    pid->Output = out;
    pid->Last_Measure = measure;
    pid->Last_Err = err;
    pid->Last_ITerm = pid->ITerm;
    pid->dt = dt;

    return out;
}
