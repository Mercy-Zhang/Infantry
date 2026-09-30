/**
 * @file ins_task.c
 * @brief 姿态解算实现(四元数 EKF) + IMU 恒温控制
 *
 * 读取 BMI088 陀螺/加速度, 用 QuaternionEKF 融合, 输出欧拉角与 yaw 总角度。
 * 恒温控制稳住陀螺零偏(yaw 轴无重力参考, 零偏温漂直接导致 yaw 漂移)。
 * 单位: Gyro=rad/s, Accel=m/s^2, 角度=度。
 */
#include "ins_task.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"
#include "controller.h"
#include "bsp_dwt.h"
#include "spi.h"          /* hspi2: BMI088 SPI */
#include "tim.h"          /* htim3: 加热片 PWM (CubeMX 配置) */
#include <math.h>
#include <string.h>

/* IMU 恒温控制: 1=启用(需加热片+对应 PWM), 0=禁用 */
#define CFG_IMU_HEAT_ENABLE   1
#define CFG_IMU_HEAT_REF_DEG  40.0f

static attitude_t ins;
volatile uint8_t INS_init_done = 0;

static uint32_t ins_dwt = 0;

#if CFG_IMU_HEAT_ENABLE
static PIDInstance heat_pid;

static void IMU_TempCtrl(void)
{
    float pwm = PIDCalculate(&heat_pid, BMI088.Temperature, CFG_IMU_HEAT_REF_DEG);
    if (pwm < 0.0f) pwm = 0.0f;
    if (pwm > 800.0f) pwm = 800.0f;
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, (uint16_t)pwm);  /* 加热片 PWM 通道 */
}
#endif

/* 用加速度计初始化 roll/pitch(yaw=0), 避免开机姿态误差 */
static void InitQuaternion(float *q)
{
    float acc[3] = {0.0f};

    for (uint8_t i = 0; i < 100; i++)
    {
        BMI088_Read(&BMI088);
        acc[0] += BMI088.Accel[0];
        acc[1] += BMI088.Accel[1];
        acc[2] += BMI088.Accel[2];
        DWT_Delay(0.001f);
    }
    acc[0] /= 100.0f;
    acc[1] /= 100.0f;
    acc[2] /= 100.0f;

    float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (n > 1e-6f)
    {
        acc[0] /= n;
        acc[1] /= n;
        acc[2] /= n;
    }

    /* 加速度矢量与重力[0,0,1]的旋转(轴角) */
    float angle = acosf(acc[2]);
    float axis[3] = {acc[1], -acc[0], 0.0f};   /* cross(acc, [0,0,1]) */
    float an = sqrtf(axis[0] * axis[0] + axis[1] * axis[1]);
    if (an > 1e-6f)
    {
        axis[0] /= an;
        axis[1] /= an;
    }

    float half = angle / 2.0f;
    q[0] = cosf(half);
    q[1] = axis[0] * sinf(half);
    q[2] = axis[1] * sinf(half);
    q[3] = 0.0f;   /* yaw 轴无分量 */
}

attitude_t *INS_Init(void)
{
    /* 幂等保护: RobotInit 已在关中断、无并发下完成初始化;
     * 后续 GimbalTask 的懒加载直接复用, 避免二次 BMI088Init(软复位+最长12s标定)
     * 与 1kHz INS_Task 抢 SPI2, 把加速度计读到 0 / 姿态被二次复位。 */
    if (INS_init_done)
        return &ins;

    /* 1=在线标定(开机测陀螺零偏); 换 &hspi2 为你的 BMI088 所在 SPI */
    BMI088Init(&hspi2, 1);

#if CFG_IMU_HEAT_ENABLE
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    PID_Init_Config_s tc = {
        .Kp = 400.0f, .Ki = 5.0f, .Kd = 0.0f,
        .MaxOut = 800.0f, .MaxOut_ = 0.0f,
        .IntegralLimit = 80.0f,
        .Improve = PID_Integral_Limit,
    };
    PIDInit(&heat_pid, &tc);
#endif

    float q[4] = {0.0f};
    InitQuaternion(q);
    /* 过程噪声1=10, 过程噪声2=0.001, 量测噪声=1e6, 渐消=1, 低通=0 */
    IMU_QuaternionEKF_Init(q, 10.0f, 0.001f, 1000000.0f, 1.0f, 0.0f);

    ins_dwt = 0;
    memset(&ins, 0, sizeof(ins));

    INS_init_done = 1;
    return &ins;
}

void INS_Task(void)
{
    static uint32_t cnt = 0;
    float dt = DWT_GetDeltaT(&ins_dwt);
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.01f) dt = 0.01f;

    BMI088_Read(&BMI088);

    IMU_QuaternionEKF_Update(BMI088.Gyro[0], BMI088.Gyro[1], BMI088.Gyro[2],
                             BMI088.Accel[0], BMI088.Accel[1], BMI088.Accel[2],
                             dt);

    ins.Gyro[0] = BMI088.Gyro[0];
    ins.Gyro[1] = BMI088.Gyro[1];
    ins.Gyro[2] = BMI088.Gyro[2];
    ins.Accel[0] = BMI088.Accel[0];
    ins.Accel[1] = BMI088.Accel[1];
    ins.Accel[2] = BMI088.Accel[2];

    ins.Roll = QEKF_INS.Roll;
    ins.Pitch = QEKF_INS.Pitch;
    ins.Yaw = QEKF_INS.Yaw;
    ins.YawTotalAngle = QEKF_INS.YawTotalAngle;

#if CFG_IMU_HEAT_ENABLE
    if ((cnt % 2) == 0)   /* 500Hz */
        IMU_TempCtrl();
#endif
    cnt++;
}

void INS_GetAttitude(float *yaw, float *pitch, float *roll)
{
    if (yaw)   *yaw   = ins.YawTotalAngle;
    if (pitch) *pitch = ins.Pitch;
    if (roll)  *roll  = ins.Roll;
}

void INS_GetGyro(float *gx, float *gy, float *gz)
{
    if (gx) *gx = ins.Gyro[0];
    if (gy) *gy = ins.Gyro[1];
    if (gz) *gz = ins.Gyro[2];
}
