/**
 * @file bsp_dwt.h
 * @brief DWT 周期计数器: 微秒延时与时间轴
 */
#ifndef INF_BSP_DWT_H
#define INF_BSP_DWT_H

#include <stdint.h>

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/* 初始化, 传入 CPU 频率(单位 MHz) */
void DWT_Init(uint32_t CPU_Freq_mHz);

/* 两次调用之间的时间间隔(秒), cnt_last 为时间戳 */
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);

/* 时间轴(自初始化起) */
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);

/* 秒级延时, 可在临界区使用 */
void DWT_Delay(float Delay);

/* 更新时间轴(长时间不调 timeline 时手动调用, 防 CYCCNT 溢出) */
void DWT_SysTimeUpdate(void);

#endif /* INF_BSP_DWT_H */
