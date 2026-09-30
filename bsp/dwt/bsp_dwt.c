/**
 * @file bsp_dwt.c
 * @brief DWT 周期计数器实现
 */
#include "bsp_dwt.h"
#include "main.h"

static uint32_t us_factor;        /* 每微秒的时钟周期数(= CPU MHz) */
static uint64_t sys_time_us;      /* 累计微秒时间轴 */
static uint32_t last_cyccnt;

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    (void)CPU_Freq_mHz;
    us_factor = SystemCoreClock / 1000000UL;   /* cycles per us */

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    sys_time_us = 0;
    last_cyccnt = 0;
}

void DWT_SysTimeUpdate(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t delta = now - last_cyccnt;
    last_cyccnt = now;
    sys_time_us += delta / us_factor;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    return sys_time_us;
}

float DWT_GetTimeline_ms(void)
{
    return (float)(DWT_GetTimeline_us() / 1000ULL);
}

float DWT_GetTimeline_s(void)
{
    return (float)(DWT_GetTimeline_us() / 1000000ULL);
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t now = DWT->CYCCNT;
    float dt = (float)(now - *cnt_last) / (float)(us_factor * 1000000UL);
    *cnt_last = now;
    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t now = DWT->CYCCNT;
    double dt = (double)(now - *cnt_last) / (double)(us_factor * 1000000UL);
    *cnt_last = now;
    return dt;
}

void DWT_Delay(float Delay)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (uint32_t)(Delay * (float)(us_factor * 1000000UL));
    while ((DWT->CYCCNT - start) < cycles)
    {
    }
}
