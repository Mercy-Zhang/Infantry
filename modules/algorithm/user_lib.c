/**
 * @file user_lib.c
 * @brief 常用数学工具函数实现
 */
#include "user_lib.h"
#include <string.h>

void *zmalloc(size_t size)
{
    void *p = malloc(size);
    if (p != NULL)
        memset(p, 0, size);
    return p;
}

float abs_limit(float num, float limit)
{
    if (num > limit)  return limit;
    if (num < -limit) return -limit;
    return num;
}

float sign(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

float float_deadband(float value, float min, float max)
{
    if (value > min && value < max)
        return 0.0f;
    return value;
}

float float_constrain(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

int16_t int16_constrain(int16_t value, int16_t min, int16_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float loop_float_constrain(float input, float min, float max)
{
    float range = max - min;
    while (input > max) input -= range;
    while (input < min) input += range;
    return input;
}

float theta_format(float ang)
{
    return loop_float_constrain(ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    return (int)(raw + (raw >= 0.0f ? 0.5f : -0.5f));
}

/* 滑动平均: 新数据入队, 返回当前平均值 */
float AverageFilter(float new_data, float *buf, uint8_t len)
{
    float sum = new_data;
    for (uint8_t i = 1; i < len; i++)
    {
        buf[i - 1] = buf[i];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    return sum / (float)len;
}
