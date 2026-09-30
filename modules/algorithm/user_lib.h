/**
 * @file user_lib.h
 * @brief 常用数学工具函数(独立实现)
 */
#ifndef INF_USER_LIB_H
#define INF_USER_LIB_H

#include <stdint.h>
#include <stdlib.h>
#include "arm_math.h"

#ifndef PI
#define PI 3.1415926535f
#endif

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

#define VAL_LIMIT(val, min, max)                       \
    do {                                               \
        if ((val) <= (min))      (val) = (min);        \
        else if ((val) >= (max)) (val) = (max);        \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/* 分配并清零 */
void *zmalloc(size_t size);

/* 数值限幅 */
float abs_limit(float num, float limit);
float sign(float value);
float float_deadband(float value, float min, float max);
float float_constrain(float value, float min, float max);
int16_t int16_constrain(int16_t value, int16_t min, int16_t max);
float loop_float_constrain(float input, float min, float max);

/* 角度折到 [-180, 180] */
float theta_format(float ang);

int float_rounding(float raw);
float AverageFilter(float new_data, float *buf, uint8_t len);

#define rad_format(ang) loop_float_constrain((ang), -PI, PI)

#endif /* INF_USER_LIB_H */
