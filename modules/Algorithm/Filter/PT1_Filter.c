//
// Created by Administrator on 2026/1/30.
//

#include "PT1_Filter.h"
#include <math.h>

#define M_PI_F 3.14159265f

void PT1_Filter_Init(PT1_Filter_t *filter, float cutoff_freq)
{
    filter->state = 0.0f;
    filter->cutoff_freq = cutoff_freq;
}

float PT1_Filter_Apply(PT1_Filter_t *filter, float input, float dt)
{
    // 保护：如果 dt 极小（比如刚启动），直接返回原值，防止除零或异常
    if (dt < 0.000001f) return input;

    // 计算 RC 常数: RC = 1 / (2 * pi * fc)
    float rc = 1.0f / (2.0f * M_PI_F * filter->cutoff_freq);

    // 核心：动态计算 Alpha
    // alpha = dt / (dt + RC)
    float alpha = dt / (dt + rc);

    // 赋值给 state 并返回
    filter->state = filter->state + alpha * (input - filter->state);
    return filter->state;
}
