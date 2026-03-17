//
// Created by Administrator on 2026/1/10.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_0_BSP_DWT_H
#define UAV_BAICE_FRAMEWORK_V1_0_BSP_DWT_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint32_t s;
    uint32_t ms;
    uint32_t us;
} DWT_Time_t;

/**
 * @brief  初始化DWT计时器
 * @param  CPU_Freq_MHz  CPU频率，单位MHz（如168）
 */
void DWT_Init(uint32_t CPU_Freq_MHz);

/**
 * @brief  获取两次调用之间的时间差（float精度）
 * @param  cnt_last  上次的CYCCNT值指针，函数内部会自动更新
 * @return 时间差，单位秒
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief  获取两次调用之间的时间差（double精度）
 * @param  cnt_last  上次的CYCCNT值指针，函数内部会自动更新
 * @return 时间差，单位秒
 */
double DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief  更新系统时间
 */
void DWT_SysTimeUpdate(void);

/**
 * @brief  获取系统运行时间线（秒）
 */
float DWT_GetTimeline_s(void);

/**
 * @brief  获取系统运行时间线（毫秒）
 */
double DWT_GetTimeline_ms(void);

/**
 * @brief  获取系统运行时间线（微秒）
 */
uint64_t DWT_GetTimeline_us(void);

/**
 * @brief  DWT延时（秒）
 * @param  Delay  延时时间，单位秒
 */
void DWT_Delay(float Delay);

/**
 * @brief  获取CPU每微秒的周期数（供外部直接换算用）
 */
uint32_t DWT_GetCPUFreq_us(void);

#endif //UAV_BAICE_FRAMEWORK_V1_0_BSP_DWT_H