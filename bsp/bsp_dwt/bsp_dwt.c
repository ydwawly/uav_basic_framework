//
// Created by Administrator on 2026/1/10.
//
#include "bsp_dwt.h"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz;       // CPU频率 (Hz)
static uint32_t CPU_FREQ_Hz_ms;    // 每毫秒周期数
static uint32_t CPU_FREQ_Hz_us;    // 每微秒周期数
static uint32_t CYCCNT_RountCount; // CYCCNT 溢出计数
static uint32_t CYCCNT_LAST;       // 上次 CYCCNT 值
static uint64_t CYCCNT64;          // 64 位扩展周期计数

/**
 * @brief  私有函数，检查DWT CYCCNT寄存器是否溢出并更新溢出计数
 * @note   使用关中断保护，确保原子性
 */
static void DWT_CNT_Update(void)
{
    /* 关中断保证原子操作，避免中断重入导致溢出漏检 */
    __disable_irq();

    volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;

    CYCCNT_LAST = cnt_now;  // ✅ 修复：使用cnt_now而非重新读取寄存器

    __enable_irq();
}

/**
 * @brief  初始化DWT计时器
 * @param  CPU_Freq_MHz  CPU频率，单位MHz（如168）
 */
void DWT_Init(uint32_t CPU_Freq_MHz)  // ✅ 修复：命名从mHz改为MHz
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz    = CPU_Freq_MHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
    CYCCNT_LAST = 0;

    DWT_CNT_Update();
}

/**
 * @brief  获取两次调用之间的时间差（float精度）
 * @param  cnt_last  上次的CYCCNT值指针
 * @return 时间差，单位秒
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    /* 利用uint32_t无符号减法自动处理溢出 */
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief  获取两次调用之间的时间差（double精度）
 * @param  cnt_last  上次的CYCCNT值指针
 * @return 时间差，单位秒
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief  更新系统时间
 * @note   使用关中断保证cnt_now与CYCCNT_RountCount的一致性
 */
void DWT_SysTimeUpdate(void)
{
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    /* ✅ 修复：原子地读取cnt_now并更新溢出计数，保证两者一致 */
    __disable_irq();

    volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;
    CYCCNT_LAST = cnt_now;

    uint32_t round_count = CYCCNT_RountCount;

    __enable_irq();

    /* ✅ 修复：使用 (1ULL << 32) 代替 UINT32_MAX，避免差一错误 */
    CYCCNT64 = (uint64_t)round_count * ((uint64_t)1 << 32) + (uint64_t)cnt_now;

    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = (uint32_t)CNT_TEMP1;
    SysTime.ms = (uint32_t)(CNT_TEMP2 / CPU_FREQ_Hz_ms);
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = (uint32_t)(CNT_TEMP3 / CPU_FREQ_Hz_us);
}

/**
 * @brief  获取系统运行时间线（秒）
 * @return 时间，单位秒（float）
 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

/**
 * @brief  获取系统运行时间线（毫秒）
 * @return 时间，单位毫秒（double，避免float精度不足）
 */
double DWT_GetTimeline_ms(void)  // ✅ 修复：返回值改为double，避免float精度不足
{
    DWT_SysTimeUpdate();

    double DWT_Timelinef64 = (double)SysTime.s * 1000.0
                           + (double)SysTime.ms
                           + (double)SysTime.us * 0.001;

    return DWT_Timelinef64;
}

/**
 * @brief  获取系统运行时间线（微秒）
 * @return 时间，单位微秒（uint64_t）
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timeline_us = (uint64_t)SysTime.s * 1000000
                             + (uint64_t)SysTime.ms * 1000
                             + (uint64_t)SysTime.us;

    return DWT_Timeline_us;
}

/**
 * @brief  DWT阻塞延时
 * @param  Delay  延时时间，单位秒
 * @note   最大延时不应超过CYCCNT溢出周期（168MHz下约25.5秒）
 */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;

    /* ✅ 修复：提前计算目标周期数，转为uint32_t避免每次循环做浮点运算 */
    uint32_t wait_cycles = (uint32_t)(Delay * (float)CPU_FREQ_Hz);

    while ((DWT->CYCCNT - tickstart) < wait_cycles)
        ;
}
uint32_t DWT_GetCPUFreq_us(void)
{
    return CPU_FREQ_Hz_us;
}