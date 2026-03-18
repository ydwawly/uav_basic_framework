//
// Created by Administrator on 2026/3/2.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_5_SBUS_H
#define UAV_BAICE_FRAMEWORK_V1_5_SBUS_H

// 标准 SBUS 协议参数
#define SBUS_FRAME_SIZE    25     // 标准 SBUS 帧长
#define SBUS_HEADER        0x0F   // 标准 SBUS 帧头
#define SBUS_FOOTER        0x00   // 标准 SBUS 帧尾

// 通道定义
#define SBUS_CH_TOTAL      16     // 16个模拟通道
#define SBUS_CH_MIN        172    // 遥控器输出的最小值 (根据实际校准)
#define SBUS_CH_MAX        1811   // 遥控器输出的最大值
#define SBUS_CH_MID        992    // 中位值

#include <stdbool.h>
#include <stdint.h>

// 解析后的遥控器数据结构体
typedef struct
{
    // 原始通道数据 (11-bit, range: 0 ~ 2047)
    uint16_t ch_raw[SBUS_CH_TOTAL];

    // 归一化后的数据 (range: -1.0 ~ 1.0), 方便 PID 计算
    float ch_normalized[SBUS_CH_TOTAL];

    // 功能开关与状态
    uint8_t ch17;         // 数字通道 17
    uint8_t ch18;         // 数字通道 18
    uint8_t frame_lost;   // 丢帧标志
    uint8_t failsafe;     // 失控保护标志

} Sbus_RC_Data_t;

void Sbus_Init(void);

void Sbus_Task_Handler(void);

// 🌟 零拷贝快速接口（推荐控制任务使用）
const Sbus_RC_Data_t* Sbus_GetLatestData(void);

// 🌟 可选：启用/禁用消息发布（运行时控制）
void Sbus_EnablePublish(bool enable);
#endif //UAV_BAICE_FRAMEWORK_V1_5_SBUS_H