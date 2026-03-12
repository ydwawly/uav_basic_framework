//
// Created by Administrator on 2026/3/5.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_5_IBUS_H
#define UAV_BAICE_FRAMEWORK_V1_5_IBUS_H

#include <stdint.h>

// 标准 i-BUS 协议参数
#define IBUS_FRAME_SIZE    32     // 标准 i-BUS 帧长
#define IBUS_HEADER1       0x20   // i-BUS 帧头 1 (长度)
#define IBUS_HEADER2       0x40   // i-BUS 帧头 2 (指令)

// 通道定义
#define IBUS_CH_TOTAL      14     // 协议最大支持 14 个通道 (FS-I6X 实际输出前 10 个)
#define IBUS_CH_MIN        1000   // 遥控器输出的最小值 (通常在 1000 左右)
#define IBUS_CH_MAX        2000   // 遥控器输出的最大值 (通常在 2000 左右)
#define IBUS_CH_MID        1500   // 中位值

// 解析后的遥控器数据结构体
typedef struct
{
    // 原始通道数据 (range: 通常 1000 ~ 2000)
    uint16_t ch_raw[IBUS_CH_TOTAL];

    // 归一化后的数据 (range: -1.0 ~ 1.0), 方便 PID 计算
    float ch_normalized[IBUS_CH_TOTAL];

    // i-BUS 协议没有像 SBUS 那样专门的丢帧和失控位，通常通过校验和或超时来判断
    uint8_t is_connected; // 连接状态标志 (1: 正常, 0: 丢失/校验错误)
} Ibus_RC_Data_t;

void Ibus_Init(void);
void Ibus_Task_Handler(void);

#endif //UAV_BAICE_FRAMEWORK_V1_5_IBUS_H