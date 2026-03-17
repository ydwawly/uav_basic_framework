//
// Created by Administrator on 2026/3/13.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_6_MAVLINK_USER_H
#define UAV_BAICE_FRAMEWORK_V1_6_MAVLINK_USER_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

// MAVLink 系统参数
#define MAVLINK_SYSTEM_ID      1
#define MAVLINK_COMPONENT_ID   1

// RingBuffer 大小（必须是 2 的幂）
#define MAVLINK_RX_BUF_SIZE    (4 * 1024)   // 4KB 接收缓冲
#define MAVLINK_TX_BUF_SIZE    (2 * 1024)   // 2KB 发送缓冲

void Mavlink_Init(void);
void Comm_Task(void *pvParameters);

// 统计接口
typedef struct {
    uint32_t rx_packets;        // 成功解析的包数
    uint32_t rx_errors;         // CRC 错误包数
    uint32_t rx_buffer_drops;   // 缓冲区满丢包数
    uint32_t tx_packets;        // 发送包数
} Mavlink_Stats_t;

void Mavlink_GetStats(Mavlink_Stats_t *stats);


#endif //UAV_BAICE_FRAMEWORK_V1_6_MAVLINK_USER_H