//
// Created by Administrator on 2026/3/13.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_6_MAVLINK_USER_H
#define UAV_BAICE_FRAMEWORK_V1_6_MAVLINK_USER_H

#include "FreeRTOS.h"
#include "task.h"

// 飞控在 MAVLink 网络中的“身份证”
#define MAVLINK_SYSTEM_ID      1    // 当前无人机编号为 1
#define MAVLINK_COMPONENT_ID   MAV_COMP_ID_AUTOPILOT1 // 代表主飞控

/**
 * @brief MAVLink 应用层初始化 (在 Comm_Task 中调用)
 */
void Mavlink_Init(void);

/**
 * @brief 通信任务入口 (50Hz)
 */
void Comm_Task(void *pvParameters);

#endif //UAV_BAICE_FRAMEWORK_V1_6_MAVLINK_USER_H