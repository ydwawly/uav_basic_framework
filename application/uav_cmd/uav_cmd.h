//
// Created by Administrator on 2026/2/28.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_5_UAV_CMD_H
#define UAV_BAICE_FRAMEWORK_V1_5_UAV_CMD_H

#include <stdint.h>

// 1. 定义飞行器当前状态机
typedef enum {
    UAV_STATE_DISARMED = 0, // 上锁状态 (电机停转)
    UAV_STATE_ARMED         // 解锁状态 (准备起飞)
} Uav_Arm_State_e;

// 2. 定义飞行模式
typedef enum {
    UAV_MODE_MANUAL = 0,    // 纯手动模式 (类似穿梭机Acro)
    UAV_MODE_STABILIZE,     // 自稳模式 (摇杆控制角度，松手自动回平)
    UAV_MODE_ALTHOLD        // 定高模式
} Uav_Flight_Mode_e;

// 3. 核心：期望设定值数据包 (发布给控制任务的凭证)
typedef struct {
    Uav_Arm_State_e   arm_state;
    Uav_Flight_Mode_e flight_mode;

    float roll_ref;       // 期望横滚 (度)
    float pitch_ref;      // 期望俯仰 (度)
    float yaw_ref;
    float yaw_rate_ref;   // 期望偏航角速度 (度/秒)
    float throttle_base;  // 基础油门 (0.0 ~ 1.0)
} Uav_Cmd_Data_t;

void Uav_Cmd_init(void);
void Uav_Cmd_Task(void *pvParameters);




#endif //UAV_BAICE_FRAMEWORK_V1_5_UAV_CMD_H