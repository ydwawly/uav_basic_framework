//
// Created by Administrator on 2026/1/26.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_2_QUADROTOR_H
#define UAV_BAICE_FRAMEWORK_V1_2_QUADROTOR_H
#include "pid.h"

// 宏定义：根据你的电调(ESC)协议调整
// 如果是 PWM 协议，通常是 1000~2000
// 如果是比例协议，通常是 0.0~1.0
#define MOTOR_MAX 1000  // 假设基础油门最大值 (或者 1.0)
#define MOTOR_MIN 0     // 假设基础油门最小值 (或者 0.0)

typedef struct
{
    PIDInstance *roll_pid;
    PIDInstance *pitch_pid;
    PIDInstance *yaw_pid;
    PIDInstance *roll_rate_pid;
    PIDInstance *pitch_rate_pid;
    PIDInstance *yaw_rate_pid;
    PIDInstance *height_pid;

    float throttle[4]; // 四个电机的油门输出
}Quadrotor_t;

void Quadrotor_init();
void Quadrotor_Task();



#endif //UAV_BAICE_FRAMEWORK_V1_2_QUADROTOR_H