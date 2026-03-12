//
// Created by Administrator on 2026/1/20.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_2_MAHONY_H
#define UAV_BAICE_FRAMEWORK_V1_2_MAHONY_H

#include <math.h>

// --- 参数调优 ---
// Kp: 比例增益，控制加速度计修正陀螺仪的速度。
// 越大收敛越快，但震动时容易过冲；越小抗震越好，但修正慢。
#define MAHONY_KP 700.0f

// Ki: 积分增益，用于消除陀螺仪零偏。
// 需长时间静止调试，通常设很小或为0。
#define MAHONY_KI 0.00f

// 采样周期 (秒)，例如 1000Hz 就是 0.001f
#define MAHONY_SAMPLE_PERIOD 0.001f

typedef struct {
    float q0, q1, q2, q3; // 四元数 (姿态核心)
    float integralFBx, integralFBy, integralFBz; // 积分误差累积 (用于消除零偏)
} Mahony_Handle_t;

/**
 * @brief 初始化 Mahony 算法
 */
void Mahony_Init(Mahony_Handle_t *handle);

/**
 * @brief 6轴融合更新函数
 * @param gx, gy, gz : 陀螺仪数据 (单位：弧度/秒 rad/s) !!!注意单位!!!
 * @param ax, ay, az : 加速度计数据 (单位任意，只要归一化即可，通常用 g)
 */
void Mahony_Update(Mahony_Handle_t *handle, float gx, float gy, float gz, float ax, float ay, float az, float dt);

/**
 * @brief 获取欧拉角 (Roll, Pitch, Yaw)
 * @param roll, pitch, yaw : 输出角度 (单位：度)
 */
void Mahony_GetEulerAngle(Mahony_Handle_t *handle, float *roll, float *pitch, float *yaw);
#endif //UAV_BAICE_FRAMEWORK_V1_2_MAHONY_H