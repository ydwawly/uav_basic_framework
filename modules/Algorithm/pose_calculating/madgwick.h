//
// Created by Administrator on 2026/1/20.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_2_MADGWICK_H
#define UAV_BAICE_FRAMEWORK_V1_2_MADGWICK_H

#include <math.h>

// --- 参数调优 ---
// Beta: 算法增益。
// 物理意义：sqrt(3/4) * Gyro_Measurement_Error (rad/s)
// 经验值：0.03f ~ 0.5f。
// 如果发现收敛慢（类似Mahony现象），增大 Beta；如果静止时角度乱跳，减小 Beta。
#define MADGWICK_BETA  10.0f

typedef struct {
    float q0, q1, q2, q3; // 四元数
} Madgwick_Handle_t;

void Madgwick_Init(Madgwick_Handle_t *handle);

/**
 * @brief 6轴 Madgwick 更新
 * @param gx, gy, gz : 弧度/秒 (rad/s)
 * @param ax, ay, az : 加速度 (归一化即可)
 * @param dt : 采样周期 (s)
 */
void Madgwick_Update(Madgwick_Handle_t *handle, float gx, float gy, float gz, float ax, float ay, float az, float dt);

void Madgwick_GetEulerAngle(Madgwick_Handle_t *handle, float *roll, float *pitch, float *yaw);
#endif //UAV_BAICE_FRAMEWORK_V1_2_MADGWICK_H