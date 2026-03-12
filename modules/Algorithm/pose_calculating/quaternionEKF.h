//
// Created by Administrator on 2026/1/21.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_2_QUATERNIONEKF_H
#define UAV_BAICE_FRAMEWORK_V1_2_QUATERNIONEKF_H

#include "kalman_filter.h"

typedef struct {
    // 1. 继承核心滤波器对象
    KalmanFilter_t kf;

    // 2. 物理量缓存 (用于外部获取)
    float q[4];      // 四元数

    // 3. 内部缓存 (用于回调函数传参)
    float gyro_raw[3]; // 当前时刻的陀螺仪读数 (rad/s)
    float accel_raw[3];
    float dt;          // 当前时刻的时间间隔 (s)
    float Q; // 四元数更新过程噪声
    float R;  // 加速度计量测噪声
    // 卡方阈值 (应用层逻辑)
    float chi_square_limit;

    // === 状态备份区 (用于回滚) ===
    float xhat_backup[4];
    float P_backup[16];
} IMU_EKF_Handle_t;

/**
 * @brief 初始化 IMU EKF
 */
void IMU_EKF_Init(IMU_EKF_Handle_t *handle, float* init_quaternion ,float process_noise, float measure_noise);

/**
 * @brief 更新步 (放入定时器或数据中断中)
 * @param gyro: 陀螺仪数据 [x, y, z] 单位: rad/s
 * @param accel: 加速度计数据 [x, y, z] 单位: m/s^2 或 g (需归一化)
 * @param dt: 时间间隔 (s)
 */
void IMU_EKF_Update(IMU_EKF_Handle_t *handle,float gx, float gy, float gz, float ax, float ay, float az, float dt);

/**
 * @brief 获取欧拉角
 * @param rpy: 返回 [Roll, Pitch, Yaw] 单位: 度
 */
void EKF_GetEulerAngle(IMU_EKF_Handle_t *handle, float *roll, float *pitch, float *yaw);


#endif //UAV_BAICE_FRAMEWORK_V1_2_QUATERNIONEKF_H