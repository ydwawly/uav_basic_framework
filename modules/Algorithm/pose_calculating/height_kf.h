//
// Created by Administrator on 2026/2/9.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_4_HEIGHT_KF_H
#define UAV_BAICE_FRAMEWORK_V1_4_HEIGHT_KF_H

#include "kalman_filter.h"

typedef struct {
    // 1. 继承核心滤波器对象
    KalmanFilter_t kf;

    // 2. 物理量缓存 (用于外部获取)
    float height;     // 当前高度 (m)
    float vertical_speed; // 当前垂直速度 (m/s)
    float accelebias; // 当前垂直加速度 (m/s^2)
    float process_noise_accel,process_noise_bias,measure_noise_height;
} Height_KF;



void Height_KF_Init(Height_KF *handle, float process_noise_accel, float process_noise_bias, float measure_noise_height);
void Height_KF_Update(Height_KF *handle, float *accele_z, float *height_ptr, float dt,float r_noise);

#endif //UAV_BAICE_FRAMEWORK_V1_4_HEIGHT_KF_H