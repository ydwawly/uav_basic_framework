//
// Created by Administrator on 2026/1/21.
//

#include "quaternionEKF.h"
#include <math.h>
#include <string.h>

#include "BMI088driver.h"

static void IMU_EKF_SetF(KalmanFilter_t *kf)
{
    // 获取父结构体指针 (假设 kf 是 handle 的第一个成员)
    IMU_EKF_Handle_t *att = kf->user_data;

    float dt_half = 0.5f * att->dt;
    float wx = kf->u_data[0];
    float wy = kf->u_data[1];
    float wz = kf->u_data[2];

    // 填充 F 矩阵 (4x4)
    // Row 0
    kf->F_data[0] = 1.0f;
    kf->F_data[1] = -wx * dt_half;
    kf->F_data[2] = -wy * dt_half;
    kf->F_data[3] = -wz * dt_half;
    // Row 1
    kf->F_data[4] = wx * dt_half;
    kf->F_data[5] = 1.0f;
    kf->F_data[6] = wz * dt_half;
    kf->F_data[7] = -wy * dt_half;
    // Row 2
    kf->F_data[8] = wy * dt_half;
    kf->F_data[9] = -wz * dt_half;
    kf->F_data[10] = 1.0f;
    kf->F_data[11] = wx * dt_half;
    // Row 3
    kf->F_data[12] = wz * dt_half;
    kf->F_data[13] = wy * dt_half;
    kf->F_data[14] = -wx * dt_half;
    kf->F_data[15] = 1.0f;
}

static void IMU_EKF_Sethx(KalmanFilter_t *kf,float *h_x_out)
{
    float q0 = kf->xhatminus_data[0];
    float q1 = kf->xhatminus_data[1];
    float q2 = kf->xhatminus_data[2];
    float q3 = kf->xhatminus_data[3];

    // 理论重力向量 h(x)
    h_x_out[0] = 2.0f * (q1 * q3 - q0 * q2);
    h_x_out[1] = 2.0f * (q2 * q3 + q0 * q1);
    h_x_out[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

static void IMU_EKF_SetH(KalmanFilter_t *kf)
{
    float q0 = kf->xhatminus_data[0];
    float q1 = kf->xhatminus_data[1];
    float q2 = kf->xhatminus_data[2];
    float q3 = kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof(float) * kf->zSize * kf->xhatSize);

    // Row 0 (对 ax 的导数)
    kf->H_data[0] = -2.0f * q2;
    kf->H_data[1] =  2.0f * q3;
    kf->H_data[2] = -2.0f * q0;
    kf->H_data[3] =  2.0f * q1;

    // Row 1 (对 ay 的导数)
    kf->H_data[4] =  2.0f * q1;
    kf->H_data[5] =  2.0f * q0;
    kf->H_data[6] =  2.0f * q3;
    kf->H_data[7] =  2.0f * q2;

    // Row 2 (对 az 的导数)
    kf->H_data[8] =  2.0f * q0;
    kf->H_data[9] = -2.0f * q1;
    kf->H_data[10]= -2.0f * q2;
    kf->H_data[11]=  2.0f * q3;
}






void IMU_EKF_Init(IMU_EKF_Handle_t *handle, float* init_quaternion, float process_noise, float measure_noise)
{
    Kalman_Filter_Init(&handle->kf, 4, 3, 3);

    handle->kf.user_data = (void*)handle;
    handle->kf.User_OnStateUpdate = IMU_EKF_SetF;
    handle->kf.User_OnPredictMeas = IMU_EKF_Sethx;
    handle->kf.User_OnMeasUpdate = IMU_EKF_SetH;

    handle->Q = process_noise;
    handle->R = measure_noise;
    // [修正] 先清空所有矩阵数据，防止垃圾值
    memset(handle->kf.Q_data, 0, sizeof(float) * handle->kf.xhatSize * handle->kf.xhatSize); // 4x4
    memset(handle->kf.R_data, 0, sizeof(float) * handle->kf.zSize * handle->kf.zSize);  // 3x3
    memset(handle->kf.P_data, 0, sizeof(float) * handle->kf.xhatSize * handle->kf.xhatSize); // 4x4
    // 3. 初始化 P 矩阵 (初始不确定性)
    for(int i=0; i<4; i++) {
        handle->kf.P_data[i*4 + i] = 0.01f;
    }

    // 4. 初始化 Q 矩阵 (过程噪声)
    for(int i=0; i<4; i++) {
        handle->kf.Q_data[i*4 + i] = handle->Q;
    }

    // 5. 初始化 R 矩阵 (观测噪声)
    for(int i=0; i<3; i++) {
        handle->kf.R_data[i*3 + i] = handle->R;
    }

    // 6. 状态初始化 (q=[1,0,0,0])
    for(int i = 0; i < 4; i++)
    {
        handle->kf.xhat_data[i] = init_quaternion[i];
    }

    handle->chi_square_limit = 3.3f;
}

void IMU_EKF_Update(IMU_EKF_Handle_t *handle,float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    memcpy(handle->xhat_backup, handle->kf.xhat_data, sizeof(float) * 4);
    memcpy(handle->P_backup, handle->kf.P_data, sizeof(float) * 16);

    handle->dt = dt;
    handle->gyro_raw[IMU_X] = gx;
    handle->gyro_raw[IMU_Y] = gy;
    handle->gyro_raw[IMU_Z] = gz;

    handle->accel_raw[IMU_X] = ax;
    handle->accel_raw[IMU_Y] = ay;
    handle->accel_raw[IMU_Z] = az;

    float norm = sqrtf(handle->accel_raw[IMU_X]*handle->accel_raw[IMU_X] +handle->accel_raw[IMU_Y]*handle->accel_raw[IMU_Y] + handle->accel_raw[IMU_Z] * handle->accel_raw[IMU_Z]);
    if(norm > 0.01f)
    {
        handle->accel_raw[IMU_X] /= norm;
        handle->accel_raw[IMU_Y] /= norm;
        handle->accel_raw[IMU_Z] /= norm;

        Kalman_Filter_Update(&handle->kf,handle->gyro_raw,handle->accel_raw);
    }
    else
    {
        Kalman_Filter_Update(&handle->kf,handle->gyro_raw,NULL);
    }

    if (handle->kf.ChiSquare > handle->chi_square_limit)
    {
        // === 发现离群值 (Outlier Detected) ===
        // 此时 xhat 已经被污染了，我们需要回滚！

        // 策略A：完全回滚到上一步状态 (由于已经做过预测，回滚意味着丢失了陀螺仪积分)
        // 策略B：回滚 xhat 和 P，然后以 z=NULL 再跑一次 (仅保留陀螺仪预测) <--- 推荐

        // 恢复数据
        memcpy(handle->kf.xhat_data, handle->xhat_backup, sizeof(float) * 4);
        memcpy(handle->kf.P_data, handle->P_backup, sizeof(float) * 4 * 4);

        // 重新运行一次 Update，但这次不传观测值 (只做预测)
        Kalman_Filter_Update(&handle->kf, handle->gyro_raw, NULL);
    }
    // 更新四元数缓存
    handle->q[0] = handle->kf.xhat_data[0];
    handle->q[1] = handle->kf.xhat_data[1];
    handle->q[2] = handle->kf.xhat_data[2];
    handle->q[3] = handle->kf.xhat_data[3];

    // [修改] 增强的归一化逻辑
    float q_norm_sq = handle->q[0]*handle->q[0] + handle->q[1]*handle->q[1] + handle->q[2]*handle->q[2] + handle->q[3]*handle->q[3];

    // 正常归一化
    float q_norm = sqrtf(q_norm_sq);
    float inv_norm = 1.0f / q_norm;

    handle->kf.xhat_data[0] *= inv_norm;
    handle->kf.xhat_data[1] *= inv_norm;
    handle->kf.xhat_data[2] *= inv_norm;
    handle->kf.xhat_data[3] *= inv_norm;

    handle->q[0] = handle->kf.xhat_data[0];
    handle->q[1] = handle->kf.xhat_data[1];
    handle->q[2] = handle->kf.xhat_data[2];
    handle->q[3] = handle->kf.xhat_data[3];

}

void EKF_GetEulerAngle(IMU_EKF_Handle_t *handle, float *roll, float *pitch, float *yaw)
{
    // 将四元数转换为欧拉角 (Z-Y-X 顺序)
    // 这里的公式取决于你的定义，通常如下：

    // Pitch (俯仰角) -90 ~ 90
    // 注意：2*(q0q2 - q1q3) 可能会因为浮点误差略微大于1，导致 asin 出错 (NaN)
    float pitch_val = 2.0f * (handle->q[0] * handle->q[2] - handle->q[1] * handle->q[3]);
    if (pitch_val > 1.0f) pitch_val = 1.0f;
    if (pitch_val < -1.0f) pitch_val = -1.0f;
    *pitch = asinf(pitch_val) * 57.29578f; // 弧度转度

    // Roll (横滚角) -180 ~ 180
    *roll = atan2f(2.0f * (handle->q[0] * handle->q[1] + handle->q[2] * handle->q[3]),
                   handle->q[0] * handle->q[0] - handle->q[1] * handle->q[1] - handle->q[2] * handle->q[2] + handle->q[3] * handle->q[3]) * 57.29578f;

    // Yaw (偏航角) -180 ~ 180 (注意：六轴融合没有磁力计，Yaw 会随时间缓慢漂移)
    *yaw = atan2f(2.0f * (handle->q[1] * handle->q[2] + handle->q[0] * handle->q[3]),
                  handle->q[0] * handle->q[0] + handle->q[1] * handle->q[1] - handle->q[2] * handle->q[2] - handle->q[3] * handle->q[3]) * 57.29578f;
}