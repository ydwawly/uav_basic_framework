//
// Created by Administrator on 2026/1/21.
// Updated by Gemini on 2026/2/08.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_2_KALMAN_FILTER_H
#define UAV_BAICE_FRAMEWORK_V1_2_KALMAN_FILTER_H

#include "arm_math.h"
#include <string.h>

// ================= 配置区域 =================
// 根据你的实际需求修改最大维数
// 例如：12维状态（位置3+速度3+姿态3+零漂3），6维观测
#define KF_MAX_STATE_DIM 12
#define KF_MAX_MEAS_DIM  6
#define KF_MAX_CTRL_DIM  6
// ===========================================

typedef struct kf_t
{
    // === 核心维度信息 ===
    uint8_t xhatSize; // 状态维数
    uint8_t zSize;    // 观测维数
    uint8_t uSize;    // 控制维数

    // === 矩阵实例 (CMSIS-DSP 句柄) ===
    arm_matrix_instance_f32 xhat;      // 状态估计 x(k|k)
    arm_matrix_instance_f32 xhatminus; // 先验状态估计 x(k|k-1)
    arm_matrix_instance_f32 u;         // 控制输入 u(k)
    arm_matrix_instance_f32 z;         // 观测值 z(k) (同时用于存储残差)
    arm_matrix_instance_f32 P;         // 估计误差协方差 P(k|k)
    arm_matrix_instance_f32 Pminus;    // 先验估计误差协方差 P(k|k-1)
    arm_matrix_instance_f32 F, FT;     // 状态转移矩阵 F, FT
    arm_matrix_instance_f32 B;         // 控制输入矩阵 B
    arm_matrix_instance_f32 H, HT;     // 观测矩阵 H, HT
    arm_matrix_instance_f32 R;         // 观测噪声协方差 R
    arm_matrix_instance_f32 Q;         // 过程噪声协方差 Q
    arm_matrix_instance_f32 K;         // 卡尔曼增益 K

    // === 中间计算矩阵 (必须保留以避免栈溢出和重入问题) ===
    arm_matrix_instance_f32 S, S_inv;        // S = H*P*H' + R
    arm_matrix_instance_f32 temp_matrix_PxH; // P*H' (x*z)
    arm_matrix_instance_f32 temp_matrix_KxH; // K*H  (x*x) 或 F*P (x*x) 复用
    arm_matrix_instance_f32 temp_matrix_ZxX; // H*P- (z*x) [新增: 修复维度Bug专用]
    arm_matrix_instance_f32 temp_vector_Hx;  // H*x  (z*1)
    arm_matrix_instance_f32 temp_vector_Bu;  // B*u  (x*1) [新增: 控制量中间值]
    arm_matrix_instance_f32 temp_vector_Kz;  // K*y  (x*1) [新增: 状态修正量]

    // === 静态数据存储区 ===
    float xhat_data[KF_MAX_STATE_DIM];
    float xhatminus_data[KF_MAX_STATE_DIM];
    float u_data[KF_MAX_CTRL_DIM];
    float z_data[KF_MAX_MEAS_DIM];
    float P_data[KF_MAX_STATE_DIM * KF_MAX_STATE_DIM];
    float Pminus_data[KF_MAX_STATE_DIM * KF_MAX_STATE_DIM];
    float F_data[KF_MAX_STATE_DIM * KF_MAX_STATE_DIM];
    float FT_data[KF_MAX_STATE_DIM * KF_MAX_STATE_DIM];
    float B_data[KF_MAX_STATE_DIM * KF_MAX_CTRL_DIM];
    float H_data[KF_MAX_MEAS_DIM * KF_MAX_STATE_DIM];
    float HT_data[KF_MAX_STATE_DIM * KF_MAX_MEAS_DIM];
    float Q_data[KF_MAX_STATE_DIM * KF_MAX_STATE_DIM];
    float R_data[KF_MAX_MEAS_DIM * KF_MAX_MEAS_DIM];
    float K_data[KF_MAX_STATE_DIM * KF_MAX_MEAS_DIM];
    float S_data[KF_MAX_MEAS_DIM * KF_MAX_MEAS_DIM];
    float S_inv_data[KF_MAX_MEAS_DIM * KF_MAX_MEAS_DIM];
    float ChiSquare;

    // === 临时缓冲区数据 ===
    float temp_matrix_PxH_data[KF_MAX_STATE_DIM * KF_MAX_MEAS_DIM];
    float temp_matrix_KxH_data[KF_MAX_STATE_DIM * KF_MAX_STATE_DIM];
    float temp_matrix_ZxX_data[KF_MAX_MEAS_DIM * KF_MAX_STATE_DIM]; // [新增]
    float temp_vector_Hx_data[KF_MAX_MEAS_DIM];
    float temp_vector_Bu_data[KF_MAX_STATE_DIM]; // [新增]
    float temp_vector_Kz_data[KF_MAX_STATE_DIM]; // [新增]

    // 输出结果指针
    float *FilteredValue;

    void *user_data;
    // === 用户回调函数 (用于扩展 EKF / UKF) ===
    void (*User_OnStateUpdate)(struct kf_t *kf); // 更新 F 矩阵 (Jacobian)
    void (*User_OnMeasUpdate)(struct kf_t *kf);  // 更新 H 矩阵 (Jacobian)

    // === EKF 专用扩展 ===
    // 1. 存储非线性预测观测值 h(x) 的缓存 (维度与 z 相同)
    arm_matrix_instance_f32 z_predict;
    float z_predict_data[KF_MAX_MEAS_DIM];

    // 2. 非线性观测方程回调函数指针
    // 参数说明: kf本身, h_x_out用于输出计算结果
    void (*User_OnPredictMeas)(struct kf_t *kf, float *h_x_out);
} KalmanFilter_t;

/**
 * @brief 初始化卡尔曼滤波器
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);

/**
 * @brief 执行卡尔曼滤波更新
 * @param u_input 控制输入 (如果无控制量传 NULL)
 * @param z_measure 测量值 (如果仅做预测传 NULL)
 * @return float* 指向状态估计数组的指针
 */
float* Kalman_Filter_Update(KalmanFilter_t *kf, float *u_input, float *z_measure);

#endif //UAV_BAICE_FRAMEWORK_V1_2_KALMAN_FILTER_H