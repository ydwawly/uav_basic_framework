//
// Created by Administrator on 2026/1/21.
// Updated by Gemini on 2026/2/08.
//

#include "kalman_filter.h"

/**
 * @brief 初始化卡尔曼滤波器，绑定静态内存
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    // 1. 参数检查 (防止数组越界)
    if (xhatSize > KF_MAX_STATE_DIM || zSize > KF_MAX_MEAS_DIM || uSize > KF_MAX_CTRL_DIM)
    {
        while(1); // 错误陷阱
    }
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    // 2. 初始化矩阵 (CMSIS-DSP)
    // 状态向量
    arm_mat_init_f32(&kf->xhat, xhatSize, 1, kf->xhat_data);
    arm_mat_init_f32(&kf->xhatminus, xhatSize, 1, kf->xhatminus_data);

    // 协方差矩阵
    arm_mat_init_f32(&kf->P, xhatSize, xhatSize, kf->P_data);
    arm_mat_init_f32(&kf->Pminus, xhatSize, xhatSize, kf->Pminus_data);

    // 状态转移与控制
    arm_mat_init_f32(&kf->F, xhatSize, xhatSize, kf->F_data);
    arm_mat_init_f32(&kf->FT, xhatSize, xhatSize, kf->FT_data);

    if (uSize > 0) {
        arm_mat_init_f32(&kf->u, uSize, 1, kf->u_data);
        arm_mat_init_f32(&kf->B, xhatSize, uSize, kf->B_data);
    }

    // 观测矩阵
    arm_mat_init_f32(&kf->z, zSize, 1, kf->z_data);
    arm_mat_init_f32(&kf->H, zSize, xhatSize, kf->H_data);
    arm_mat_init_f32(&kf->HT, xhatSize, zSize, kf->HT_data);

    // 噪声矩阵
    arm_mat_init_f32(&kf->Q, xhatSize, xhatSize, kf->Q_data);
    arm_mat_init_f32(&kf->R, zSize, zSize, kf->R_data);

    // 增益与中间变量
    arm_mat_init_f32(&kf->K, xhatSize, zSize, kf->K_data);
    arm_mat_init_f32(&kf->S, zSize, zSize, kf->S_data);
    arm_mat_init_f32(&kf->S_inv, zSize, zSize, kf->S_inv_data);

    // === 临时矩阵初始化 (关键修改) ===
    arm_mat_init_f32(&kf->temp_matrix_PxH, xhatSize, zSize, kf->temp_matrix_PxH_data);
    arm_mat_init_f32(&kf->temp_matrix_KxH, xhatSize, xhatSize, kf->temp_matrix_KxH_data);
    arm_mat_init_f32(&kf->temp_matrix_ZxX, zSize, xhatSize, kf->temp_matrix_ZxX_data); // [New]

    arm_mat_init_f32(&kf->temp_vector_Hx, zSize, 1, kf->temp_vector_Hx_data);
    arm_mat_init_f32(&kf->temp_vector_Bu, xhatSize, 1, kf->temp_vector_Bu_data);      // [New]
    arm_mat_init_f32(&kf->temp_vector_Kz, xhatSize, 1, kf->temp_vector_Kz_data);      // [New]

    // 3. 数据清零
    memset(kf->xhat_data, 0, sizeof(kf->xhat_data));
    memset(kf->P_data, 0, sizeof(kf->P_data)); // 注意：用户需在外部将 P 对角线初始化为非零值

    // 清空新增的 buffer，防止脏数据
    memset(kf->temp_matrix_ZxX_data, 0, sizeof(kf->temp_matrix_ZxX_data));
    memset(kf->temp_vector_Bu_data, 0, sizeof(kf->temp_vector_Bu_data));
    memset(kf->temp_vector_Kz_data, 0, sizeof(kf->temp_vector_Kz_data));

    kf->FilteredValue = kf->xhat_data;

    // EKF 预测观测向量
    arm_mat_init_f32(&kf->z_predict, zSize, 1, kf->z_predict_data);

    kf->user_data = NULL;
    // 初始化回调为空
    kf->User_OnPredictMeas = NULL;
    kf->User_OnStateUpdate = NULL;
    kf->User_OnMeasUpdate = NULL;
}

/**
 * @brief 执行卡尔曼滤波更新
 */
float* Kalman_Filter_Update(KalmanFilter_t *kf, float *u_input, float *z_measure)
{
    // === 0. 数据装载 ===
    if (u_input != NULL && kf->uSize > 0)
    {
        memcpy(kf->u_data, u_input, sizeof(float) * kf->uSize);
    }
    if (z_measure != NULL)
    {
        memcpy(kf->z_data, z_measure, sizeof(float) * kf->zSize);
    }

    // [User Hook] EKF 更新 F 矩阵
    if (kf->User_OnStateUpdate != NULL)
    {
        kf->User_OnStateUpdate(kf);
    }

    // === 1. 预测状态 (Predict State) ===
    // x(k|k-1) = F * x(k-1|k-1)
    arm_mat_mult_f32(&kf->F, &kf->xhat, &kf->xhatminus);

    // 如果有控制量: x = Fx + Bu
    if (kf->uSize > 0 && kf->User_OnStateUpdate == NULL)
    {
        // temp_vector_Bu = B * u
        arm_mat_mult_f32(&kf->B, &kf->u, &kf->temp_vector_Bu);
        // xhatminus = xhatminus + temp_vector_Bu
        arm_mat_add_f32(&kf->xhatminus, &kf->temp_vector_Bu, &kf->xhatminus);
    }

    // === 2. 预测协方差 (Predict Covariance) ===
    // P(k|k-1) = F * P(k-1|k-1) * F^T + Q

    arm_mat_trans_f32(&kf->F, &kf->FT); // 更新 FT

    // temp_KxH (复用为 x*x 矩阵) = F * P
    arm_mat_mult_f32(&kf->F, &kf->P, &kf->temp_matrix_KxH);

    // Pminus = temp_KxH * FT
    arm_mat_mult_f32(&kf->temp_matrix_KxH, &kf->FT, &kf->Pminus);

    // Pminus = Pminus + Q
    arm_mat_add_f32(&kf->Pminus, &kf->Q, &kf->Pminus);

    // [User Hook] EKF 更新 H 矩阵
    if (kf->User_OnMeasUpdate != NULL) {
        kf->User_OnMeasUpdate(kf);
    }

    // [分支] 如果没有测量数据，直接跳过更新
    if (z_measure == NULL)
    {
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof(float) * kf->xhatSize * kf->xhatSize);
        return kf->FilteredValue;
    }

    // === 3. 计算卡尔曼增益 (Calculate Gain) ===
    // K = Pminus * HT * inv(H * Pminus * HT + R)

    arm_mat_trans_f32(&kf->H, &kf->HT); // 更新 HT

    // temp_PxH = Pminus * HT (x*x * x*z = x*z)
    arm_mat_mult_f32(&kf->Pminus, &kf->HT, &kf->temp_matrix_PxH);

    // S = H * temp_PxH + R (z*x * x*z = z*z)
    arm_mat_mult_f32(&kf->H, &kf->temp_matrix_PxH, &kf->S);
    arm_mat_add_f32(&kf->S, &kf->R, &kf->S);

    // 求逆 inv(S)
    arm_status status = arm_mat_inverse_f32(&kf->S, &kf->S_inv);

    // 数值稳定性保护
    if (status != ARM_MATH_SUCCESS) {
        // 矩阵奇异，放弃更新
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof(float) * kf->xhatSize * kf->xhatSize);
        return kf->FilteredValue;
    }

    // K = temp_PxH * S_inv (x*z * z*z = x*z)
    arm_mat_mult_f32(&kf->temp_matrix_PxH, &kf->S_inv, &kf->K);

    // === 4. 量测更新 (Measurement Update) ===
    // y = z - H * x(k|k-1)

    if (kf->User_OnPredictMeas != NULL)
    {
        // EKF: y = z - h(x)
        kf->User_OnPredictMeas(kf, kf->z_predict_data);
        arm_mat_sub_f32(&kf->z, &kf->z_predict, &kf->z);
    }
    else
    {
        // Linear: y = z - Hx
        arm_mat_mult_f32(&kf->H, &kf->xhatminus, &kf->temp_vector_Hx);
        arm_mat_sub_f32(&kf->z, &kf->temp_vector_Hx, &kf->z);
    }

    // === 3. 计算卡方值 (Chi-Square / NIS) ===
    // 注意：此时 kf->z 存放的是残差 (y = z_meas - Hx)
    // 此时 kf->S_inv 存放的是 S 的逆矩阵

    // 步骤 A: temp_vec = S_inv * y
    // (z*z) * (z*1) = (z*1)
    // 我们复用 temp_vector_Hx 作为临时存储，因为它大小正好是 zSize
    arm_mat_mult_f32(&kf->S_inv, &kf->z, &kf->temp_vector_Hx);

    // 步骤 B: Chi = y' * temp_vec
    // (1*z) * (z*1) = scalar
    // 使用点积函数加速
    float chi_val = 0.0f;
    arm_dot_prod_f32(kf->z_data, kf->temp_vector_Hx_data, kf->zSize, &chi_val);

    kf->ChiSquare = chi_val; // <--- 存入结构体，供外部读取

    // temp_vector_Kz = K * y (x*z * z*1 = x*1)
    arm_mat_mult_f32(&kf->K, &kf->z, &kf->temp_vector_Kz);

    // xhat = xhatminus + temp_vector_Kz
    arm_mat_add_f32(&kf->xhatminus, &kf->temp_vector_Kz, &kf->xhat);

    // === 5. 协方差更新 (Update Covariance) ===
    // P = Pminus - K * (H * Pminus)
    // 之前版本的 BUG 修正：必须保证中间矩阵维度匹配

    // 1. temp_ZxX = H * Pminus (z*x * x*x = z*x)
    // 注意：这里必须用 z*x 的专用矩阵，不能复用 HT (x*z)
    arm_mat_mult_f32(&kf->H, &kf->Pminus, &kf->temp_matrix_ZxX);

    // 2. temp_KxH = K * temp_ZxX (x*z * z*x = x*x)
    // 这里复用 temp_matrix_KxH 是安全的，因为它是 x*x
    arm_mat_mult_f32(&kf->K, &kf->temp_matrix_ZxX, &kf->temp_matrix_KxH);

    // 3. P = Pminus - temp_KxH
    arm_mat_sub_f32(&kf->Pminus, &kf->temp_matrix_KxH, &kf->P);

    return kf->FilteredValue;
}