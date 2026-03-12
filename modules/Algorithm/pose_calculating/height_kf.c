#include "height_kf.h"
#include <string.h> // for memset

void Height_KF_Init(Height_KF *handle, float process_noise_accel, float process_noise_bias, float measure_noise_height)
{
    // 1. 初始化: 3状态(h, v, bias), 1观测(h), 1输入(accel)
    Kalman_Filter_Init(&handle->kf, 3, 1, 1);

    // 2. 清空矩阵 (使用实际分配的大小，假设库内部根据维度分配了足够的空间)
    // 注意：这里建议使用库自带的 clear 或者标准的 memset，确保大小计算正确
    // 假设 xhat_data 等指针指向的是连续内存
    int s_dim = handle->kf.xhatSize; // 3
    int z_dim = handle->kf.zSize;    // 1

    memset(handle->kf.P_data, 0, sizeof(float) * s_dim * s_dim);
    memset(handle->kf.Q_data, 0, sizeof(float) * s_dim * s_dim);
    memset(handle->kf.R_data, 0, sizeof(float) * z_dim * z_dim);
    memset(handle->kf.xhat_data, 0, sizeof(float) * s_dim);

    // 3. 初始化 P 矩阵 (初始协方差)
    // 对角线依次为: 高度误差, 速度误差, 零偏误差
    handle->kf.P_data[0*3 + 0] = 1.0f;   // 高度初始不确定性
    handle->kf.P_data[1*3 + 1] = 0.5f;   // 速度初始不确定性
    handle->kf.P_data[2*3 + 2] = 0.01f;  // 零偏初始不确定性

    // 4. 初始化 Q 矩阵 (过程噪声协方差) - 核心调参部分
    // 模型: x = Fx + B(u + w), w是加速度噪声
    // Q矩阵通常对应 B*Var(a)*B' 或者直接独立设置状态噪声
    // 这里采用简化设置：
    // Q[0][0] (高度过程噪声): 极小，主要靠速度积分
    // Q[1][1] (速度过程噪声): 对应加速度计的噪声特性
    // Q[2][2] (零偏过程噪声): 极小，假设零偏几乎不变

    handle->kf.Q_data[0*3 + 0] = process_noise_accel * 0.01f;
    handle->kf.Q_data[1*3 + 1] = process_noise_accel;
    handle->kf.Q_data[2*3 + 2] = process_noise_bias; // 这一项必须很小! 例如 1e-5


    // 5. 初始化 R 矩阵 (观测噪声协方差)
    // 只有 1 个观测值
    handle->kf.R_data[0] = measure_noise_height;
    handle->process_noise_accel = process_noise_accel;
    handle->process_noise_bias = process_noise_bias;
    handle->measure_noise_height = measure_noise_height;

}

void Height_KF_Update(Height_KF *handle, float *accele_z, float *height_ptr, float dt,float r_noise)
{
    // 假设矩阵存储是行优先 (Row-Major): index = row * col_num + col

    // 1. 更新状态转移矩阵 F (3x3)
    // [1,  dt, -0.5*dt^2]
    // [0,  1,  -dt      ]
    // [0,  0,  1        ]
    float *F = handle->kf.F_data;
    F[0] = 1.0f; F[1] = dt;   F[2] = -0.5f * dt * dt;
    F[3] = 0.0f; F[4] = 1.0f; F[5] = -dt;
    F[6] = 0.0f; F[7] = 0.0f; F[8] = 1.0f;

    float qa = handle->process_noise_accel;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    float *Q = handle->kf.Q_data;
    memset(Q, 0, sizeof(float) * 9);

    Q[0*3+0] = qa * 0.25f * dt4;
    Q[0*3+1] = qa * 0.5f  * dt3;
    Q[1*3+0] = qa * 0.5f  * dt3;
    Q[1*3+1] = qa * dt2;
    Q[2*3+2] = handle->process_noise_bias * dt;
    // 2. 更新控制矩阵 B (3x1)
    // [0.5*dt^2]
    // [dt      ]
    // [0       ]
    // 注意：输入的 accele_z 必须是 (运动加速度 - 重力)，即去除重力后的垂直分量
    float *B = handle->kf.B_data;
    B[0] = 0.5f * dt * dt;
    B[1] = dt;
    B[2] = 0.0f; // 加速度计读数不直接影响零偏状态的更新

    // 3. 更新观测矩阵 H (1x3)
    // [1, 0, 0]
    float *H = handle->kf.H_data;
    H[0] = 1.0f; H[1] = 0.0f; H[2] = 0.0f;
    if (height_ptr != NULL) {
        handle->kf.R_data[0] = r_noise;
    }
    // 4. 构造观测向量 z
    Kalman_Filter_Update(&handle->kf, accele_z, height_ptr);

    // 6. 输出限制 (可选，防止发散)
    // if(handle->kf.xhat_data[2] > 2.0f) handle->kf.xhat_data[2] = 2.0f;

    // 7. 更新结果
    handle->height = handle->kf.xhat_data[0];
    handle->vertical_speed = handle->kf.xhat_data[1];
    handle->accelebias = handle->kf.xhat_data[2];
}