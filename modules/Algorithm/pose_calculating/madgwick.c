//
// Created by Administrator on 2026/1/20.
//

#include "madgwick.h"

// 快速平方根倒数 (经典算法)
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Madgwick_Init(Madgwick_Handle_t *handle) {
    handle->q0 = 1.0f;
    handle->q1 = 0.0f;
    handle->q2 = 0.0f;
    handle->q3 = 0.0f;
}

void Madgwick_Update(Madgwick_Handle_t *handle, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 0. 指针转局部变量，加快访问速度
    float q0 = handle->q0;
    float q1 = handle->q1;
    float q2 = handle->q2;
    float q3 = handle->q3;

    // 1. 陀螺仪积分步 (Rate of change of quaternion from gyroscope)
    // qDot = 0.5 * q * omega
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // 2. 如果加速度计数据无效，则仅使用陀螺仪积分
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 3. 加速度计归一化
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 4. 辅助变量计算 (避免重复运算)
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 5. 梯度下降算法 (Gradient Descent)
        // 计算误差函数的梯度 (Gradient of the error function)
        // 这些公式推导自 Jacobian 矩阵，目的是找到重力方向的最优解
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        // 梯度归一化 (Normalize the gradient)
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 6. 融合：陀螺仪积分 - Beta * 梯度方向
        qDot1 -= MADGWICK_BETA * s0;
        qDot2 -= MADGWICK_BETA * s1;
        qDot3 -= MADGWICK_BETA * s2;
        qDot4 -= MADGWICK_BETA * s3;
    }

    // 7. 更新四元数 (Integrate to yield quaternion)
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 8. 四元数归一化 (Normalize quaternion)
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    handle->q0 = q0 * recipNorm;
    handle->q1 = q1 * recipNorm;
    handle->q2 = q2 * recipNorm;
    handle->q3 = q3 * recipNorm;
}

// 欧拉角转换函数 (同 Mahony，通用)
void Madgwick_GetEulerAngle(Madgwick_Handle_t *handle, float *roll, float *pitch, float *yaw) {
    float q0 = handle->q0;
    float q1 = handle->q1;
    float q2 = handle->q2;
    float q3 = handle->q3;

    float pitch_val = 2.0f * (q0 * q2 - q1 * q3);
    if (pitch_val > 1.0f) pitch_val = 1.0f;
    if (pitch_val < -1.0f) pitch_val = -1.0f;
    *pitch = asinf(pitch_val) * 57.29578f;

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   q0*q0 - q1*q1 - q2*q2 + q3*q3) * 57.29578f;

    *yaw = atan2f(2.0f * (q1 * q2 + q0 * q3),
                  q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.29578f;
}