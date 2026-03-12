//
// Created by Administrator on 2026/1/20.
//
#include "mahony.h"
// 快速平方根倒数算法 (Quake III Arena legacy)
// 比系统 sqrt 库函数快很多，适合高频解算
static float refx,refy,refz;
static float measx,measy,measz;
static float halfex, halfey, halfez;
float PID_K = 1000;


static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Init(Mahony_Handle_t *handle) {
    handle->q0 = 1.0f;
    handle->q1 = 0.0f;
    handle->q2 = 0.0f;
    handle->q3 = 0.0f;
    handle->integralFBx = 0.0f;
    handle->integralFBy = 0.0f;
    handle->integralFBz = 0.0f;
}

void Mahony_Update(Mahony_Handle_t *handle, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float qa, qb, qc;

    // 1. 如果加速度计数据无效(全0)，则只进行陀螺仪积分，不进行修正
    // if (!((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f))) {
    //     // 直接去步骤5进行积分
    //     // (略，为保持代码结构，这里继续执行，但在实际工程应return)
    //     return;
    // }

    // 2. 加速度计数据归一化
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 3. 计算“理论重力方向”
    halfvx = handle->q1 * handle->q3 - handle->q0 * handle->q2;
    halfvy = handle->q0 * handle->q1 + handle->q2 * handle->q3;
    halfvz = handle->q0 * handle->q0 - 0.5f + handle->q3 * handle->q3;
    // 4. 计算误差 (叉乘)
    halfex = (ay *halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    refx = ay *halfvz;
    refy = az * halfvx;
    refz = ax * halfvy;
    measx = az * halfvy;
    measy = ax * halfvz;
    measz = ay * halfvx;

    // 5. PI 控制器补偿
    // 积分项 (纠正陀螺仪零偏)
    if (MAHONY_KI > 0.0f) {
        handle->integralFBx += MAHONY_KI * halfex * dt;
        handle->integralFBy += MAHONY_KI * halfey * dt;
        handle->integralFBz += MAHONY_KI * halfez * dt;
        gx += handle->integralFBx;
        gy += handle->integralFBy;
        gz += handle->integralFBz;
    } else {
        handle->integralFBx = 0.0f;
        handle->integralFBy = 0.0f;
        handle->integralFBz = 0.0f;
    }

    // 比例项 (直接纠正)
    gx += MAHONY_KP * halfex;
    gy += MAHONY_KP * halfey;
    gz += MAHONY_KP * halfez;

    // 6. 一阶龙格库塔法 (Runge-Kutta) 更新四元数
    // q_new = q_old + 0.5 * q_old * omega * dt
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = handle->q0;
    qb = handle->q1;
    qc = handle->q2;

    handle->q0 += (-qb * gx - qc * gy - handle->q3 * gz);
    handle->q1 += (qa * gx + qc * gz - handle->q3 * gy);
    handle->q2 += (qa * gy - qb * gz + handle->q3 * gx);
    handle->q3 += (qa * gz + qb * gy - qc * gx);

    // 7. 四元数归一化
    // 随着积分运算，四元数模长会偏离1，必须归一化
    recipNorm = invSqrt(handle->q0 * handle->q0 + handle->q1 * handle->q1 + handle->q2 * handle->q2 + handle->q3 * handle->q3);
    handle->q0 *= recipNorm;
    handle->q1 *= recipNorm;
    handle->q2 *= recipNorm;
    handle->q3 *= recipNorm;


}

void Mahony_GetEulerAngle(Mahony_Handle_t *handle, float *roll, float *pitch, float *yaw)
{
    // 将四元数转换为欧拉角 (Z-Y-X 顺序)
    // 这里的公式取决于你的定义，通常如下：

    // Pitch (俯仰角) -90 ~ 90
    // 注意：2*(q0q2 - q1q3) 可能会因为浮点误差略微大于1，导致 asin 出错 (NaN)
    float pitch_val = 2.0f * (handle->q0 * handle->q2 - handle->q1 * handle->q3);
    if (pitch_val > 1.0f) pitch_val = 1.0f;
    if (pitch_val < -1.0f) pitch_val = -1.0f;
    *pitch = asinf(pitch_val) * 57.29578f; // 弧度转度

    // Roll (横滚角) -180 ~ 180
    *roll = atan2f(2.0f * (handle->q0 * handle->q1 + handle->q2 * handle->q3),
                   handle->q0 * handle->q0 - handle->q1 * handle->q1 - handle->q2 * handle->q2 + handle->q3 * handle->q3) * 57.29578f;

    // Yaw (偏航角) -180 ~ 180 (注意：六轴融合没有磁力计，Yaw 会随时间缓慢漂移)
    *yaw = atan2f(2.0f * (handle->q1 * handle->q2 + handle->q0 * handle->q3),
                  handle->q0 * handle->q0 + handle->q1 * handle->q1 - handle->q2 * handle->q2 - handle->q3 * handle->q3) * 57.29578f;
}