//
// Created by Administrator on 2026/1/26.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_2_USER_MATH_H
#define UAV_BAICE_FRAMEWORK_V1_2_USER_MATH_H

float constrain_float(float amt, float low, float high);

// 快速开方
float Sqrt(float x);
// 三维向量点乘
float Dot3d(float *v1, float *v2);
// 三维向量归一化
float *Norm3d(float *v);
// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res);

/**
 * @brief 机体系(Body)向量 转换到 绝对系(Earth)
 * @note  公式：v_n = q * v_b * q*
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
/**
 * @brief 绝对系(Earth)向量 转换到 机体系(Body)
 * @note  公式：v_b = q* * v_n * q
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
#endif //UAV_BAICE_FRAMEWORK_V1_2_USER_MATH_H