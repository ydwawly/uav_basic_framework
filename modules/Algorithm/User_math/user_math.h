// user_math.h
#ifndef UAV_BAICE_FRAMEWORK_V1_2_USER_MATH_H
#define UAV_BAICE_FRAMEWORK_V1_2_USER_MATH_H

#include <stdint.h>
#include <string.h>  // for memset

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f
#endif

/* ========== 传感器仲裁相关 ========== */
#define GYRO_DIVERGE_THRESHOLD  (30.0f * DEG_TO_RAD)   // 约 0.524 rad/s (30°/s)
#define ACCEL_DIVERGE_THRESHOLD (3.0f)                 // 3 m/s²
#define DIVERGE_CONFIRM_COUNT   50                     // 连续50帧(50ms)确认故障

typedef struct {
    uint16_t diverge_count;       // 连续分歧计数器
    uint16_t diverge_threshold;   // 触发阈值
    uint8_t  bmi088_suspected;    // 0=正常, 1=被怀疑故障
    uint8_t  bmi270_suspected;    // 0=正常, 1=被怀疑故障
    float    prev_fused[3];       // 上一帧融合值（用于连续性检测）
} SensorArbiter_t;

/* ========== 滑动窗口方差估计器 ========== */
// 必须为2的幂次方(32, 64, 128...)，以便用位运算加速取模
#define VAR_WINDOW_SIZE 32

typedef struct {
    float    buffer[VAR_WINDOW_SIZE];  // 环形缓冲区
    uint16_t index;                    // 当前写入位置
    uint16_t count;                    // 当前有效样本数(≤WINDOW_SIZE)
    float    sum;                      // 窗口内数据和
    float    sum_sq;                   // 窗口内数据平方和
} RunningVariance_t;

/* ========== 基础数学函数 ========== */
float constrain_float(float amt, float low, float high);
float Sqrt(float x);
float Dot3d(float *v1, float *v2);
float *Norm3d(float *v);
void Cross3d(float *v1, float *v2, float *res);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

/* ========== 传感器融合相关 ========== */
void CrossValidate_Gyro(float *gyro_088, float *gyro_270, SensorArbiter_t *arb);

void RunningVariance_Init(RunningVariance_t *rv);

/**
 * @brief 动态加权融合（含方差估计更新）
 * @param data_088   BMI088输入数据[3]
 * @param data_270   BMI270输入数据[3]
 * @param rv_088     BMI088方差估计器[3]（X,Y,Z轴）
 * @param rv_270     BMI270方差估计器[3]
 * @param output     融合输出[3]
 * @param arb        仲裁器（用于惩罚故障传感器）
 * @param penalty_factor 怀疑时的方差惩罚系数（建议10.0f）
 */
void FuseWithDynamicWeight(float *data_088, float *data_270,
                           RunningVariance_t *rv_088, RunningVariance_t *rv_270,
                           float *output, SensorArbiter_t *arb,
                           float penalty_factor);

#endif