// user_math.c
#include "user_math.h"
#include <math.h>

float constrain_float(float amt, float low, float high) {
    if (amt < low) return low;
    if (amt > high) return high;
    return amt;
}

// 快速开方（牛顿迭代）
float Sqrt(float x) {
    float y, delta, maxError;
    if (x <= 0) return 0;
    y = x / 2;
    maxError = x * 0.001f;
    do {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);
    return y;
}

float Dot3d(float *v1, float *v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

float *Norm3d(float *v) {
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 1e-6f) {
        v[0] /= len; v[1] /= len; v[2] /= len;
    }
    return v;
}

void Cross3d(float *v1, float *v2, float *res) {
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);
    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);
    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);
    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);
    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/* ========== 滑动窗口方差实现 ========== */

void RunningVariance_Init(RunningVariance_t *rv) {
    memset(rv, 0, sizeof(RunningVariance_t));
}

/**
 * @brief 更新滑动窗口方差（Welford算法变体，数值稳定）
 * @return 当前方差估计值
 */
static float RunningVariance_Update_Internal(RunningVariance_t *rv, float new_val) {
    // 如果窗口已满，移除最老的样本
    if (rv->count >= VAR_WINDOW_SIZE) {
        float old_val = rv->buffer[rv->index];
        rv->sum -= old_val;
        rv->sum_sq -= old_val * old_val;
    } else {
        rv->count++;
    }

    // 存入新样本
    rv->buffer[rv->index] = new_val;
    rv->sum += new_val;
    rv->sum_sq += new_val * new_val;

    // 环形缓冲区指针推进（VAR_WINDOW_SIZE为2的幂，可用位与代替取模）
    rv->index = (rv->index + 1) & (VAR_WINDOW_SIZE - 1);

    // 计算方差: Var = E[X^2] - (E[X])^2
    if (rv->count < 2) return 1.0f;  // 样本不足，返回较大值表示不确定

    float mean = rv->sum / (float)rv->count;
    float mean_sq = rv->sum_sq / (float)rv->count;
    float variance = mean_sq - mean * mean;

    // 数值保护：方差不可能为负（浮点误差可能导致极小负值）
    if (variance < 1e-8f) variance = 1e-8f;

    return variance;
}

/* ========== 交叉校验实现 ========== */

void CrossValidate_Gyro(float *gyro_088, float *gyro_270, SensorArbiter_t *arb) {
    // 计算三轴最大偏差
    float max_diff = 0.0f;
    for (int i = 0; i < 3; i++) {
        float diff = fabsf(gyro_088[i] - gyro_270[i]);
        if (diff > max_diff) max_diff = diff;
    }

    // 检查是否超过阈值
    if (max_diff > GYRO_DIVERGE_THRESHOLD) {
        arb->diverge_count++;

        // 连续多次超标，判定为故障
        if (arb->diverge_count >= arb->diverge_threshold) {
            // 故障判定策略：谁离"历史轨迹"远，谁就是故障方
            // 历史轨迹用 prev_fused 表示（上一帧的融合值，代表可信的历史状态）
            float err_088 = 0.0f, err_270 = 0.0f;

            for (int i = 0; i < 3; i++) {
                err_088 += fabsf(gyro_088[i] - arb->prev_fused[i]);
                err_270 += fabsf(gyro_270[i] - arb->prev_fused[i]);
            }

            // 重置怀疑标志
            arb->bmi088_suspected = 0;
            arb->bmi270_suspected = 0;

            // 偏离历史值更大的那个被标记为怀疑
            if (err_088 > err_270) {
                arb->bmi088_suspected = 1;
            } else {
                arb->bmi270_suspected = 1;
            }
        }
    } else {
        // 差异正常，清零计数器（允许偶尔的单帧毛刺）
        arb->diverge_count = 0;
        arb->bmi088_suspected = 0;
        arb->bmi270_suspected = 0;
    }
}

/* ========== 动态加权融合实现 ========== */

void FuseWithDynamicWeight(float *data_088, float *data_270,
                           RunningVariance_t *rv_088, RunningVariance_t *rv_270,
                           float *output, SensorArbiter_t *arb,
                           float penalty_factor) {
    for (int i = 0; i < 3; i++) {
        // 1. 更新两个传感器在该轴上的实时方差
        float var_088 = RunningVariance_Update_Internal(&rv_088[i], data_088[i]);
        float var_270 = RunningVariance_Update_Internal(&rv_270[i], data_270[i]);

        // 2. 如果传感器被怀疑故障，人为增大其方差（降低权重）
        if (arb->bmi088_suspected) var_088 *= penalty_factor;
        if (arb->bmi270_suspected) var_270 *= penalty_factor;

        // 3. 最优加权（最小方差估计）
        // w1 = σ2² / (σ1² + σ2²), w2 = σ1² / (σ1² + σ2²)
        float sum_var = var_088 + var_270;
        float w_088 = var_270 / sum_var;  // 方差小的权重反而大
        float w_270 = var_088 / sum_var;

        // 4. 加权融合
        output[i] = w_088 * data_088[i] + w_270 * data_270[i];
    }

    // 5. 保存本次融合结果，供下次交叉校验做历史参考
    for (int i = 0; i < 3; i++) {
        arb->prev_fused[i] = output[i];
    }
}