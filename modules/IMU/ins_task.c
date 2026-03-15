//
// Created by Administrator on 2026/1/20.
//

#include "ins_task.h"
#include "mahony.h"
#include "bsp_dwt.h"
#include "BMI088driver.h"
#include "BMI270driver.h"

#include "message_center.h"
#include "madgwick.h"
#include "quaternionEKF.h"
#include "PT1_Filter.h"
#include "user_math.h"
#include "height_kf.h"
#include "bsp_log.h"
#include "SPL06.h"
#include "TFmini_Plus.h"

static Publisher_t *imu_data_pub;
static IMU_EKF_Handle_t IMU_EKF_handle;

// 气压计与雷达订阅句柄
static Subscriber_t *spl06_data_sub;
static UAV_Altitude_Data_t spl06_recv_data;
static TFminiPlus_Data_t tfmini_recv_data;
static Subscriber_t *tfmini_data_sub = NULL;

// 高度融合相关变量
static Height_KF height_kf_handle;
static uint8_t height_kf_divider = 0;
static float height_dt_accum = 0.0f;
static float acc_z_accum = 0.0f;
static uint8_t acc_z_count = 0;
float process_noise_accel = 2.0f, measure_noise_height = 0.1f, process_noise_bias = 1e-5f;
float measure_noise_baro  = 2.0f;
float measure_noise_lidar = 0.05f;

// 姿态解算相关变量
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static IMU_Data_t INS_data;
static float Q = 0.001f, R = 1.0f;

// 滤波器声明
static PT1_Filter_t BMI088_Gyro_Filter[3];
static PT1_Filter_t BMI088_Accel_Filter[3];

// 预定义绝对系的基向量 (前-右-下 坐标系 FRD)
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
const float gravity_n[3] = {0.0f, 0.0f, 9.81f};

// 双IMU方差估计器（各3轴）
static RunningVariance_t var_gyro_088[3], var_gyro_270[3];
static RunningVariance_t var_accel_088[3], var_accel_270[3];

// 仲裁器实例
static SensorArbiter_t gyro_arbiter = {
    .diverge_count = 0,
    .diverge_threshold = DIVERGE_CONFIRM_COUNT,
    .bmi088_suspected = 0,
    .bmi270_suspected = 0,
    .prev_fused = {0, 0, 0}
};

static SensorArbiter_t accel_arbiter = {
    .diverge_count = 0,
    .diverge_threshold = DIVERGE_CONFIRM_COUNT,
    .bmi088_suspected = 0,
    .bmi270_suspected = 0,
    .prev_fused = {0, 0, 0}
};




// ======================================================================
// 初始四元数对齐 (支持双 IMU 容错)
// ======================================================================
static void InitQuaternion(float *init_q4)
{
    float acc[3] = {0};
    BMI088_Data_t raw_bmi088;
    BMI270_Data_t raw_bmi270;
    float fused_accel[3] = {0};

    LOGINFO("[INS] Start aligning initial quaternion...");

    // 1. 多次采样取平均 (利用 DWT 阻塞延时，采集 100 帧最新数据)
    for (int i = 0; i < 100; i++)
    {
        // 瞬间拉取双缓冲区的最新数据
        BMI088_Get_RawData(&raw_bmi088);
        BMI270_Get_RawData(&raw_bmi270);

        // 冗余融合逻辑 (Sensor Blending)
        if (raw_bmi088.healthy && raw_bmi270.healthy) {
            for(int j=0; j<3; j++) fused_accel[j] = (raw_bmi088.Accel[j] + raw_bmi270.Accel[j]) * 0.5f;
        } else if (raw_bmi088.healthy) {
            for(int j=0; j<3; j++) fused_accel[j] = raw_bmi088.Accel[j];
        } else if (raw_bmi270.healthy) {
            for(int j=0; j<3; j++) fused_accel[j] = raw_bmi270.Accel[j];
        } else {
            // 如果上电时两个都坏了，直接死循环报警，绝不允许起飞！
            LOGERROR("[INS] FATAL: Both IMUs failed during initialization!");
            while(1);
        }

        acc[IMU_X] += fused_accel[IMU_X];
        acc[IMU_Y] += fused_accel[IMU_Y];
        acc[IMU_Z] += fused_accel[IMU_Z];

        DWT_Delay(0.001); // 严格等待 1ms，确保下一帧 DMA 已经更新了缓冲区
    }

    acc[IMU_X] /= 100.0f;
    acc[IMU_Y] /= 100.0f;
    acc[IMU_Z] /= 100.0f;

    // 2. 直接从加速度计算 Roll 和 Pitch (假设初始 Yaw = 0)
    float roll  = atan2f(acc[IMU_Y], acc[IMU_Z]);
    float pitch = atan2f(-acc[IMU_X], sqrtf(acc[IMU_Y]*acc[IMU_Y] + acc[IMU_Z]*acc[IMU_Z]));
    float yaw   = 0.0f;

    // 3. 欧拉角(ZYX) → 四元数
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);

    init_q4[0] = cr * cp * cy + sr * sp * sy;
    init_q4[1] = sr * cp * cy - cr * sp * sy;
    init_q4[2] = cr * sp * cy + sr * cp * sy;
    init_q4[3] = cr * cp * sy - sr * sp * cy;

    // LOGINFO("[INS] Init Quaternion: w=%.2f x=%.2f y=%.2f z=%.2f", init_q4[0], init_q4[1], init_q4[2], init_q4[3]);
}

// ======================================================================
// 惯性导航系统初始化
// ======================================================================
void INS_Init(void)
{
    spl06_data_sub  = SubRegister("spl06_data", sizeof(UAV_Altitude_Data_t));
    tfmini_data_sub = SubRegister("tfmini_data", sizeof(TFminiPlus_Data_t));

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);

    IMU_EKF_Init(&IMU_EKF_handle, init_quaternion, Q, R);
    Height_KF_Init(&height_kf_handle, process_noise_accel, 1e-5f, measure_noise_lidar);

    PT1_Filter_Init(&BMI088_Gyro_Filter[0], 90.0f);
    PT1_Filter_Init(&BMI088_Gyro_Filter[1], 90.0f);
    PT1_Filter_Init(&BMI088_Gyro_Filter[2], 90.0f);

    PT1_Filter_Init(&BMI088_Accel_Filter[0], 15.0f);
    PT1_Filter_Init(&BMI088_Accel_Filter[1], 15.0f);
    PT1_Filter_Init(&BMI088_Accel_Filter[2], 15.0f);

    imu_data_pub = PubRegister("imu_data", sizeof(IMU_Data_t));

    // 初始化方差估计器
    for (int i = 0; i < 3; i++) {
        RunningVariance_Init(&var_gyro_088[i]);
        RunningVariance_Init(&var_gyro_270[i]);
        RunningVariance_Init(&var_accel_088[i]);
        RunningVariance_Init(&var_accel_270[i]);
    }

    // 初始化仲裁器（清零 prev_fused，避免第一次校验误判）
    memset(&gyro_arbiter.prev_fused, 0, sizeof(gyro_arbiter.prev_fused));
    memset(&accel_arbiter.prev_fused, 0, sizeof(accel_arbiter.prev_fused));


    DWT_GetDeltaT(&INS_DWT_Count); // 清零积分时间
}

// ======================================================================
// 核心解算任务 (运行频率: 取决于 BMI088 中断频率，通常为 1000Hz)
// ======================================================================
void INS_Task(void *argument)
{
    uint32_t notifyBits;
    static BMI088_Data_t raw_bmi088;
    static BMI270_Data_t raw_bmi270;

    static float fused_gyro[3];
    static float fused_accel[3];

    static float ins_dt;
    static float ins_start;
    // 错误记录，防止日志刷屏
    static uint8_t bmi270_fail_logged = 0;
    INS_Init();
    while (1)
    {
        // 1. 深度挂起任务，等待主 IMU (BMI088) 的中断触发
        BaseType_t res = xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifyBits, pdMS_TO_TICKS(5));

        if (res == pdTRUE)
        {
            if (notifyBits & NOTIFY_BIT_IMU_READY)
            {
                ins_start = DWT_GetTimeline_ms();

                // 2. 瞬间拉取双份原始数据
                BMI088_Get_RawData(&raw_bmi088);
                BMI270_Get_RawData(&raw_bmi270);

                // 3. 计算极其精准的 dt
                dt = DWT_GetDeltaT(&INS_DWT_Count);
                t += dt;

                /* ========================================================== */
                /* ================= 冗余仲裁与数据融合 ======================= */
                /* ========================================================== */
                /* ========== 改进后的融合逻辑 ========== */
                if (raw_bmi088.healthy && raw_bmi270.healthy) {
                    // 1. 交叉校验（检测硬故障/突变）
                    CrossValidate_Gyro(raw_bmi088.Gyro, raw_bmi270.Gyro, &gyro_arbiter);
                    CrossValidate_Gyro(raw_bmi088.Accel, raw_bmi270.Accel, &accel_arbiter);

                    // 2. 动态加权融合（软故障自动降权）
                    FuseWithDynamicWeight(raw_bmi088.Gyro, raw_bmi270.Gyro,
                                          var_gyro_088, var_gyro_270,
                                          fused_gyro, &gyro_arbiter, 10.0f);

                    FuseWithDynamicWeight(raw_bmi088.Accel, raw_bmi270.Accel,
                                          var_accel_088, var_accel_270,
                                          fused_accel, &accel_arbiter, 10.0f);

                    // 日志节流：只在状态变化时打印
                    static uint8_t last_088_suspect = 0, last_270_suspect = 0;
                    if (gyro_arbiter.bmi088_suspected != last_088_suspect) {
                        if (gyro_arbiter.bmi088_suspected)
                            LOGWARNING("[INS] BMI088 suspected! Variance penalty applied.");
                        last_088_suspect = gyro_arbiter.bmi088_suspected;
                    }
                    if (gyro_arbiter.bmi270_suspected != last_270_suspect) {
                        if (gyro_arbiter.bmi270_suspected)
                            LOGWARNING("[INS] BMI270 suspected! Variance penalty applied.");
                        last_270_suspect = gyro_arbiter.bmi270_suspected;
                    }
                }
                else if (raw_bmi088.healthy && !raw_bmi270.healthy) {
                    // 副传感器彻底失效（硬件标志位异常）
                    for (int i = 0; i < 3; i++) {
                        fused_gyro[i] = raw_bmi088.Gyro[i];
                        fused_accel[i] = raw_bmi088.Accel[i];
                    }
                    if (!bmi270_fail_logged) {
                        LOGWARNING("[INS] BMI270 Dead! Using BMI088 only.");
                        bmi270_fail_logged = 1;
                    }
                    // 重置仲裁器，避免恢复时误判
                    gyro_arbiter.diverge_count = 0;
                    accel_arbiter.diverge_count = 0;
                }
                else if (!raw_bmi088.healthy && raw_bmi270.healthy) {
                    // 主传感器失效，副接管
                    for (int i = 0; i < 3; i++) {
                        fused_gyro[i] = raw_bmi270.Gyro[i];
                        fused_accel[i] = raw_bmi270.Accel[i];
                    }
                    LOGERROR("[INS] CRITICAL: BMI088 Dead! Switched to BMI270!");
                }
                else {
                    LOGERROR("[INS] FATAL: BOTH IMUs DEAD!");
                    continue;
                }

                /* ========================================================== */
                /* ================= 信号滤波与姿态解算 ======================= */
                /* ========================================================== */
                // 将融合后的纯净信号送入一阶低通滤波器
                INS_data.Gyro[IMU_X] = PT1_Filter_Apply(&BMI088_Gyro_Filter[0], fused_gyro[IMU_X], dt);
                INS_data.Gyro[IMU_Y] = PT1_Filter_Apply(&BMI088_Gyro_Filter[1], fused_gyro[IMU_Y], dt);
                INS_data.Gyro[IMU_Z] = PT1_Filter_Apply(&BMI088_Gyro_Filter[2], fused_gyro[IMU_Z], dt);

                INS_data.Accel[IMU_X] = PT1_Filter_Apply(&BMI088_Accel_Filter[0], fused_accel[IMU_X], dt);
                INS_data.Accel[IMU_Y] = PT1_Filter_Apply(&BMI088_Accel_Filter[1], fused_accel[IMU_Y], dt);
                INS_data.Accel[IMU_Z] = PT1_Filter_Apply(&BMI088_Accel_Filter[2], fused_accel[IMU_Z], dt);

                // 更新 EKF
                IMU_EKF_Update(&IMU_EKF_handle, INS_data.Gyro[IMU_X], INS_data.Gyro[IMU_Y], INS_data.Gyro[IMU_Z],
                               INS_data.Accel[IMU_X], INS_data.Accel[IMU_Y], INS_data.Accel[IMU_Z], dt);

                float *q = IMU_EKF_handle.q;

                // 将机体轴映射到绝对系
                BodyFrameToEarthFrame(xb, INS_data.xn, q);
                BodyFrameToEarthFrame(yb, INS_data.yn, q);
                BodyFrameToEarthFrame(zb, INS_data.zn, q);

                // 计算纯运动加速度 (扣除重力)
                float accel_earth[3];
                BodyFrameToEarthFrame(INS_data.Accel, accel_earth, q);

                INS_data.MotionAccel_n[IMU_X] = accel_earth[IMU_X] - gravity_n[IMU_X];
                INS_data.MotionAccel_n[IMU_Y] = accel_earth[IMU_Y] - gravity_n[IMU_Y];
                INS_data.MotionAccel_n[IMU_Z] = accel_earth[IMU_Z] - gravity_n[IMU_Z];

                // 提取欧拉角供 PID 使用
                EKF_GetEulerAngle(&IMU_EKF_handle, &INS_data.Roll, &INS_data.Pitch, &INS_data.Yaw);

                /* ========================================================== */
                /* ================= 高度数据融合 (Height EKF) ================ */
                /* ========================================================== */
                uint8_t has_new_baro   = SubCopyMessage(spl06_data_sub, (void *)&spl06_recv_data, 0);
                uint8_t has_new_tfmini = SubCopyMessage(tfmini_data_sub, (void *)&tfmini_recv_data, 0);

                float acc_z_up = INS_data.MotionAccel_n[IMU_Z]; // 注意这里 Z 轴向下的定义
                height_dt_accum += dt;
                acc_z_accum += acc_z_up;
                acc_z_count++;
                height_kf_divider++;

                // 降频执行高度融合 (例如 1000Hz 降到 200Hz)
                if (height_kf_divider >= 5)
                {
                    float avg_acc_z = acc_z_accum / (float)acc_z_count;
                    float *measure_ptr = NULL;
                    float current_r = measure_noise_baro;

                    // 优先级决策
                    if (has_new_tfmini && tfmini_recv_data.distance > 0.05 && tfmini_recv_data.distance < 8.0 && tfmini_recv_data.is_valid == 1) {
                        measure_ptr = &tfmini_recv_data.distance;
                        current_r = measure_noise_lidar;
                    }
                    else if (has_new_baro && spl06_recv_data.is_calibrated) {
                        measure_ptr = &spl06_recv_data.relative_alt;
                        current_r = measure_noise_baro;
                    }

                    Height_KF_Update(&height_kf_handle, &avg_acc_z, measure_ptr, height_dt_accum, current_r);

                    height_kf_divider = 0;
                    height_dt_accum = 0.0f;
                    acc_z_accum = 0.0f;
                    acc_z_count = 0;
                }

                // 4. 将最终最纯净、最精准的姿态和高度发送给 Control_Task
                PubPushFromPool(imu_data_pub, &INS_data);

                ins_dt = DWT_GetTimeline_ms() - ins_start;
                if (ins_dt > 0.5f) {  // 500μs（严重超时，可能影响控制）
                    LOGERROR("[INS] SEVERE overrun! dt=%.3fms", ins_dt);
                }
                else if (ins_dt > 0.2f)
                {
                    // 200μs（警告，接近极限）
                    LOGWARNING("[INS] Task overrun! dt=%.3fms", ins_dt);
                }
                // 可选：统计最大耗时
                static float max_ins_dt = 0;
                if (ins_dt > max_ins_dt) {
                    max_ins_dt = ins_dt;
                }

            }
            else if (notifyBits & NOTIFY_BIT_BMI270_READY)
            {

            }
            else
            {
                // 理论上不应该进这里，除非有其他任务误发了通知
                LOGWARNING("[INS] Unknown notify bits: 0x%08X", notifyBits);
            }
        }
        else
        {
            LOGERROR("[INS] IMU Dead! INS Task Timeout!");
        }
    }
}