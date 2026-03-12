//
// Created by Administrator on 2026/1/20.
//

#include "ins_task.h"
#include "mahony.h"
#include "bsp_dwt.h"
#include "BMI088driver.h"
#include "bmi2.h"
#include "BMI270driver.h"

#include "cmsis_os.h"
#include "message_center.h"
#include "madgwick.h"
#include "quaternionEKF.h"
#include "spi.h"
#include "bmi270user.h"
#include "bmi2_defs.h"
#include "PT1_Filter.h"
#include "user_math.h"
#include "height_kf.h"
// 假设包含定义了 UAV_Altitude_Data_t 的头文件
#include "SPL06.h"

static Subscriber_t *bmi088_data_sub;
static Publisher_t *imu_data_pub;
static IMU_EKF_Handle_t IMU_EKF_handle;
static BMI088_Data_t bmi088_recv_data;
// 新增：气压计订阅与高度滤波句柄
static Subscriber_t *spl06_data_sub;
static UAV_Altitude_Data_t spl06_recv_data;
static Height_KF height_kf_handle;
// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static IMU_Data_t INS_data;
static float Q = 0.001f,R = 1.0f;
static PT1_Filter_t BIM088_Gyro_Filter[3];
static PT1_Filter_t BIM088_Accel_Filter[3]; // 专门给 Accel 用

// 预定义绝对系的基向量
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
// 定义绝对系重力加速度向量 (注意：如果你的坐标系是前-右-下，重力方向是 [0, 0, 9.81])
const float gravity_n[3] = {0.0f, 0.0f, 9.81f};


static void InitQuaternion(float *init_q4)
{
    float acc[3] = {0};
    // 1. 多次采样取平均
    for (int i = 0; i < 100; i++) {
        if (SubCopyMessage(bmi088_data_sub, (void *)&bmi088_recv_data, 2) == 1) {
            acc[IMU_X] += bmi088_recv_data.Accel[IMU_X];
            acc[IMU_Y] += bmi088_recv_data.Accel[IMU_Y];
            acc[IMU_Z] += bmi088_recv_data.Accel[IMU_Z];
            DWT_Delay(0.001);
        }
    }
    acc[IMU_X] /= 100.0f;
    acc[IMU_Y] /= 100.0f;
    acc[IMU_Z] /= 100.0f;

    // 2. 直接从加速度计算 Roll 和 Pitch (Yaw = 0)
    //    FRD 坐标系: 前x, 右y, 下z
    float roll  = atan2f(acc[IMU_Y], acc[IMU_Z]);
    float pitch = atan2f(-acc[IMU_X],
                         sqrtf(acc[IMU_Y]*acc[IMU_Y] + acc[IMU_Z]*acc[IMU_Z]));
    float yaw   = 0.0f;

    // 3. 欧拉角(ZYX) → 四元数
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);

    init_q4[0] = cr * cp * cy + sr * sp * sy;
    init_q4[1] = sr * cp * cy - cr * sp * sy;
    init_q4[2] = cr * sp * cy + sr * cp * sy;
    init_q4[3] = cr * cp * sy - sr * sp * cy;
}


void INS_Init(void)
{
    bmi088_data_sub = SubRegister("bmi088_data", sizeof(BMI088_Data_t));
    spl06_data_sub = SubRegister("spl06_data", sizeof(UAV_Altitude_Data_t));
    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_EKF_Init(&IMU_EKF_handle,init_quaternion ,Q,R);
    Height_KF_Init(&height_kf_handle, 2.0f, 1e-5f, 0.1f);
    DWT_GetDeltaT(&INS_DWT_Count);
    PT1_Filter_Init(&BIM088_Gyro_Filter[0], 90.0f);
    PT1_Filter_Init(&BIM088_Gyro_Filter[1], 90.0f);
    PT1_Filter_Init(&BIM088_Gyro_Filter[2], 90.0f);
    // 10Hz - 20Hz 都是合理范围，推荐先用 15Hz
    PT1_Filter_Init(&BIM088_Accel_Filter[0], 15.0f);
    PT1_Filter_Init(&BIM088_Accel_Filter[1], 15.0f);
    PT1_Filter_Init(&BIM088_Accel_Filter[2], 15.0f);
    imu_data_pub = PubRegister("imu_data",sizeof(IMU_Data_t));
}
/* 注意以1kHz的频率运行此任务 */
void INS_Task(void)
{
    /* 1. 读取原始数据 (Raw Data)
     * 数据会存入 sensor_data.acc.x/y/z 和 sensor_data.gyr.x/y/z
     */

    if (SubCopyMessage(bmi088_data_sub, (void *)&bmi088_recv_data, 2) == 1)
    {
        dt = DWT_GetDeltaT(&INS_DWT_Count);
        t += dt;
        INS_data.Gyro[IMU_X] = PT1_Filter_Apply(&BIM088_Gyro_Filter[0], bmi088_recv_data.Gyro[IMU_X], dt);
        INS_data.Gyro[IMU_Y] = PT1_Filter_Apply(&BIM088_Gyro_Filter[1], bmi088_recv_data.Gyro[IMU_Y], dt);
        INS_data.Gyro[IMU_Z] = PT1_Filter_Apply(&BIM088_Gyro_Filter[2], bmi088_recv_data.Gyro[IMU_Z], dt);
        INS_data.Accel[IMU_X] = PT1_Filter_Apply(&BIM088_Accel_Filter[0], bmi088_recv_data.Accel[IMU_X], dt);
        INS_data.Accel[IMU_Y] = PT1_Filter_Apply(&BIM088_Accel_Filter[1], bmi088_recv_data.Accel[IMU_Y], dt);
        INS_data.Accel[IMU_Z] = PT1_Filter_Apply(&BIM088_Accel_Filter[2], bmi088_recv_data.Accel[IMU_Z], dt);

        IMU_EKF_Update(&IMU_EKF_handle, INS_data.Gyro[IMU_X], INS_data.Gyro[IMU_Y], INS_data.Gyro[IMU_Z],
                     INS_data.Accel[IMU_X], INS_data.Accel[IMU_Y], INS_data.Accel[IMU_Z], dt);


        float *q = IMU_EKF_handle.q;

        // 2. 将机体轴映射到绝对系 (可视化或进一步计算用)
        BodyFrameToEarthFrame(xb, INS_data.xn, q);
        BodyFrameToEarthFrame(yb, INS_data.yn, q);
        BodyFrameToEarthFrame(zb, INS_data.zn, q);

        // 3. 计算纯运动加速度 (优化：直接将机体系加速度转到绝对系，然后减去重力)
        float accel_earth[3];
        BodyFrameToEarthFrame(INS_data.Accel, accel_earth, q);

        INS_data.MotionAccel_n[IMU_X] = accel_earth[IMU_X] - gravity_n[IMU_X];
        INS_data.MotionAccel_n[IMU_Y] = accel_earth[IMU_Y] - gravity_n[IMU_Y];
        INS_data.MotionAccel_n[IMU_Z] = accel_earth[IMU_Z] - gravity_n[IMU_Z];


        EKF_GetEulerAngle(&IMU_EKF_handle, &INS_data.Roll, &INS_data.Pitch, &INS_data.Yaw);

        /* ========================================================== */
        /* ================= 高度数据融合 (Height EKF) ================ */
        /* ========================================================== */

        // 零阻塞尝试获取气压计新数据。返回 1 说明气压计更新了，返回 0 说明仍是旧数据
        uint8_t has_new_baro = SubCopyMessage(spl06_data_sub, (void *)&spl06_recv_data, 0);
        float acc_z_up = INS_data.MotionAccel_n[IMU_Z];

        // 只有气压计完成地面零点校准锁定后，才启动高度融合
        if (spl06_recv_data.is_calibrated) {

            // 提取 Z 轴向上的加速度分量 (FRD 坐标系 Z 轴朝下，故取反)

            if (has_new_baro == 1) {
                Height_KF_Update(&height_kf_handle, &acc_z_up, &spl06_recv_data.relative_alt, dt);
            } else {
                Height_KF_Update(&height_kf_handle, &acc_z_up, NULL, dt);
            }
        } else {
            // 校准未完成，仅做预测，积累零偏估计
            Height_KF_Update(&height_kf_handle, &acc_z_up, NULL, dt);
        }

        PubPushFromPool(imu_data_pub,&INS_data);
    }
}
