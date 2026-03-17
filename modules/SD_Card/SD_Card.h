//
// Created by Administrator on 2026/2/20.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_6_SD_CARD_H
#define UAV_BAICE_FRAMEWORK_V1_6_SD_CARD_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
// RingBuffer 大小必须是 2 的幂！
// 64KB 可缓存约 910 帧数据（72字节/帧 × 910 ≈ 65KB）
// 在 1000Hz 采样率下 = 0.91秒 的缓冲时间
#define LOG_RB_SIZE  (64 * 1024)  // 65536 bytes = 2^16
#define NOTIFY_BIT_SD_WRITE_READY (1 << 0) // 定义一个唤醒位
/* ========================================================== */
/*                    黑匣子数据结构                           */
/* ========================================================== */
// 使用 pragma pack 确保跨平台一致性
#pragma pack(push, 1)
typedef struct {
    // 时间戳（改为微秒精度，避免1kHz采样时重复）
    uint64_t timestamp_us;   // 8 bytes

    // 姿态数据（浮点，保留完整精度）
    float roll;              // 4 bytes
    float pitch;             // 4 bytes
    float yaw;               // 4 bytes

    // 陀螺仪数据（浮点，单位：rad/s）
    float gyro_x;            // 4 bytes
    float gyro_y;            // 4 bytes
    float gyro_z;            // 4 bytes

    // 加速度计数据（浮点，单位：m/s²）
    float accel_x;           // 4 bytes
    float accel_y;           // 4 bytes
    float accel_z;           // 4 bytes

    // 高度与速度
    float height;            // 4 bytes
    float velocity_z;        // 4 bytes

    // IMU 健康状态标志
    // bit0: BMI088 健康
    // bit1: BMI270 健康
    // bit2-7: 预留
    uint8_t imu_status;      // 1 byte

    // 飞行模式（可选，需要从控制任务获取）
    uint8_t flight_mode;     // 1 byte

    // 预留字段（对齐到 4 字节边界）
    uint8_t reserve[2];      // 2 bytes

} BlackboxData_t;  // 总共 60 bytes
#pragma pack(pop)



extern TaskHandle_t UAV_DataLog_Task_Handle;

/* ========================================================== */
/*                      API 接口                              */
/* ========================================================== */

/**
 * @brief  初始化 SD 卡并创建新的日志文件
 * @retval true: 成功, false: 失败
 */
bool SD_Log_Init(void);

/**
 * @brief  [生产者接口] 将数据极速推入环形缓冲区
 * @param  data: 数据指针（通常是 BlackboxData_t*）
 * @param  len:  数据长度（字节）
 * @retval true: 成功推入, false: 缓冲区满，数据被丢弃
 * @note   此函数耗时 <2μs，适合在高频任务中调用
 */
bool SD_Log_PushData(const void* data, uint32_t len);

/**
 * @brief  强制同步数据到物理 SD 卡
 * @note   由写卡任务定时调用，用户无需手动调用
 */
void SD_Log_Sync(void);

/**
 * @brief  关闭日志文件并卸载文件系统
 */
void SD_Log_Close(void);

/**
 * @brief  [消费者任务] SD 卡写入任务函数
 * @param  argument: RTOS 任务参数（未使用）
 * @note   此任务以低优先级运行，从 RingBuffer 取数据写入 SD 卡
 */
void UAV_DataLog_Task(void *argument);

/* ========================================================== */
/*                   调试接口（可选）                          */
/* ========================================================== */

/**
 * @brief  获取日志统计信息
 * @param  total_frames:   总采集帧数
 * @param  dropped_frames: 丢弃帧数
 * @param  buffer_usage:   当前缓冲区使用率（0.0 ~ 1.0）
 */
void SD_Log_GetStats(uint32_t *total_frames, uint32_t *dropped_frames, float *buffer_usage);

#endif //UAV_BAICE_FRAMEWORK_V1_6_SD_CARD_H