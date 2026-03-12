//
// Created by Administrator on 2026/2/20.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_4_SD_CARD_H
#define UAV_BAICE_FRAMEWORK_V1_4_SD_CARD_H

#include "stdint.h"
#include "stdbool.h"

#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp; // 时间戳
    float roll;         // 姿态
    float pitch;
    float yaw;
    int16_t gyro_x;     // 陀螺仪原始数据
    int16_t gyro_y;
    int16_t gyro_z;
    uint16_t throttle;  // 油门
} BlackboxData_t;
#pragma pack(pop)

/* 初始化 SD 卡并打开/创建日志文件 (追加模式) */
bool SD_Log_Init(void);

/* 格式化写入字符串 (类似于 printf，常用于记录 CSV 数据) */
bool SD_Log_Printf(const char* format, ...);

/* 写入纯二进制数据 (常用于高速记录自定义结构体) */
bool SD_Log_Write(const void* data, uint32_t len);

/* 同步数据到物理 SD 卡 (极其重要！防止炸机断电导致数据丢失) */
void SD_Log_Sync(void);

/* 关闭日志文件并卸载文件系统 */
void SD_Log_Close(void);

void UAV_DataLog_Task(void *argument);
#endif //UAV_BAICE_FRAMEWORK_V1_4_SD_CARD_H