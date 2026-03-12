//
// Created by Administrator on 2026/2/20.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_5_SENSORHUB_H
#define UAV_BAICE_FRAMEWORK_V1_5_SENSORHUB_H
#include "FreeRTOS.h"
#include "task.h"
#include "message_center.h"

/* ---- 任务通知位定义：每个传感器占一个 bit ---- */
#define NOTIFY_BIT_BMI088   (1 << 0)   // bit0: BMI088 DMA 完成
#define NOTIFY_BIT_BMI270   (1 << 1)   // bit1: BMI270 DMA 完成
#define NOTIFY_BIT_SPL06    (1 << 2)   // bit2: SPL06  DMA 完成
#define NOTIFY_BIT_QMC5883  (1 << 3)   // bit3: QMC5883L DMA 完成
#define NOTIFY_BIT_SPL06_TRIG (1 << 4)   // bit4: SPL06 触发中断（准备读取）
#define NOTIFY_BIT_I2C_DMA_DONE (1 << 5)   // bit5: I2C DMA 完成（SPL06 或 QMC5883L）
#define NOTIFY_BIT_REMOTE (1 << 6)
#define NOTIFY_BIT_TFMINI (1 << 7)
#define DEVICE_SPL06 1
#define DEVICE_QMC 2

/* ---- 全局任务句柄 ---- */
extern TaskHandle_t SensorHub_Task_Handle;
void SensorHub_Init(void);
void SensorHub_Task(void *pvParameters);
#endif //UAV_BAICE_FRAMEWORK_V1_5_SENSORHUB_H