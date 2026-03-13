#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "FreeRTOS.h"
#include "task.h"
#include "stdint.h"
#include "main.h"

// 灵敏度定义

#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2

#define BMI088_ACCEL_6G_SEN   0.00179443359375f
#define BMI088_GYRO_2000_SEN  0.001065264436031695f
#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)

typedef struct
{
    float Accel[3]; // m/s^2
    float Gyro[3];  // rad/s
    float GyroOffset[3]; // 新增：陀螺仪零偏
    float AccelScale;    // 新增：加速度计比例缩放
    float gNorm;         // 辅助记录重力模长
    float Temperature;
    uint8_t healthy; // 1 为健康，0 为存在故障
} BMI088_Data_t;

typedef enum {
    BMI088_OK = 0,
    BMI088_ERR_ID,      // ID错误
    BMI088_ERR_BIST,    // 硬件自检失败
    BMI088_ERR_TIMEOUT, // 通信超时
    BMI088_ERR_STUCK    // 数据卡死
} BMI088_Status_e;


extern TaskHandle_t IMU_Task_Handle;

/**
 * @brief 初始化 BMI088 (阻塞式, 仅上电运行一次)
 */
uint8_t BMI088_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief 启动 DMA 读取 (在 1kHz 定时器中调用)
 */
void BMI088_Read_DMA_Start(void);

/**
 * @brief DMA 回调 (在 HAL_SPI_TxRxCpltCallback 中调用)
 */
void BMI088_DMA_Callback(void);
void BMI088_DMA_ISR_Handler(BaseType_t *HigherPriorityTaskWoken);
void BMI088_Get_RawData(BMI088_Data_t *out_data);
#endif
