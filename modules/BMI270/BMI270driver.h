//
// Created by Administrator on 2026/1/30.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_3_BMI270DRIVER_H
#define UAV_BAICE_FRAMEWORK_V1_3_BMI270DRIVER_H

#include "FreeRTOS.h"
#include "ins_task.h"
#include "task.h"
#include "main.h"

// 定义数据结构 (模仿 BMI088_Data_t)
typedef struct {
    float Accel[3];      // 单位: g
    float Gyro[3];       // 单位: rad/s (注意：为了适配EKF，底层直接转弧度)
    float GyroDeg[3];    // 单位: dps (保留一份角度制用于调试)
    float GyroOffset[3]; // 零偏
    float AccelScale;    // 加速度比例系数
    float gNorm;         // 重力模长
    float Temp;          // 温度
    uint8_t healthy;     // 健康标志位
} BMI270_Data_t;

extern volatile uint8_t bmi270_init_done; // 新增：初始化完成标志


// 外部调用函数
uint8_t BMI270_Driver_Init(SPI_HandleTypeDef *hspi);
void BMI270_Read_DMA_Start(void);
void BMI270_DMA_Callback(void);
void BMI270_DMA_ISR_Handler(BaseType_t *HigherPriorityTaskWoken);
void BMI270_Get_RawData(BMI270_Data_t *out_data);
// CS引脚宏 (根据你的原理图修改)
#define BMI270_CS_L() HAL_GPIO_WritePin(CS_BMI270_GPIO_Port, CS_BMI270_Pin, GPIO_PIN_RESET)
#define BMI270_CS_H() HAL_GPIO_WritePin(CS_BMI270_GPIO_Port, CS_BMI270_Pin, GPIO_PIN_SET)

#endif //UAV_BAICE_FRAMEWORK_V1_3_BMI270DRIVER_H