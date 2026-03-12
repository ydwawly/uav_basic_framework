//
// Created by Administrator on 2026/2/20.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_4_QMC5883L_H
#define UAV_BAICE_FRAMEWORK_V1_4_QMC5883L_H

#include "main.h"
#include <stdbool.h>

// QMC5883L I2C 8位地址 (0x0D << 1)
#define QMC5883L_ADDR      0x1A
#define QMC_DATA_START_REG 0x00 // 数据起始寄存器 (X_LSB)
#define QMC_CTRL_REG1      0x09 // 控制寄存器1
#define QMC_SET_RESET_REG  0x0B // 周期设置寄存器

// 解析后的磁力计数据结构体
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MagData_t;

// // 必须使用 extern 声明，在 .c 文件中定义
// MagData_t mag_data;

void QMC5883L_Init(void);
void QMC5883L_Read_DMA_Start(void);
void QMC5883L_DMA_Callback(void);

#endif //UAV_BAICE_FRAMEWORK_V1_4_QMC5883L_H