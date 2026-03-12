//
// Created by Administrator on 2026/2/20.
//

#include "QMC5883L.h"

#include "bsp_log.h"
#include "i2c.h"
#include "message_center.h"
// 在这里真正定义全局变量
static MagData_t mag_data;
static Publisher_t *qmc_data_pub;

// 原始数据缓冲区 (6个字节对应 X, Y, Z 的 LSB 和 MSB)
__attribute__((section(".dma_buffer"), aligned(32))) static uint8_t qmc_dma_buffer[32];

void QMC5883L_Init(void) {
    uint8_t config[1];

    // 1. 设置配置寄存器 0x09
    // 模式: 连续测量 (0x01)
    // 输出速率: 200Hz (0x0C)
    // 量程: 8G (0x00)
    // 过采样率: 512 (0x00)
    // 组合计算: 0x01 | 0x0C | 0x00 | 0x00 = 0x0D
    config[0] = 0x0D;
    HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDR, QMC_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, config, 1, 100);

    // 2. 设置周期寄存器 0x0B (官方手册推荐设置为 0x01)
    config[0] = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDR, QMC_SET_RESET_REG, I2C_MEMADD_SIZE_8BIT, config, 1, 100);

    LOGINFO("[QMC5883L] QMC5883L Init Success ！");

    qmc_data_pub = PubRegister("qmc_data",sizeof(MagData_t));
}

void QMC5883L_Read_DMA_Start(void) {
    // 作为底层驱动，只保留 HAL 库硬件状态的最后一道防线
    if (hi2c2.State != HAL_I2C_STATE_READY) return;

    // 启动 DMA 读取 6 字节数据
    HAL_I2C_Mem_Read_DMA(&hi2c2, QMC5883L_ADDR, QMC_DATA_START_REG,
                         I2C_MEMADD_SIZE_8BIT, qmc_dma_buffer, 6);
}

void QMC5883L_DMA_Callback(void) {
    // 1. 数据解析 (小端模式：LSB 在前，MSB 在后)
    mag_data.x = (int16_t)(qmc_dma_buffer[1] << 8 | qmc_dma_buffer[0]);
    mag_data.y = (int16_t)(qmc_dma_buffer[3] << 8 | qmc_dma_buffer[2]);
    mag_data.z = (int16_t)(qmc_dma_buffer[5] << 8 | qmc_dma_buffer[4]);
    PubPushFromPool(qmc_data_pub, &mag_data);
}