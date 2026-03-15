#include "SPL06.h"
#include "message_center.h" // 引入你的消息中心
#include <math.h>

#include "bsp_dwt.h"
#include "bsp_log.h"

/* --- 静态私有变量 (完全与外部隔离) --- */
static I2C_HandleTypeDef *p_hi2c_spl06;
static T_SPL06_calibPara calib_para;
static UAV_Altitude_Data_t alt_data = {0};
static Publisher_t *spl06_data_pub;  // 消息发布者指针

// 32字节对齐的 DMA 接收缓冲区
__attribute__((section(".dma_buffer"), aligned(32))) static uint8_t spl06_dma_buffer[32];

// 校准状态机变量
static uint16_t calib_count = 0;
static float sum_pressure = 0;

/* ==================== 内部静态函数 ==================== */

static void SPL06_Write_Reg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(p_hi2c_spl06, SPL06DeviceAdd, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static void SPL06_Read_Buffer(uint8_t reg, uint8_t *pData, uint16_t len) {
    HAL_I2C_Mem_Read(p_hi2c_spl06, SPL06DeviceAdd, reg, I2C_MEMADD_SIZE_8BIT, pData, len, 100);
}

// 解析 DMA 缓冲区，直接更新 alt_data 里的温度和气压
static void Process_Raw_Data(void) {
    int32_t raw_press = (int32_t)(spl06_dma_buffer[0] << 16) | (int32_t)(spl06_dma_buffer[1] << 8) | spl06_dma_buffer[2];
    raw_press = (raw_press & 0x800000) ? (0xFF000000 | raw_press) : raw_press;

    int32_t raw_temp = (int32_t)(spl06_dma_buffer[3] << 16) | (int32_t)(spl06_dma_buffer[4] << 8) | spl06_dma_buffer[5];
    raw_temp = (raw_temp & 0x800000) ? (0xFF000000 | raw_temp) : raw_temp;

    float Traw_src = (float)raw_temp / calib_para.kT;
    float Praw_src = (float)raw_press / calib_para.kP;

    // 1. 算出了真实的传感器温度 (℃)
    alt_data.temperature = 0.5f * calib_para.C0 + Traw_src * calib_para.C1;

    // 2. 算出了真实的气压 (Pa)
    float qua2 = calib_para.C10 + Praw_src * (calib_para.C20 + Praw_src * calib_para.C30);
    float qua3 = Traw_src * Praw_src * (calib_para.C11 + Praw_src * calib_para.C21);
    alt_data.pressure = calib_para.C00 + Praw_src * qua2 + Traw_src * calib_para.C01 + qua3;
}

// 核心：基于真实温度补偿的高精度高度解算
static void Update_Altitude(void) {
    // 绝对高度 (使用标准海平面气压公式，发给地面站看就行)
    // alt_data.absolute_alt = 44330.0f * (1.0f - powf((alt_data.pressure / STANDARD_SEA_LEVEL_PRESSURE), 0.190295f));

    if (alt_data.is_calibrated) {
        // 相对高度 (利用物理测高公式 Hypsometric Equation 引入真实的开尔文温度补偿)
        // 公式: Δh = (R * T_kelvin) / g * ln(P_ground / P_current)
        // 常数: R/g = 287.05 / 9.80665 ≈ 29.27126
        float T_kelvin = alt_data.temperature + 273.15f;

        alt_data.relative_alt = 29.27126f * T_kelvin * logf(alt_data.ground_pressure / alt_data.pressure);

        // 一阶低通滤波 (滤除高频风噪)
        float alpha = 0.1f;
        alt_data.filtered_alt = alt_data.filtered_alt * (1.0f - alpha) + alt_data.relative_alt * alpha;
    }
}

/* ==================== 对外公开 API ==================== */

/**
 * @brief 初始化 SPL06 传感器并注册消息发布者
 */
void SPL06_Init_DMA(I2C_HandleTypeDef *hi2c) {
    p_hi2c_spl06 = hi2c;
    uint8_t coef[18];

    // 1. 软复位 (清除可能残留的配置和中断)
    uint8_t reset_cmd = 0x09;
    HAL_I2C_Mem_Write(p_hi2c_spl06, SPL06DeviceAdd, SPL06_RESET, I2C_MEMADD_SIZE_8BIT, &reset_cmd, 1, 100);
    DWT_Delay(0.05f);
    // HAL_Delay(50);

    // 2. 读取出厂校准参数
    SPL06_Read_Buffer(SPL06_COEF, coef, 18);
    calib_para.C0 = ((int16_t)coef[0] << 4) | (coef[1] >> 4);
    calib_para.C0 = (calib_para.C0 & 0x0800) ? (0xF000 | calib_para.C0) : calib_para.C0;
    calib_para.C1 = ((int16_t)(coef[1] & 0x0F) << 8) | coef[2];
    calib_para.C1 = (calib_para.C1 & 0x0800) ? (0xF000 | calib_para.C1) : calib_para.C1;
    calib_para.C00 = ((int32_t)coef[3] << 12) | ((int32_t)coef[4] << 4) | (coef[5] >> 4);
    calib_para.C00 = (calib_para.C00 & 0x080000) ? (0xFFF00000 | calib_para.C00) : calib_para.C00;
    calib_para.C10 = ((int32_t)(coef[5] & 0x0F) << 16) | ((int32_t)coef[6] << 8) | coef[7];
    calib_para.C10 = (calib_para.C10 & 0x080000) ? (0xFFF00000 | calib_para.C10) : calib_para.C10;
    calib_para.C01 = ((int16_t)coef[8] << 8) | coef[9];
    calib_para.C11 = ((int16_t)coef[10] << 8) | coef[11];
    calib_para.C20 = ((int16_t)coef[12] << 8) | coef[13];
    calib_para.C21 = ((int16_t)coef[14] << 8) | coef[15];
    calib_para.C30 = ((int16_t)coef[16] << 8) | coef[17];

    // calib_para.kP = 516096.0f;
    calib_para.kP = 7864320.0f;  // 8x 过采样对应的 kP（原来32x对应516096）
    calib_para.kT = 7864320.0f;

    // 3. 配置工作模式 (高速高精度连续采样 + 硬件中断开启)
    // SPL06_Write_Reg(SPL06_PSR_CFG, PM_RATE_4 | PM_PRC_32);
    // SPL06_Write_Reg(SPL06_TMP_CFG, TMP_RATE_4 | TMP_PRC_8 | 0x80);
    SPL06_Write_Reg(SPL06_PSR_CFG, PM_RATE_16 | PM_PRC_8);       // 16Hz, 8x
    SPL06_Write_Reg(SPL06_TMP_CFG, TMP_RATE_16 | TMP_PRC_8 | 0x80); // 16Hz, 8x

    SPL06_Write_Reg(SPL06_CFG_REG, 0x80 | 0x20 | 0x10 | SPL06_CFG_P_SHIFT);
    SPL06_Write_Reg(SPL06_MEAS_CFG, MEAS_CTRL_ContinuousPressTemp);

    LOGINFO("[SPL06] SPL06 Init Success !");

    // 4. 将自己注册进飞控的消息中心
    spl06_data_pub = PubRegister("spl06_data", sizeof(UAV_Altitude_Data_t));
}

/**
 * @brief 触发 I2C DMA 异步读取
 */
void SPL06_Read_DMA_Start(void) {
    if (p_hi2c_spl06->State != HAL_I2C_STATE_READY) return;

    // 读取 11 个字节 (涵盖压力、温度原始值及中断状态寄存器)
    HAL_I2C_Mem_Read_DMA(p_hi2c_spl06, SPL06DeviceAdd, SPL06_PSR_B2,
                         I2C_MEMADD_SIZE_8BIT, spl06_dma_buffer, 11);
}

/**
 * @brief 数据解析、校准与发布流水线 (在 SensorHub 任务中被调用)
 */
void SPL06_Data_Handler(void) {
    // 1. 解析原始数据，存入 alt_data 结构体
    Process_Raw_Data();

    // 2. 飞控开机地面零点锁定状态机
    if (!alt_data.is_calibrated) {
        if (calib_count < 10) {
            calib_count++; // 丢弃前10帧脏数据
        } else if (calib_count < 60) {
            sum_pressure += alt_data.pressure; // 累加50帧稳定气压
            calib_count++;
        } else {
            alt_data.ground_pressure = sum_pressure / 50.0f; // 锁定基准面
            alt_data.filtered_alt = 0.0f;
            alt_data.is_calibrated = 1; // 解锁高度计算
        }
    }
    // 3. 正常工作模式 (高度解算)
    else {
        Update_Altitude();
    }

    // 4. 将算好的优质数据推送到飞控全局消息池
    if (spl06_data_pub != NULL) {
        PubPushFromPool(spl06_data_pub, &alt_data);
    }
}

/**
 * @brief 强制重置气压计地面零点
 * @note  飞控应用层应该在"无人机解锁(Arming)"瞬间调用此函数
 */
void SPL06_Reset_Zero_Datum(void) {
    // 将标志位置 0，让数据处理状态机重新采集接下来的 50 帧作为新基准
    alt_data.is_calibrated = 0;
    calib_count = 0;
    sum_pressure = 0.0f;
    alt_data.filtered_alt = 0.0f;
}