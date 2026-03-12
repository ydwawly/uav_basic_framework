#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include <math.h>
#include <string.h>
#include "message_center.h"
#include "PT1_Filter.h"

static BMI088_Data_t bmi088_data;
static Publisher_t *bmi088_data_pub;
SPI_HandleTypeDef *bmi_spi;

// DMA缓冲区
// static uint8_t gyro_tx[7], gyro_rx[7];// Gyro: Reg(1) + Data(6)
// static uint8_t accel_tx[8], accel_rx[8];// Accel: Reg(1) + Dummy(1) + Data(6)
// DMA缓冲区 (强制放入无 Cache 区，并按 32 字节对齐防止踩踏)
__attribute__((section(".dma_buffer"), aligned(32))) static uint8_t gyro_tx[32];
__attribute__((section(".dma_buffer"), aligned(32))) static uint8_t gyro_rx[32];
__attribute__((section(".dma_buffer"), aligned(32))) static uint8_t accel_tx[32];
__attribute__((section(".dma_buffer"), aligned(32))) static uint8_t accel_rx[32];
// 状态机: 0=Idle, 1=Reading_Gyro, 2=Reading_Accel
volatile uint8_t bmi088_dma_state = 0;
// 内部函数声明
static void write_reg(uint8_t reg, uint8_t data, uint8_t is_accel);
static uint8_t read_reg(uint8_t reg, uint8_t is_accel);
static void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
static void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
static void Calibrate_BMI088(BMI088_Data_t *bmi);
static uint32_t stuck_count = 0;
static float last_gyro_x = 0;

// ================= 初始化函数 =================
uint8_t BMI088_Init(SPI_HandleTypeDef *hspi)
{
    bmi_spi = hspi;
    uint8_t id = 0;

    // 1. 验证陀螺仪 ID
    id = read_reg(BMI088_GYRO_CHIP_ID, 0);
    if (id != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_ERR_ID;

    // 2. 触发陀螺仪自检 (BIST)
    write_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST, 0);
    DWT_Delay(0.05f); // 必须等待复位完成

    uint8_t bist_res = read_reg(BMI088_GYRO_SELF_TEST, 0);
    if (!(bist_res & BMI088_GYRO_BIST_RDY) || (bist_res & BMI088_GYRO_BIST_FAIL)) {
        return BMI088_ERR_BIST; // 硬件物理损伤
    }
    // --- 1. 初始化加速度计 (Accel) ---
    // 软复位
    write_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE, 1);
    DWT_Delay(0.05f); // 必须等待复位完成

    // 开启电源 (必须先切到 Active 模式，再开启数据流)
    write_reg(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, 1);
    DWT_Delay(0.01f); // 上电延时

    write_reg(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, 1);
    DWT_Delay(0.01f);

    // 配置: 正常模式, 800Hz, OSR2 (带宽)
    write_reg(BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, 1);

    // 配置: 量程 6G
    write_reg(BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, 1);

    // --- 2. 初始化陀螺仪 (Gyro) ---
    // 软复位
    write_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE, 0);
    DWT_Delay(0.05f);

    // 配置: 量程 2000dps
    write_reg(BMI088_GYRO_RANGE, BMI088_GYRO_2000, 0);

    // 配置: 带宽 230Hz
    write_reg(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, 0);
    Calibrate_BMI088(&bmi088_data);

    // --- 3. 准备 DMA 缓冲区命令 (保持不变) ---
    // Gyro 读命令: 从 0x02 开始读 (MSB=1)
    gyro_tx[0] = BMI088_GYRO_X_L | 0x80;
    memset(&gyro_tx[1], 0xFF, 6);

    // Accel 读命令: 从 0x12 开始读 (MSB=1)
    accel_tx[0] = BMI088_ACCEL_XOUT_L | 0x80;
    memset(&accel_tx[1], 0xFF, 7);


    // ================== 新增：配置硬件中断输出 ==================
    // 1. 陀螺仪 (Gyro) DRDY 中断配置
    // 开启陀螺仪的 DRDY 中断生成
    write_reg(BMI088_GYRO_CTRL, BMI088_DRDY_ON, 0);
    // 配置 INT3 引脚为推挽输出，高电平有效 (对应原理图的 PC15 BMI088_GYRO_DR)
    write_reg(BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_HIGH, 0);
    // 将 DRDY 信号映射到 INT3 引脚
    write_reg(BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, 0);

    // 2. 加速度计 (Accel) DRDY 中断配置 (可选，虽然我们只用 Gyro 触发状态机，但配上以备后用)
    // 配置 INT1 引脚为使能、推挽输出，高电平有效 (对应原理图的 PC14 BMI088_ACCEL_DR)
    write_reg(BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_HIGH, 1);
    // 将 DRDY 信号映射到 INT1 引脚
    write_reg(BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, 1);
    // ==========================================================


    // 重置状态
    bmi088_dma_state = 0;
    bmi088_data.healthy = 1;
    bmi088_data_pub = PubRegister("bmi088_data", sizeof(BMI088_Data_t));

    LOGINFO("[BMI088] BMI088 Init Success !");
    return BMI088_OK;
}

// ================= DMA 核心逻辑 =================

void BMI088_Read_DMA_Start(void)
{
    if(bmi088_dma_state != 0) return; // 防止重入

    // 1. 开始读取 Gyro
    bmi088_dma_state = 1;
    BMI088_GYRO_NS_L();
    HAL_SPI_TransmitReceive_DMA(bmi_spi, gyro_tx, gyro_rx, 7);
}

void BMI088_DMA_Callback(void)
{
    static int16_t raw;

    if (bmi088_dma_state == 1) // Gyro 完成
    {
        BMI088_GYRO_NS_H(); // 拉高 Gyro CS

        // 2. 紧接着读取 Accel
        bmi088_dma_state = 2;
        BMI088_ACCEL_NS_L();
        HAL_SPI_TransmitReceive_DMA(bmi_spi, accel_tx, accel_rx, 8);

        // 处理 Gyro 数据 (Rx[0]是垃圾数据, 数据从Rx[1]开始)
        // 使用宏定义的离线 Offset 进行修正
        // 处理并修正 Gyro 偏置
        raw = (int16_t)((gyro_rx[2] << 8) | gyro_rx[1]);
        bmi088_data.Gyro[IMU_Y] = -(raw * BMI088_GYRO_2000_SEN - bmi088_data.GyroOffset[IMU_Y]);

        raw = (int16_t)((gyro_rx[4] << 8) | gyro_rx[3]);
        bmi088_data.Gyro[IMU_X] = -(raw * BMI088_GYRO_2000_SEN - bmi088_data.GyroOffset[IMU_X]);

        raw = (int16_t)((gyro_rx[6] << 8) | gyro_rx[5]);
        bmi088_data.Gyro[IMU_Z] = -(raw * BMI088_GYRO_2000_SEN - bmi088_data.GyroOffset[IMU_Z]);
    }
    else if (bmi088_dma_state == 2) // Accel 完成
    {
        BMI088_ACCEL_NS_H(); // 拉高 Accel CS
        bmi088_dma_state = 0; // 结束

        // 处理并修正 Accel Scale (注意对应 X/Y/Z)
        raw = (int16_t)((accel_rx[3] << 8) | accel_rx[2]);
        bmi088_data.Accel[IMU_Y] = (raw * BMI088_ACCEL_6G_SEN) * bmi088_data.AccelScale;

        raw = (int16_t)((accel_rx[5] << 8) | accel_rx[4]);
        bmi088_data.Accel[IMU_X] = (raw * BMI088_ACCEL_6G_SEN) * bmi088_data.AccelScale;

        raw = (int16_t)((accel_rx[7] << 8) | accel_rx[6]);
        bmi088_data.Accel[IMU_Z] = (raw * BMI088_ACCEL_6G_SEN) * bmi088_data.AccelScale;

        // 可选：在此处释放信号量给解算任务
    }
    // 如果角速度连续 50 次完全不变化，判定为传感器失效
    if (fabs(bmi088_data.Gyro[0] - last_gyro_x) < 0.000001f) {
        stuck_count++;
    } else {
        stuck_count = 0;
        bmi088_data.healthy = 1;
    }
    last_gyro_x = bmi088_data.Gyro[0];

    if (stuck_count > 50) {
        bmi088_data.healthy = 0; // 标记传感器不健康，飞控应进入紧急降落模式
    }
    if (bmi088_data.healthy ==1) {
        PubPushFromPool(bmi088_data_pub, &bmi088_data);
    }
}


// ================= 内部辅助函数 (仅初始化用) =================
// 简单的阻塞写
// ================= 内部辅助函数 =================

// 修改后的写寄存器：自动处理 CS 片选，防止连写失败
static void write_reg(uint8_t reg, uint8_t data, uint8_t is_accel)
{
    uint8_t tx[2] = {reg, data};

    if(is_accel)
        BMI088_ACCEL_NS_L();
    else
        BMI088_GYRO_NS_L();

    HAL_SPI_Transmit(bmi_spi, tx, 2, 100);

    if(is_accel)
        BMI088_ACCEL_NS_H();
    else
        BMI088_GYRO_NS_H();

    // BMI088 手册要求：两次 SPI 操作间至少留一点空隙
    // 对于 1MHz SPI，简单的指令执行延迟通常就够了，保险起见延时一点点
    for(volatile int i=0; i<50; i++);
}

/**
 * @brief 读寄存器函数 (支持 CS 切换和 Accel Dummy Byte)
 * @param reg 寄存器地址
 * @param is_accel 1 为加速度计, 0 为陀螺仪
 */
static uint8_t read_reg(uint8_t reg, uint8_t is_accel)
{
    uint8_t tx = reg | 0x80; // 最高位置 1 表示读取
    uint8_t rx;

    // 1. 拉低对应片选
    if (is_accel)
        BMI088_ACCEL_NS_L();
    else
        BMI088_GYRO_NS_L();

    // 2. 发送寄存器地址
    HAL_SPI_Transmit(bmi_spi, &tx, 1, 100);

    // 3. 处理加速度计特有的 Dummy Byte
    // BMI088 手册规定：加速度计在 SPI 模式下读取，地址后第一个字节是无效数据
    if (is_accel)
    {
        HAL_SPI_Receive(bmi_spi, &rx, 1, 100);
    }

    // 4. 读取真正的数据
    HAL_SPI_Receive(bmi_spi, &rx, 1, 100);

    // 5. 拉高对应片选
    if (is_accel)
        BMI088_ACCEL_NS_H();
    else
        BMI088_GYRO_NS_H();

    return rx;
}
/**
 * @brief BMI088 离线校准函数
 * @note  需保持传感器静止。如果环境振动过大，校准会自动循环直到环境安静。
 */
void Calibrate_BMI088(BMI088_Data_t *bmi)
{
    uint8_t buf[8];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f, gNormDiff = 0.0f;
    float gyroMax[3], gyroMin[3], gyroDiff[3];
    float startTime;
    uint16_t CaliTimes = 6000; // 采样次数，越高性能越好但速度慢
    uint8_t success = 0;

    LOGINFO("[BMI088] Calibrating Start. Keep the device still...");
    startTime = DWT_GetTimeline_s();

    do {
        // 每轮循环初始化
        bmi->gNorm = 0;
        for(int j=0; j<3; j++) bmi->GyroOffset[j] = 0;

        // 超时退出保护 (例如 15 秒没校准完说明环境确实太抖)
        if (DWT_GetTimeline_s() - startTime > 15.0f) {
            LOGERROR("[BMI088] Calibration Timeout! Use manual settings.");
            // 设定一套保底偏移（或不改变现状）
            break;
        }

        for (uint16_t i = 0; i < CaliTimes; ++i) {
            // 1. 读取加速度计并计算模长
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            float ax = (int16_t)((buf[1] << 8) | buf[0]) * BMI088_ACCEL_6G_SEN;
            float ay = (int16_t)((buf[3] << 8) | buf[2]) * BMI088_ACCEL_6G_SEN;
            float az = (int16_t)((buf[5] << 8) | buf[4]) * BMI088_ACCEL_6G_SEN;
            gNormTemp = sqrtf(ax * ax + ay * ay + az * az);
            bmi->gNorm += gNormTemp;

            // 2. 读取陀螺仪并累加零偏
            // 注意：某些驱动包这里寄存器名为 BMI088_GYRO_CH_ID, 确认是指向 0x02
            BMI088_gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);
            float gx = (int16_t)((buf[1] << 8) | buf[0]) * BMI088_GYRO_2000_SEN;
            float gy = (int16_t)((buf[3] << 8) | buf[2]) * BMI088_GYRO_2000_SEN;
            float gz = (int16_t)((buf[5] << 8) | buf[4]) * BMI088_GYRO_2000_SEN;

            float gyro_current[3] = {gx, gy, gz};
            for (int j = 0; j < 3; j++) bmi->GyroOffset[j] += gyro_current[j];

            // 3. 环境稳定性判断 (计算峰峰值)
            if (i == 0) {
                gNormMax = gNormMin = gNormTemp;
                for (int j = 0; j < 3; j++) gyroMax[j] = gyroMin[j] = gyro_current[j];
            } else {
                if (gNormTemp > gNormMax) gNormMax = gNormTemp;
                if (gNormTemp < gNormMin) gNormMin = gNormTemp;
                for (int j = 0; j < 3; j++) {
                    if (gyro_current[j] > gyroMax[j]) gyroMax[j] = gyro_current[j];
                    if (gyro_current[j] < gyroMin[j]) gyroMin[j] = gyro_current[j];
                }
            }

            gNormDiff = gNormMax - gNormMin;
            gyroDiff[0] = gyroMax[0] - gyroMin[0];
            gyroDiff[1] = gyroMax[1] - gyroMin[1];
            gyroDiff[2] = gyroMax[2] - gyroMin[2];

            // 如果此时抖动超过门限，直接放弃本轮并报错重试
            if (gNormDiff > 0.5f || gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f) {
                LOGWARNING("[BMI088] VIBRATION DETECTED! Restarting calibration...");
                DWT_Delay(0.1); // 等待平稳再重启
                i = CaliTimes + 1; // 强制跳出内层循环
            }
            DWT_Delay(0.0005); // 约 2KHz 的校准采样
        }

        // 计算均值
        bmi->gNorm /= (float)CaliTimes;
        for (int j = 0; j < 3; j++) bmi->GyroOffset[j] /= (float)CaliTimes;

        // 最终合规性判断
        // 如果重力模长不在 9.1~10.5 之间，或者偏置极大（比如坏了），就继续循环
        if (fabsf(bmi->gNorm - 9.806f) < 0.5f && fabsf(bmi->GyroOffset[0]) < 0.1f) {
            success = 1;
        }

    } while (!success);

    // 计算缩放比例，修正量程误差
    bmi->AccelScale = 9.80665f / bmi->gNorm;
    // LOGINFO("[BMI088] CALIBRATED SUCCESS! Scale: %.3f Offset: X%.4f Y%.4f Z%.4f",
    //          bmi->AccelScale, bmi->GyroOffset[0], bmi->GyroOffset[1], bmi->GyroOffset[2]);
    LOGERROR("[BMI088] Calibration Success!");

}
static void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t tx = reg | 0x80;
    BMI088_ACCEL_NS_L();
    HAL_SPI_Transmit(bmi_spi, &tx, 1, 100);
    uint8_t dummy;
    HAL_SPI_Receive(bmi_spi, &dummy, 1, 100); // Dummy byte for Accel
    HAL_SPI_Receive(bmi_spi, data, len, 100);
    BMI088_ACCEL_NS_H();
}

static void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t tx = reg | 0x80;
    BMI088_GYRO_NS_L();
    HAL_SPI_Transmit(bmi_spi, &tx, 1, 100);
    HAL_SPI_Receive(bmi_spi, data, len, 100);
    BMI088_GYRO_NS_H();
}