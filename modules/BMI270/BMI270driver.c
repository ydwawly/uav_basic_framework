//
// Created by Administrator on 2026/1/30.
//

#include "BMI270driver.h"
#include "bmi270.h"      // 官方库头文件
#include "bmi2_defs.h"   // 官方定义
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "message_center.h"
#include <math.h>
#include <string.h>

#include "BMI088driver.h"

// ================= 宏定义与变量 =================
#define BMI270_ACC_X_LSB_REG    0x0C
#define BMI270_DMA_LEN          14   // 1(CMD) + 1(Dummy) + 6(Acc) + 6(Gyr)

// 灵敏度 (4G, 2000dps)
#define BMI270_ACC_SEN       (16.0f / 32768.0f)
#define BMI270_GYRO_SEN_DEG  (2000.0f / 32768.0f)
#define DEG_TO_RAD           0.01745329252f

static BMI270_Data_t bmi270_data;
static Publisher_t *bmi270_data_pub;
SPI_HandleTypeDef *bmi270_spi;

// DMA 缓冲区
// static uint8_t bmi270_tx[BMI270_DMA_LEN];
// static uint8_t bmi270_rx[BMI270_DMA_LEN];
// DMA 缓冲区 (强制放入无 Cache 区，并按 32 字节对齐防止踩踏)
__attribute__((section(".dma_buffer"), aligned(32))) static uint8_t bmi270_tx[32];
__attribute__((section(".dma_buffer"), aligned(32))) static uint8_t bmi270_rx[32];
volatile uint8_t bmi270_dma_busy = 0;

// 官方库需要的接口结构体 (仅初始化用)
extern struct bmi2_dev bmi270_dev;
extern void BMI270_Init_Process(void); // 引用你原来的初始化代码

// 内部函数声明
static void Calibrate_BMI270(BMI270_Data_t *bmi);
static void read_multi_reg_blocking(uint8_t reg, uint8_t *data, uint8_t len);
volatile uint8_t bmi270_init_done = 0; // 新增：初始化完成标志
// ================= 1. 初始化接口 =================
uint8_t BMI270_Driver_Init(SPI_HandleTypeDef *hspi)
{
    bmi270_spi = hspi;

    // 1. 调用你之前调试通的官方初始化流程
    // 注意：BMI270_Init_Process 里应该包含 bmi270_init() 和 sensor_enable()
    // 务必确保你的 BMI270_Init_Process 里的 hspi 指针和这里的 hspi 一致
    BMI270_Init_Process();
    bmi270_data.AccelScale = 1.0f;

    // 2. 执行离线校准 (阻塞式，耗时约几秒)
    Calibrate_BMI270(&bmi270_data);

    // 3. 准备 DMA 发送缓冲区
    // 读命令: 0x0C | 0x80 (地址0x0C, 读位置1)
    bmi270_tx[0] = BMI270_ACC_X_LSB_REG | 0x80;
    memset(&bmi270_tx[1], 0xFF, BMI270_DMA_LEN - 1);

    // 4. 注册发布者
    bmi270_data.healthy = 1;
    bmi270_data_pub = PubRegister("bmi270_data", sizeof(BMI270_Data_t));

    bmi270_init_done = 1; // ✨ 新增：校准全弄完了，才允许处理中断！

    LOGINFO("[BMI270] BMI270 Init Success ！");

    return 0;
}

// ================= 2. DMA 读取启动 =================
void BMI270_Read_DMA_Start(void)
{
    if(bmi270_dma_busy) return;

    bmi270_dma_busy = 1;
    BMI270_CS_L();

    // ✨ 检查返回值，如果不是 HAL_OK (比如 HAL_BUSY 或 HAL_ERROR)
    if (HAL_SPI_TransmitReceive_DMA(bmi270_spi, bmi270_tx, bmi270_rx, BMI270_DMA_LEN) != HAL_OK)
    {
        // 🚨 启动失败！必须释放总线和状态位，防止永远卡死
        BMI270_CS_H();
        bmi270_dma_busy = 0;
    }
}

// ================= 3. DMA 回调处理 =================
// 需在 HAL_SPI_TxRxCpltCallback 中调用
// ================= 3. DMA 回调处理 (修正版) =================
void BMI270_DMA_Callback(void)
{
    // 使用 float 临时变量，防止精度丢失
    static float acc_phys[3];
    static float gyr_phys[3];
    static float last_gyro_x = 0;
    static uint32_t stuck_count = 0;

    if (bmi270_dma_busy)
    {
        BMI270_CS_H();
        bmi270_dma_busy = 0;

        // --- 1. 原始数据拼接 (纯整数操作) ---
        int16_t raw_acc_x = (int16_t)((bmi270_rx[3] << 8) | bmi270_rx[2]);
        int16_t raw_acc_y = (int16_t)((bmi270_rx[5] << 8) | bmi270_rx[4]);
        int16_t raw_acc_z = (int16_t)((bmi270_rx[7] << 8) | bmi270_rx[6]);

        int16_t raw_gyr_x = (int16_t)((bmi270_rx[9] << 8)  | bmi270_rx[8]);
        int16_t raw_gyr_y = (int16_t)((bmi270_rx[11] << 8) | bmi270_rx[10]);
        int16_t raw_gyr_z = (int16_t)((bmi270_rx[13] << 8) | bmi270_rx[12]);

        // --- 2. 物理量转换与坐标系对齐 ---
        // 假设板载坐标系定义：
        // BMI270 X -> 机体 Y
        // BMI270 Y -> 机体 X
        // BMI270 Z -> 机体 Z

        // Accel 处理 (单位 m/s^2 或 g，取决于你 scale 的定义，通常推荐归一化为 m/s^2)
        // 这里的 Scale 在校准时被计算为 9.8/Norm，所以乘出来直接是 m/s^2
        bmi270_data.Accel[IMU_Y] = raw_acc_x * BMI270_ACC_SEN * bmi270_data.AccelScale;
        bmi270_data.Accel[IMU_X] = raw_acc_y * BMI270_ACC_SEN * bmi270_data.AccelScale;
        bmi270_data.Accel[IMU_Z] = raw_acc_z * BMI270_ACC_SEN * bmi270_data.AccelScale;

        // Gyro 处理 (核心修正：先转物理值，再减 Offset，最后转 Rad)
        // 临时变量存储 deg/s
        float gyr_deg_x = raw_gyr_x * BMI270_GYRO_SEN_DEG;
        float gyr_deg_y = raw_gyr_y * BMI270_GYRO_SEN_DEG;
        float gyr_deg_z = raw_gyr_z * BMI270_GYRO_SEN_DEG;

        // 减去零偏 (注意：Offset 也是 deg/s 单位)
        // 坐标系映射时要注意 Offset 的对应关系！
        // 假设 Offset[0] 是 BMI_X 的零偏
        float clean_gyr_x = gyr_deg_x - bmi270_data.GyroOffset[0];
        float clean_gyr_y = gyr_deg_y - bmi270_data.GyroOffset[1];
        float clean_gyr_z = gyr_deg_z - bmi270_data.GyroOffset[2];

        // 赋值给数据结构 (转为 rad/s 用于控制)
        bmi270_data.Gyro[IMU_Y] =  -clean_gyr_x * DEG_TO_RAD; // 板载X对应机体Y
        bmi270_data.Gyro[IMU_X] =  -clean_gyr_y * DEG_TO_RAD; // 板载Y对应机体X
        bmi270_data.Gyro[IMU_Z] =  -clean_gyr_z * DEG_TO_RAD; // 板载Z对应机体Z

        // 调试用的 deg/s 数据
        bmi270_data.GyroDeg[0] = clean_gyr_x;
        bmi270_data.GyroDeg[1] = clean_gyr_y;
        bmi270_data.GyroDeg[2] = clean_gyr_z;

        // --- 3. 健康检测 ---
        // 检查是否卡死 (Stuck)
        if (fabsf(bmi270_data.Gyro[IMU_X] - last_gyro_x) < 0.000001f) stuck_count++;
        else stuck_count = 0;
        last_gyro_x = bmi270_data.Gyro[IMU_X];

        // 100ms 无变化视为故障 (1kHz * 100)
        if (stuck_count > 100) bmi270_data.healthy = 0;
        else bmi270_data.healthy = 1;

        // --- 4. 发布数据 ---
        if (bmi270_data_pub) { // 判空，防止未注册导致死机
             PubPushFromPool(bmi270_data_pub, &bmi270_data);
        }
    }
}

// ================= 4. 校准函数 (阻塞) =================
/**
 * @brief BMI270 离线校准函数 (完全复刻 BMI088 逻辑)
 * @note  需保持传感器静止。校准后 Accel 输出单位为 m/s², Gyro 输出单位为 deg/s
 */
// ================= 4. 校准函数 (修正版) =================
static void Calibrate_BMI270(BMI270_Data_t *bmi)
{
    uint8_t buf[12];
    float gNormTemp, gNormMax, gNormMin, gNormDiff;
    float gyroMax[3], gyroMin[3], gyroDiff[3];
    float startTime;
    uint16_t CaliTimes = 3000;
    uint8_t success = 0;

    LOGINFO("[BMI270] Calibration Start. Keep static...");
    startTime = DWT_GetTimeline_s();

    do {
        // 重置累加器
        bmi->gNorm = 0;
        for(int j=0; j<3; j++) bmi->GyroOffset[j] = 0;

        // 超时检查
        if (DWT_GetTimeline_s() - startTime > 15.0f) {
            LOGERROR("[BMI270] Calibration Timeout!");
            bmi->AccelScale = 1.0f; // 失败时保持原样
            memset(bmi->GyroOffset, 0, sizeof(bmi->GyroOffset));
            break;
        }

        for (uint16_t i = 0; i < CaliTimes; ++i) {
            read_multi_reg_blocking(BMI270_ACC_X_LSB_REG, buf, 12);

            // --- A. 加速度 (使用 float 计算 g 值) ---
            int16_t rax = (int16_t)((buf[1] << 8) | buf[0]);
            int16_t ray = (int16_t)((buf[3] << 8) | buf[2]);
            int16_t raz = (int16_t)((buf[5] << 8) | buf[4]);

            float ax = rax * BMI270_ACC_SEN;
            float ay = ray * BMI270_ACC_SEN;
            float az = raz * BMI270_ACC_SEN;
            gNormTemp = sqrtf(ax*ax + ay*ay + az*az);
            bmi->gNorm += gNormTemp;

            // --- B. 陀螺仪 (全程使用 deg/s 计算，避免单位混淆) ---
            int16_t rgx = (int16_t)((buf[7] << 8) | buf[6]);
            int16_t rgy = (int16_t)((buf[9] << 8) | buf[8]);
            int16_t rgz = (int16_t)((buf[11] << 8) | buf[10]);

            float gx = rgx * BMI270_GYRO_SEN_DEG;
            float gy = rgy * BMI270_GYRO_SEN_DEG;
            float gz = rgz * BMI270_GYRO_SEN_DEG;

            // 累加 Offset (deg/s)
            bmi->GyroOffset[0] += gx;
            bmi->GyroOffset[1] += gy;
            bmi->GyroOffset[2] += gz;

            // --- C. 抖动检测 ---
            if (i == 0) {
                gNormMax = gNormMin = gNormTemp;
                for(int k=0; k<3; k++) gyroMax[k] = gyroMin[k] = (k==0?gx:k==1?gy:gz);
            } else {
                if(gNormTemp > gNormMax) gNormMax = gNormTemp;
                if(gNormTemp < gNormMin) gNormMin = gNormTemp;

                float cur_g[3] = {gx, gy, gz};
                for(int k=0; k<3; k++) {
                    if(cur_g[k] > gyroMax[k]) gyroMax[k] = cur_g[k];
                    if(cur_g[k] < gyroMin[k]) gyroMin[k] = cur_g[k];
                }
            }

            // 检查阈值
            gNormDiff = gNormMax - gNormMin;
            for(int k=0; k<3; k++) gyroDiff[k] = gyroMax[k] - gyroMin[k];

            // 阈值：加速度 0.1g (更严格)，陀螺仪 2.0 deg/s
            if (gNormDiff > 0.1f || gyroDiff[0] > 2.0f || gyroDiff[1] > 2.0f || gyroDiff[2] > 2.0f) {
                // LOGINFO("Moving detected, restarting...");
                i = CaliTimes + 1; // 强制重置
                DWT_Delay(0.05); // 稍作等待
            }
            DWT_Delay(0.0005);
        }

        bmi->gNorm /= (float)CaliTimes;
        for(int j=0; j<3; j++) bmi->GyroOffset[j] /= (float)CaliTimes;

        // 验证合法性
        if (fabsf(bmi->gNorm - 1.0f) < 0.15f && fabsf(bmi->GyroOffset[0]) < 10.0f) {
            success = 1;

        }
    } while (!success);

    if (success) {
        // 计算 Scale: 使得输出为 9.80665 m/s^2
        bmi->AccelScale = 9.80665f / bmi->gNorm;
        // LOGINFO("[BMI270] Calib Success. Scale: %.4f, Offset X: %.3f", bmi->AccelScale, bmi->GyroOffset[0]);
    }
}
// 简单的阻塞读取 (仅用于校准)
static void read_multi_reg_blocking(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx = reg | 0x80;
    uint8_t dummy;

    BMI270_CS_L();
    HAL_SPI_Transmit(bmi270_spi, &tx, 1, 100);
    HAL_SPI_Receive(bmi270_spi, &dummy, 1, 100); // Dummy Byte
    HAL_SPI_Receive(bmi270_spi, data, len, 100);
    BMI270_CS_H();
}