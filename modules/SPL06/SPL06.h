#ifndef UAV_BAICE_FRAMEWORK_V1_4_SPL06_H
#define UAV_BAICE_FRAMEWORK_V1_4_SPL06_H

#include "main.h"

/* --- 内部私有宏定义 --- */
#define SPL06DeviceAdd      (0x76 << 1)
#define SPL06_PSR_B2        0x00
#define SPL06_PSR_CFG       0x06
#define SPL06_TMP_CFG       0x07
#define SPL06_MEAS_CFG      0x08
#define SPL06_CFG_REG       0x09
#define SPL06_RESET         0x0C
#define SPL06_COEF          0x10

#define PM_RATE_4           (2<<4)
#define PM_PRC_32           5
#define TMP_RATE_4          (2<<4)
#define TMP_PRC_8           3
#define MEAS_CTRL_ContinuousPressTemp 0x07
#define SPL06_CFG_P_SHIFT   0x04

#define STANDARD_SEA_LEVEL_PRESSURE 101325.0f

/* --- 内部私有数据结构 --- */
typedef struct {
    int16_t C0, C1, C01, C11, C20, C21, C30;
    int32_t C00, C10;
    float kT, kP;
} T_SPL06_calibPara;

// 飞控高度数据结构体 (作为发布出去的消息格式)
typedef struct {
    float temperature;      // 当前温度 (摄氏度)
    float pressure;         // 当前气压 (Pa)

    float absolute_alt;     // 绝对海拔高度 (m) (基于标准大气压，仅供地面站参考)
    float ground_pressure;  // 起飞地面基准气压 (Pa)
    float relative_alt;     // 相对地面高度 (m) (带真实温度补偿)
    float filtered_alt;     // 滤波后的平滑高度 (m) (喂给 PID 的数据)

    uint8_t is_calibrated;  // 零点校准完成标志 (1:完成, 0:未完成)
} UAV_Altitude_Data_t;

/* --- 对外公开的 API --- */
void SPL06_Init_DMA(I2C_HandleTypeDef *hi2c); // 初始化并注册发布者
void SPL06_Read_DMA_Start(void);              // 触发一次 DMA 读取
void SPL06_Data_Handler(void);                // DMA 接收完成后的数据处理与发布

#endif // UAV_BAICE_FRAMEWORK_V1_4_SPL06_H