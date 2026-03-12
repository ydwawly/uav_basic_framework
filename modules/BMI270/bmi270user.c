//
// Created by Administrator on 2026/1/30.
//

#include "bmi270user.h"
#include "bmi270.h"
#include "main.h"
#include "bmi2_defs.h"
#include "bmi270_port.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "spi.h"

int8_t rslt;

// 定义设备结构体
struct bmi2_dev bmi270_dev;
// 定义硬件接口上下文
bmi2_interface_t bmi270_intf = {
    .hspi = &hspi3,
    .cs_port = CS_BMI270_GPIO_Port,      // 假设 CS 接在 PA4
    .cs_pin = CS_BMI270_Pin,
};

void BMI270_Init_Process(void) {
    DWT_Delay(0.05f); // 上电延时，确保传感器稳定
    // ================= ✨ 修复点：唤醒 BMI270 进入 SPI 模式 =================
    // 制造一个干净的 CS 下降沿和上升沿，强制芯片切换为 SPI 协议
    HAL_GPIO_WritePin(bmi270_intf.cs_port, bmi270_intf.cs_pin, GPIO_PIN_RESET);
    DWT_Delay(0.001f); // 延时 1ms
    HAL_GPIO_WritePin(bmi270_intf.cs_port, bmi270_intf.cs_pin, GPIO_PIN_SET);
    DWT_Delay(0.005f); // 延时 5ms 等待芯片内部状态机切换完毕
    // ========================================================================
    // 1. 绑定接口
    bmi270_dev.read = bmi2_spi_read;
    bmi270_dev.write = bmi2_spi_write;
    bmi270_dev.delay_us = bmi2_delay_us;
    bmi270_dev.intf = BMI2_SPI_INTF; // 选择 SPI 接口
    bmi270_dev.read_write_len = 32;  // 最大传输长度限制，通常32或更大都可以
    bmi270_dev.intf_ptr = &bmi270_intf; // 传入我们的硬件句柄
    // --- DEBUG START ---
    uint8_t chip_id = 0;
    uint8_t reg_addr = 0x00; // Chip ID 寄存器地址

    // 手动拉低 CS
    HAL_GPIO_WritePin(bmi270_intf.cs_port, bmi270_intf.cs_pin, GPIO_PIN_RESET);

    // 1. 发送寄存器地址 (0x80 | 0x00) -> 读操作最高位置1
    uint8_t tx_data = 0x80 | reg_addr;
    HAL_SPI_Transmit(bmi270_intf.hspi, &tx_data, 1, 100);

    // 2. 接收 Dummy Byte (必须跳过这个字节)
    uint8_t dummy_byte = 0;
    HAL_SPI_Receive(bmi270_intf.hspi, &dummy_byte, 1, 100);

    // 3. 读取有效数据
    HAL_SPI_Receive(bmi270_intf.hspi, &chip_id, 1, 100);

    // 手动拉高 CS
    HAL_GPIO_WritePin(bmi270_intf.cs_port, bmi270_intf.cs_pin, GPIO_PIN_SET);

    LOGINFO("DEBUG: Manual Read Chip ID: 0x%X (Expected: 0x24)\n", chip_id);

    if (chip_id != 0x24) {
        LOGINFO("SPI Error: Communication failed, stop init.\n");
        return; // 读不到 ID，后面都不用跑了
    }
    // --- DEBUG END ---
    // 2. 复位并加载固件 (这一步耗时较长，因为要加载 config file)
    // 确保你的工程中包含了 `bmi270_config_file` 数组
    rslt = bmi270_init(&bmi270_dev);

    if (rslt == BMI2_OK) {
        LOGINFO("BMI270 Init Success! Chip ID: 0x%X\n", bmi270_dev.chip_id);
    } else {
        LOGINFO("BMI270 Init Failed. Error: %d\n", rslt);
        return;
    }
    uint8_t internal_status = 0;
    bmi2_get_internal_status(&internal_status, &bmi270_dev);
    LOGINFO("Internal Status: 0x%X", internal_status);
    // 3. 配置传感器 (Accel + Gyro)
    struct bmi2_sens_config config[2];

    // 配置加速度计
    config[0].type = BMI2_ACCEL;
    config[0].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1; // 带宽参数
    config[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ; // 输出速率 800Hz
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_16G;

    // 配置陀螺仪
    config[1].type = BMI2_GYRO;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_3200HZ; // 陀螺仪建议高采样率 (如 1.6kHz 或 3.2kHz)
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    // 应用配置
    rslt = bmi2_set_sensor_config(config, 2, &bmi270_dev);
    bmi2_set_adv_power_save(BMI2_DISABLE, &bmi270_dev);

    // 4. 开启传感器数据输出
    uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    rslt = bmi2_sensor_enable(sensor_list, 2, &bmi270_dev);

    // ================== 新增：配置 BMI270 硬件中断 ==================
    struct bmi2_int_pin_config int_pin_cfg;

    // 1. 配置 INT1 引脚属性：推挽输出、高电平有效、输出使能
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_NON_LATCH; // 非锁存模式
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    rslt = bmi2_set_int_pin_config(&int_pin_cfg, &bmi270_dev);

    // 2. 将“数据就绪 (Data Ready)”信号映射到 INT1 引脚
    // BMI270 会在陀螺仪/加速度计更新后自动触发该引脚
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi270_dev);

    if (rslt != BMI2_OK) {
        LOGINFO("BMI270 Map Interrupt Failed: %d\n", rslt);
    } else {
        LOGINFO("BMI270 Interrupt Configured on INT1\n");
    }
    // =============================================================
}