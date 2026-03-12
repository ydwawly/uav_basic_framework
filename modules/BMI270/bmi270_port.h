//
// Created by Administrator on 2026/1/30.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_3_BMI270_PORT_H
#define UAV_BAICE_FRAMEWORK_V1_3_BMI270_PORT_H
#include "main.h" // 包含你的 HAL 库定义，如 SPI_HandleTypeDef

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} bmi2_interface_t;

/* * 1. 读函数 (Read)
 * 关键点：
 * - 发送地址时，首位(MSB)需置 1 (0x80) 表示读取
 * - 发送地址后，必须紧跟一个 Dummy Byte (空字节)，之后传感器才吐数据
 */
int8_t bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
/* * 2. 写函数 (Write)
 * 关键点：
 * - 发送地址时，首位(MSB)需置 0 (0x7F) 表示写入
 * - 写操作不需要 Dummy Byte
 */
int8_t bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
/* * 3. 延时函数 (Delay)
 * 注意：Bosch API 请求的是"微秒"(us)，HAL_Delay 是"毫秒"(ms)
 * 在初始化阶段，用毫秒代替微秒通常没问题（只是变慢了）。
 * 如果追求极致，请使用定时器实现 us 级延时。
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr);
#endif //UAV_BAICE_FRAMEWORK_V1_3_BMI270_PORT_H