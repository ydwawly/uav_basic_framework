//
// Created by Administrator on 2026/1/30.
//

#include "bmi270_port.h"
#include "bmi270.h"
#include "bsp_dwt.h"
/* * 定义一个简单的上下文结构体，
 * 这样你可以通过 intf_ptr 传入不同的 SPI 句柄或 CS 引脚
 */

/* * 1. 读函数 (Read)
 * 关键点：
 * - 发送地址时，首位(MSB)需置 1 (0x80) 表示读取
 * - 发送地址后，必须紧跟一个 Dummy Byte (空字节)，之后传感器才吐数据
 */
/* 1. 读函数 (Read) - 模仿你手动成功的代码 */
int8_t bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t rslt = 0;
    bmi2_interface_t *intf = (bmi2_interface_t *)intf_ptr;

    // 1. 拉低片选
    HAL_GPIO_WritePin(intf->cs_port, intf->cs_pin, GPIO_PIN_RESET);

    // 2. 发送地址 (Read bit 0x80)
    uint8_t tx_buff = reg_addr | 0x80;
    if (HAL_SPI_Transmit(intf->hspi, &tx_buff, 1, 100) != HAL_OK) {
        rslt = -1;
    }

    // 3. 直接读取 len 长度的数据
    // 注意：Bosch 驱动请求的 len 已经包含了 Dummy Byte 的长度 (例如读1字节它会申请2字节)
    // 所以这里不需要再手动跳过 dummy，直接把 dummy 读进 buffer[0] 交给驱动去处理即可
    if (rslt == 0) {
        if (HAL_SPI_Receive(intf->hspi, reg_data, len, 1000) != HAL_OK) {
            rslt = -1;
        }
    }

    // 4. 拉高片选
    HAL_GPIO_WritePin(intf->cs_port, intf->cs_pin, GPIO_PIN_SET);

    return rslt;
}

/* * 2. 写函数 (Write)
 * 关键点：
 * - 发送地址时，首位(MSB)需置 0 (0x7F) 表示写入
 * - 写操作不需要 Dummy Byte
 */
int8_t bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t rslt = 0;
    bmi2_interface_t *intf = (bmi2_interface_t *)intf_ptr;

    // 1. 拉低片选
    HAL_GPIO_WritePin(intf->cs_port, intf->cs_pin, GPIO_PIN_RESET);

    // 2. 准备地址字节 (Write operation: bit 7 = 0)
    uint8_t tx_buff = reg_addr & 0x7F;

    // 3. 发送地址
    if (HAL_SPI_Transmit(intf->hspi, &tx_buff, 1, 100) != HAL_OK) {
        rslt = -1;
    }

    // 4. 发送数据
    if (rslt == 0) {
        if (HAL_SPI_Transmit(intf->hspi, (uint8_t *)reg_data, len, 1000) != HAL_OK) {
            rslt = -1;
        }
    }

    // 5. 拉高片选
    HAL_GPIO_WritePin(intf->cs_port, intf->cs_pin, GPIO_PIN_SET);

    return rslt;
}

/* * 3. 延时函数 (Delay)
 * 注意：Bosch API 请求的是"微秒"(us)，HAL_Delay 是"毫秒"(ms)
 * 在初始化阶段，用毫秒代替微秒通常没问题（只是变慢了）。
 * 如果追求极致，请使用定时器实现 us 级延时。
 */
/* 替换掉原来的函数 */
/* 替换掉原来的函数，不要用 HAL_Delay 也不要用 DWT */
void bmi2_delay_us(uint32_t period, void *intf_ptr) {
    float s = (float)period / 1000000.0f;
    DWT_Delay(s);
}