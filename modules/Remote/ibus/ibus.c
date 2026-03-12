//
// Created by Administrator on 2026/3/5.
//

#include "FreeRTOS.h"
#include "ibus.h"
#include "bsp_usart.h"
#include "SensorHub.h"
#include "usart.h"

static USARTInstance *ibus_instance;
static Ibus_RC_Data_t ibus_rc_data;

// 记录当前最新的一帧 i-BUS 数据指针（指向双缓冲区的某一半）
static uint8_t * volatile p_ibus_rx_frame = NULL;
static void Ibus_Rx_Callback(uint8_t *buf, uint16_t len);

void Ibus_Init(void)
{
      USART_Init_Config_s config;
      config.usart_handle = &huart1;  // 根据实际使用的串口修改
      config.recv_buff_size = IBUS_FRAME_SIZE;
      config.module_callback = Ibus_Rx_Callback;
      config.rx_mode = USART_RX_MODE_DOUBLE_BUF;

      ibus_instance = USARTRegister(&config);
      ibus_rc_data.is_connected = 0;
}

/**

@brief i-BUS 协议解析核心函数

@param buf 接收到的原始字节数组 (长度已由底层保证为 32)
*/
static void Ibus_Decode(uint8_t *buf)
{
    // 1. 帧头校验
    if (buf[0] != IBUS_HEADER1 || buf[1] != IBUS_HEADER2)
    {
        return;
    }

    // 2. 校验和验证 (Checksum)
    // 校验和 = 0xFFFF 减去前 30 个字节的总和
    uint16_t checksum_calc = 0xFFFF;
    for (uint8_t i = 0; i < 30; i++)
    {
        checksum_calc -= buf[i];
    }

    uint16_t checksum_received = buf[30] | (buf[31] << 8);

    // 如果校验失败，直接丢弃该帧，防止舵机或电机异常抖动
    if (checksum_calc != checksum_received)
    {
        ibus_rc_data.is_connected = 0;
        return;
    }

    // 3. 提取通道数据
    // i-BUS 数据格式非常规整，每个通道 2 字节，低位在前，高位在后
    for (uint8_t i = 0; i < IBUS_CH_TOTAL; i++)
    {
        ibus_rc_data.ch_raw[i] = buf[2 + i * 2] | (buf[3 + i * 2] << 8);

        // 可选：在此处顺手完成归一化 (-1.0f 到 1.0f)
        // ibus_rc_data.ch_normalized[i] = ((float)ibus_rc_data.ch_raw[i] - IBUS_CH_MID) / (float)(IBUS_CH_MAX - IBUS_CH_MID);
    }

    // 解析并校验成功，标记为已连接
    ibus_rc_data.is_connected = 1;
}

void Ibus_Rx_Callback(uint8_t *buf, uint16_t len)
{
    // 简单校验长度 (增强鲁棒性)
    if (len != IBUS_FRAME_SIZE) return;

    // 【零拷贝】直接记录当前缓冲区的首地址
    p_ibus_rx_frame = buf;

    // 发送任务通知
    // 注意：你需要在 SensorHub.h 中定义 NOTIFY_BIT_IBUS，或者复用之前的位
    BaseType_t xWoken = pdFALSE;
    xTaskNotifyFromISR(SensorHub_Task_Handle, NOTIFY_BIT_REMOTE, eSetBits, &xWoken);
    portYIELD_FROM_ISR(xWoken);
}

/**

@brief 供 SensorHub_Task 调用
*/
void Ibus_Task_Handler(void)
{
    if (p_ibus_rx_frame != NULL)
    {
        Ibus_Decode(p_ibus_rx_frame);
        // 解码完成后将指针置空，防止同一帧数据被重复解析
        p_ibus_rx_frame = NULL;
    }
}