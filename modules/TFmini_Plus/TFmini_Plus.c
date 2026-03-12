//
// Created by Administrator on 2026/3/3.
//

#include "FreeRTOS.h"
#include "TFmini_Plus.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "SensorHub.h"
#include "usart.h"
#include "message_center.h"

static USARTInstance *tfmini_instance;
static TFminiPlus_Data_t tfmini_data;
static Publisher_t *tfmini_data_pub = NULL;

// 记录当前最新的一帧 TFmini 数据指针（指向双缓冲区的某一半）
static uint8_t *p_tfmini_rx_frame = NULL;

static void TFmini_Rx_Callback(uint8_t *buf, uint16_t len);

void TFmini_Init(void)
{
    USART_Init_Config_s config;
    config.usart_handle = &huart1;
    config.recv_buff_size = TFMINI_FRAME_SIZE; // 帧长为9字节
    config.module_callback = TFmini_Rx_Callback;
    config.rx_mode = USART_RX_MODE_DOUBLE_BUF;

    tfmini_instance = USARTRegister(&config);

    LOGINFO("[TFmini] TFmini Plus Init Success !");

    // 注册发布者
    tfmini_data_pub = PubRegister("tfmini_data", sizeof(TFminiPlus_Data_t));
}

/**
 * @brief TFmini Plus 协议解析核心函数
 * @param buf 接收到的原始字节数组 (长度已由底层保证为 9)
 */
static void TFmini_Decode(uint8_t *buf)
{
    // 1. 帧头校验：Byte0 和 Byte1 必须都是 0x59
    if (buf[0] != TFMINI_HEADER || buf[1] != TFMINI_HEADER) return;

    // 2. 校验和计算 (前 8 字节的累加和，取低 8 位)
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        checksum += buf[i];
    }

    // 如果校验和不匹配，说明数据错位或传输出错，直接丢弃
    if (checksum != buf[8]) return;

    // 3. 解析距离、强度和温度 (小端模式拼接)
    tfmini_data.distance = (uint16_t)(buf[2] | (buf[3] << 8));
    tfmini_data.strength = (uint16_t)(buf[4] | (buf[5] << 8));

    uint16_t raw_temp = (uint16_t)(buf[6] | (buf[7] << 8));
    // 摄氏度 = Temp/8 - 256
    tfmini_data.temperature = (raw_temp / 8.0f) - 256.0f;

    // 4. 判断数据可信度 [cite: 222, 223]
    // 当信号强度 < 100 或者过曝 (=65535) 时，输出的距离被认为不可信 [cite: 222, 223]
    if (tfmini_data.strength >= 100 && tfmini_data.strength != 65535) {
        tfmini_data.is_valid = 1;
    } else {
        tfmini_data.is_valid = 0;
    }

    // 5. 推送数据到消息中心
    PubPushFromPool(tfmini_data_pub, &tfmini_data);
}

void TFmini_Rx_Callback(uint8_t *buf, uint16_t len)
{
    // 简单校验长度 (增强鲁棒性)
    if (len != TFMINI_FRAME_SIZE) return;

    // 【零拷贝】直接记录当前缓冲区的首地址
    p_tfmini_rx_frame = buf;

    // 发送任务通知 (记得在 SensorHub.h 中定义 NOTIFY_BIT_TFMINI)
    BaseType_t xWoken = pdFALSE;
    if (SensorHub_Task_Handle != NULL) {
        xTaskNotifyFromISR(SensorHub_Task_Handle, NOTIFY_BIT_TFMINI, eSetBits, &xWoken);
    }
    portYIELD_FROM_ISR(xWoken);
}

/**
 * @brief 供 SensorHub_Task 调用
 */
void TFmini_Task_Handler(void)
{
    if (p_tfmini_rx_frame != NULL)
    {
        TFmini_Decode(p_tfmini_rx_frame);
    }
    p_tfmini_rx_frame = NULL;
}