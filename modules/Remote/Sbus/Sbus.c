//
// Created by Administrator on 2026/3/2.
//
#include "FreeRTOS.h"
#include "Sbus.h"

#include "bsp_log.h"
#include "bsp_usart.h"
#include "SensorHub.h"
#include "usart.h"
#include "message_center.h"
static USARTInstance *sbus_instance;
static Subs_RC_Data_t subs_rc_data;
static Publisher_t *remote_data_pub = NULL;
// static uint8_t sbus_task_buf[SBUS_FRAME_SIZE];
// 记录当前最新的一帧 SBUS 数据指针（指向双缓冲区的某一半）
static uint8_t *p_sbus_rx_frame = NULL;
static void Sbus_Rx_Callback(uint8_t *buf, uint16_t len);

void Sbus_Init(void)
{
    USART_Init_Config_s config;
    config.usart_handle = &huart6;
    config.recv_buff_size = SBUS_FRAME_SIZE;
    config.module_callback = Sbus_Rx_Callback;
    config.rx_mode = USART_RX_MODE_DOUBLE_BUF;

    sbus_instance = USARTRegister(&config);

    LOGINFO("[Sbus] Sbus Init Success !");

    remote_data_pub = PubRegister("remote_data",sizeof(Subs_RC_Data_t));
}

/**
 * @brief SBUS 协议解析核心函数 (11bit 拼接)
 * @param buf 接收到的原始字节数组 (长度已由底层保证为 25)
 */
static void Sbus_Decode(uint8_t *buf)
{
    // 1. 帧头校验 (标准 SBUS 必须是 0x0F)
    // 如果是 DJI DBUS (18字节)，请注释掉此行
    if (buf[0] != SBUS_HEADER) return;

    // 2. 位操作解析 (这是 SBUS 协议最恶心的部分，直接照抄即可)
    // 将 22 个字节的数据 (Byte 1 ~ Byte 22) 还原为 16 个 11bit 的通道值
    subs_rc_data.ch_raw[0]  = ((buf[1]       | buf[2] << 8)                  & 0x07FF);
    subs_rc_data.ch_raw[1]  = ((buf[2] >> 3  | buf[3] << 5)                  & 0x07FF);
    subs_rc_data.ch_raw[2]  = ((buf[3] >> 6  | buf[4] << 2 | buf[5] << 10)   & 0x07FF);
    subs_rc_data.ch_raw[3]  = ((buf[5] >> 1  | buf[6] << 7)                  & 0x07FF);
    subs_rc_data.ch_raw[4]  = ((buf[6] >> 4  | buf[7] << 4)                  & 0x07FF);
    subs_rc_data.ch_raw[5]  = ((buf[7] >> 7  | buf[8] << 1 | buf[9] << 9)    & 0x07FF);
    subs_rc_data.ch_raw[6]  = ((buf[9] >> 2  | buf[10] << 6)                 & 0x07FF);
    subs_rc_data.ch_raw[7]  = ((buf[10] >> 5 | buf[11] << 3)                 & 0x07FF);
    subs_rc_data.ch_raw[8]  = ((buf[12]      | buf[13] << 8)                 & 0x07FF);
    subs_rc_data.ch_raw[9]  = ((buf[13] >> 3 | buf[14] << 5)                 & 0x07FF);
    subs_rc_data.ch_raw[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
    subs_rc_data.ch_raw[11] = ((buf[16] >> 1 | buf[17] << 7)                 & 0x07FF);
    subs_rc_data.ch_raw[12] = ((buf[17] >> 4 | buf[18] << 4)                 & 0x07FF);
    subs_rc_data.ch_raw[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9)  & 0x07FF);
    subs_rc_data.ch_raw[14] = ((buf[20] >> 2 | buf[21] << 6)                 & 0x07FF);
    subs_rc_data.ch_raw[15] = ((buf[21] >> 5 | buf[22] << 3)                 & 0x07FF);

    // 3. 解析标志位 (Byte 23)
    // Bit 7: Channel 17, Bit 6: Channel 18, Bit 3: Frame Lost, Bit 2: Failsafe
    subs_rc_data.ch17 = (buf[23] & 0x80) ? 1 : 0;
    subs_rc_data.ch18 = (buf[23] & 0x40) ? 1 : 0;
    subs_rc_data.frame_lost = (buf[23] & 0x04) ? 1 : 0;
    subs_rc_data.failsafe = (buf[23] & 0x08) ? 1 : 0;

    PubPushFromPool(remote_data_pub, &subs_rc_data);
}

void Sbus_Rx_Callback(uint8_t *buf, uint16_t len)
{
    // 简单校验长度 (增强鲁棒性)
    if (len != SBUS_FRAME_SIZE) return;

    // 【零拷贝】直接记录当前缓冲区的首地址
    p_sbus_rx_frame = buf;

    // 发送任务通知
    BaseType_t xWoken = pdFALSE;
    if (SensorHub_Task_Handle != NULL) {
        xTaskNotifyFromISR(SensorHub_Task_Handle, NOTIFY_BIT_REMOTE, eSetBits, &xWoken);
    }
    // xTaskNotifyFromISR(SensorHub_Task_Handle, NOTIFY_BIT_REMOTE, eSetBits, &xWoken);
    portYIELD_FROM_ISR(xWoken);
}

/**
 * @brief 供 SensorHub_Task 调用
 */
void Sbus_Task_Handler(void)
{
    if (p_sbus_rx_frame != NULL)
    {
        Sbus_Decode(p_sbus_rx_frame);
    }
    p_sbus_rx_frame = NULL;
}