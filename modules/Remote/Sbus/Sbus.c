//
// Created by Administrator on 2026/3/2.
//
#include "FreeRTOS.h"
#include "Sbus.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "SensorHub.h"
#include "usart.h"
#include "message_center.h"

/* ========================================================== */
/*                    数据结构                                */
/* ========================================================== */
__attribute__((section(".dma_buffer")))
static uint8_t sbus_buf_A[SBUS_FRAME_SIZE];
__attribute__((section(".dma_buffer")))
static uint8_t sbus_buf_B[SBUS_FRAME_SIZE];

// 解析后的数据（双缓冲）
static Sbus_RC_Data_t rc_data_A;
static Sbus_RC_Data_t rc_data_B;

static volatile uint8_t writable_idx = 0;
static volatile uint8_t readable_idx = 0;
static volatile bool new_frame_ready = false;
static volatile uint32_t last_frame_time_ms = 0;

// 可选发布功能
static Publisher_t *remote_data_pub = NULL;
static bool publish_enabled = false;  // 默认关闭

static void Sbus_Rx_Callback(uint8_t *buf, uint16_t len)
{
    if (len != SBUS_FRAME_SIZE) return;

    uint8_t read_idx = readable_idx;
    uint8_t write_idx = (read_idx == 0) ? 1 : 0;

    uint8_t *write_buf = (write_idx == 0) ? sbus_buf_A : sbus_buf_B;
    memcpy((void *)write_buf, buf, SBUS_FRAME_SIZE);

    __DMB();
    writable_idx = write_idx;
    new_frame_ready = true;
    last_frame_time_ms = DWT_GetTimeline_ms();

    BaseType_t xWoken = pdFALSE;
    xTaskNotifyFromISR(SensorHub_Task_Handle,NOTIFY_BIT_REMOTE,eSetBits,&xWoken);
    portYIELD_FROM_ISR(xWoken);
}

void Sbus_Task_Handler(void)
{
    if (!new_frame_ready) return;
    uint8_t write_idx = writable_idx;
    __DMB();
    readable_idx = write_idx;
    new_frame_ready = false;

    const uint8_t *read_buf = (write_idx == 0) ? sbus_buf_A : sbus_buf_B;
    Sbus_RC_Data_t *rc_data = (write_idx == 0) ? &rc_data_A : &rc_data_B;

    rc_data->ch_raw[0]  = ((read_buf[1]       | read_buf[2] << 8)                  & 0x07FF);
    rc_data->ch_raw[1]  = ((read_buf[2] >> 3  | read_buf[3] << 5)                  & 0x07FF);
    rc_data->ch_raw[2]  = ((read_buf[3] >> 6  | read_buf[4] << 2 | read_buf[5] << 10)   & 0x07FF);
    rc_data->ch_raw[3]  = ((read_buf[5] >> 1  | read_buf[6] << 7)                  & 0x07FF);
    rc_data->ch_raw[4]  = ((read_buf[6] >> 4  | read_buf[7] << 4)                  & 0x07FF);
    rc_data->ch_raw[5]  = ((read_buf[7] >> 7  | read_buf[8] << 1 | read_buf[9] << 9)    & 0x07FF);
    rc_data->ch_raw[6]  = ((read_buf[9] >> 2  | read_buf[10] << 6)                 & 0x07FF);
    rc_data->ch_raw[7]  = ((read_buf[10] >> 5 | read_buf[11] << 3)                 & 0x07FF);
    rc_data->ch_raw[8]  = ((read_buf[12]      | read_buf[13] << 8)                 & 0x07FF);
    rc_data->ch_raw[9]  = ((read_buf[13] >> 3 | read_buf[14] << 5)                 & 0x07FF);
    rc_data->ch_raw[10] = ((read_buf[14] >> 6 | read_buf[15] << 2 | read_buf[16] << 10) & 0x07FF);
    rc_data->ch_raw[11] = ((read_buf[16] >> 1 | read_buf[17] << 7)                 & 0x07FF);
    rc_data->ch_raw[12] = ((read_buf[17] >> 4 | read_buf[18] << 4)                 & 0x07FF);
    rc_data->ch_raw[13] = ((read_buf[18] >> 7 | read_buf[19] << 1 | read_buf[20] << 9)  & 0x07FF);
    rc_data->ch_raw[14] = ((read_buf[20] >> 2 | read_buf[21] << 6)                 & 0x07FF);
    rc_data->ch_raw[15] = ((read_buf[21] >> 5 | read_buf[22] << 3)                 & 0x07FF);

    // 3. 解析标志位 (Byte 23)
    // Bit 7: Channel 17, Bit 6: Channel 18, Bit 3: Frame Lost, Bit 2: Failsafe
    rc_data->ch17 = (read_buf[23] & 0x80) ? 1 : 0;
    rc_data->ch18 = (read_buf[23] & 0x40) ? 1 : 0;
    rc_data->frame_lost = (read_buf[23] & 0x04) ? 1 : 0;
    rc_data->failsafe = (read_buf[23] & 0x08) ? 1 : 0;

    if (publish_enabled && remote_data_pub != NULL) {
        PubPushFromPool(remote_data_pub, rc_data);
    }

    // 失联检测
    if (rc_data->failsafe) {
        LOGWARNING("[SBUS] FAILSAFE!");
    }
}

/* ========================================================== */
/*              零拷贝访问接口                                */
/* ========================================================== */

const Sbus_RC_Data_t* Sbus_GetLatestData(void)
{
    uint8_t read_idx = readable_idx;
    return (read_idx == 0) ? &rc_data_A : &rc_data_B;
}

/* ========================================================== */
/*              运行时控制发布功能                            */
/* ========================================================== */

void Sbus_EnablePublish(bool enable)
{
    publish_enabled = enable;
    if (enable) {
        LOGINFO("[SBUS] Message publish enabled.");
    } else {
        LOGINFO("[SBUS] Message publish disabled (zero-copy mode).");
    }
}

/* ========================================================== */
/*                    初始化                                  */
/* ========================================================== */

void Sbus_Init(void)
{
    memset(sbus_buf_A, 0, SBUS_FRAME_SIZE);
    memset(sbus_buf_B, 0, SBUS_FRAME_SIZE);
    memset(&rc_data_A, 0, sizeof(Sbus_RC_Data_t));
    memset(&rc_data_B, 0, sizeof(Sbus_RC_Data_t));

    writable_idx = 0;
    readable_idx = 0;
    new_frame_ready = false;

    USART_Init_Config_s config = {
        .usart_handle = &huart6,
        .recv_buff_size = SBUS_FRAME_SIZE,
        .module_callback = Sbus_Rx_Callback,
        .rx_mode = USART_RX_MODE_DOUBLE_BUF
    };
    USARTRegister(&config);

    // 注册发布者（但默认不启用）
    remote_data_pub = PubRegister("remote_data", sizeof(Sbus_RC_Data_t));

    LOGINFO("[SBUS] Init OK. Zero-copy mode (publish disabled by default).");
}