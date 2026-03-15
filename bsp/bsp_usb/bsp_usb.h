//
// Created by Administrator on 2026/1/10.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_0_BSP_USB_H
#define UAV_BAICE_FRAMEWORK_V1_0_BSP_USB_H
#include "stdint.h"
#include "usbd_cdc_if.h"

#define DEVICE_USB_CNT 1 // 通常只有一个USB从机接口

typedef struct {
    uint8_t *recv_buff;          // 接收缓冲区指针
    uint16_t recv_buff_size;     // 期望接收长度
    void (*module_callback)(uint8_t *buf, uint16_t len); // 接收回调, 传入数据首地址和长度
} USB_Init_Config_s;

typedef struct {
    uint8_t *recv_buff;
    uint16_t recv_buff_size;
    uint16_t last_recv_len;      // 上次接收到的数据长度
    void (*module_callback)(uint8_t *buf, uint16_t len);
} USBInstance;

/* 导出函数 */
USBInstance *USBRegister(USB_Init_Config_s *init_config);
uint8_t USBSend(USBInstance *_instance, uint8_t *send_buf, uint16_t send_size);

/* 供usbd_cdc_if.c调用的底层接口 */
void USB_ReceiveHandler(uint8_t *Buf, uint32_t *Len);
void USB_Bsp_Init(void);

uint8_t USBSend_Async(uint8_t *send_buf, uint16_t send_size);

/**
 * @brief 改造后的异步 usb_printf
 */
void usb_printf(const char *format, ...);

void USB_Tx_Task(void *pvParameters);

#endif //UAV_BAICE_FRAMEWORK_V1_0_BSP_USB_H