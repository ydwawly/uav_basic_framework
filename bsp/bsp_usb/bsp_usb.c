//
// Created by Administrator on 2026/1/10.
//
#include "bsp_usb.h"


/**
 * @file bsp_usb.c
 * @author YourName
 * @brief USB虚拟串口BSP层实现
 */

#include "bsp_usb.h"
#include "stdlib.h"
#include "memory.h"
#include "FreeRTOS.h"
#include "bsp_log.h" // 如果有日志库请开启

static uint8_t idx = 0;
static USBInstance *usb_instance[DEVICE_USB_CNT] = {NULL};

/**
 * @brief 注册USB实例
 *
 * @param init_config 初始配置
 * @return USBInstance* 实例指针
 */
USBInstance *USBRegister(USB_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USB_CNT)
        while (1); // Exceed max count

    USBInstance *instance = (USBInstance *)pvPortMalloc(sizeof(USBInstance));
    if (instance == NULL)
    {
        LOGERROR("[bsp_usb] Malloc failed!");
        while(1); // 或者做其他错误处理
    }
    memset(instance, 0, sizeof(USBInstance));

    instance->recv_buff = init_config->recv_buff;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usb_instance[idx++] = instance;
    return instance;
}

/**
 * @brief 发送数据 (包装CDC_Transmit_FS)
 *
 * @param _instance 实例指针
 * @param send_buf 发送缓冲区
 * @param send_size 发送长度
 * @return uint8_t USBD_OK if success
 */
uint8_t USBSend(USBInstance *_instance, uint8_t *send_buf, uint16_t send_size)
{
    // USB发送不需要像USART那样区分IT/DMA/BLOCKING
    // USB底层驱动会自动处理异步传输
    uint8_t status = CDC_Transmit_HS(send_buf, send_size);
    return status;
}

/**
 * @brief 处理USB接收中断的回调函数
 * @note 此函数在 usbd_cdc_if.c 中的 CDC_Receive_FS 中被调用
 */
void USB_ReceiveHandler(uint8_t *Buf, uint32_t *Len)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        // 由于USB通常只有一个接口，这里直接分发给注册过的模块
        if (usb_instance[i] != NULL && usb_instance[i]->module_callback != NULL)
        {
            // 如果用户提供了接收缓存，可以在这里进行拷贝，或者直接在回调中处理Buf
            usb_instance[i]->last_recv_len = (uint16_t)(*Len);
            usb_instance[i]->module_callback(Buf, (uint16_t)(*Len));
        }
    }
}