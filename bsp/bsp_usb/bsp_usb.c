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
#include "semphr.h"
#include "stream_buffer.h"

static uint8_t idx = 0;
static USBInstance *usb_instance[DEVICE_USB_CNT] = {NULL};
// 定义全避变量
StreamBufferHandle_t mavlink_tx_stream = NULL;
SemaphoreHandle_t usb_tx_cplt_sem = NULL;
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <stdio.h>
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
    uint8_t status = CDC_Transmit_FS(send_buf, send_size);
    return status;
}
/**
 * @brief USB 通信底层内核对象初始化
 * @note  必须在 osKernelStart() 之前调用！
 */
void USB_Bsp_Init(void)
{
    // 1. 创建流缓冲区 (出餐台)
    if (mavlink_tx_stream == NULL) {
        mavlink_tx_stream = xStreamBufferCreate(2048, 1);
    }

    // 2. 创建二值信号量 (绿灯)
    if (usb_tx_cplt_sem == NULL) {
        usb_tx_cplt_sem = xSemaphoreCreateBinary();

        // 🚨 极其重要：信号量创建后默认是“红灯”
        // 必须手动 Give 一次，让第一包数据能发出去
        xSemaphoreGive(usb_tx_cplt_sem);
    }

    if (mavlink_tx_stream == NULL || usb_tx_cplt_sem == NULL) {
        LOGERROR("[USB] OS Object Create Failed!");
        while(1); // 这种底层错误必须原地卡死，不能带病起飞
    }
}

/**
 * @brief 异步发送 API (所有任务通用的安全入口)
 */
/* bsp_usb.c */

uint8_t USBSend_Async(uint8_t *send_buf, uint16_t send_size)
{
    // 防呆：如果缓冲区还没建好，直接退回，不准踩踏内存
    if (mavlink_tx_stream == NULL) {
        return 1;
    }

    // 尝试往流缓冲区塞数据
    // 这里的 0 代表：如果万一缓冲区满了（比如 USB 没插），
    // 逻辑是“宁可丢包，也绝不卡死控制任务”
    size_t xBytesSent = xStreamBufferSend(mavlink_tx_stream, send_buf, send_size, 0);

    return (xBytesSent == send_size) ? 0 : 1;
}

/**
 * @brief 改造后的异步 usb_printf
 */
void usb_printf(const char *format, ...)
{
    static uint8_t usb_tx_buf[128]; // 减小一点，省栈空间
    va_list args;
    uint16_t len;

    va_start(args, format);
    len = vsnprintf((char *)usb_tx_buf, sizeof(usb_tx_buf), format, args);
    va_end(args);

    // 发送给异步队列，而不是直接操作硬件
    USBSend_Async(usb_tx_buf, len);
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

void USB_Tx_Task(void *pvParameters)
{
    // 初始化已经在外面做过了
    uint8_t tx_buf[256];
    size_t bytes_to_send;

    for (;;)
    {
        // 1. 等水吃
        bytes_to_send = xStreamBufferReceive(mavlink_tx_stream, tx_buf, sizeof(tx_buf), portMAX_DELAY);

        if (bytes_to_send > 0)
        {
            // 2. 拿绿灯 (等待硬件空闲)
            if (xSemaphoreTake(usb_tx_cplt_sem, portMAX_DELAY) == pdTRUE)
            {
                // 3. 这里的数据在 RAM_D2 (MPU保护)，直接发
                uint8_t res = CDC_Transmit_FS(tx_buf, bytes_to_send);

                if (res != USBD_OK) {
                    // 万一底层报错（比如掉线），要把灯还回去，否则任务永远死锁
                    xSemaphoreGive(usb_tx_cplt_sem);
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
        }
    }
}

// 接收部分建议保持现在的逻辑，但提醒 Mavlink 回调内部要快！