//
// Created by Administrator on 2026/1/10.
//

#include "bsp_usb.h"
#include "bsp_ringbuffer.h"
#include "bsp_log.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* ========================================================== */
/*                    私有变量                                */
/* ========================================================== */

// USB 实例管理
static uint8_t usb_instance_count = 0;
static USBInstance *usb_instance[DEVICE_USB_CNT] = {NULL};

// 发送 RingBuffer（必须放在 DMA 安全区）
__attribute__((section(".dma_buffer")))
static uint8_t usb_tx_pool[USB_TX_RINGBUF_SIZE];

static LockFreeRingBuffer_t usb_tx_rb;

// 硬件状态标志（volatile，防止编译器优化）
static volatile bool usb_tx_busy = false;

// 统计信息
static uint32_t total_tx_bytes = 0;
static uint32_t dropped_tx_bytes = 0;

/* ========================================================== */
/*                    初始化函数                              */
/* ========================================================== */

/**
 * @brief  底层初始化（必须在 RTOS 启动前调用）
 */
void USB_Bsp_Init(void)
{
    // 初始化发送 RingBuffer
    if (!RingBuffer_Init(&usb_tx_rb, usb_tx_pool, USB_TX_RINGBUF_SIZE)) {
        LOGERROR("[USB] RingBuffer init failed!");
        while(1);  // 致命错误，必须停机
    }

    // 初始化硬件状态标志
    usb_tx_busy = false;

    LOGINFO("[USB] BSP Init OK. TX Buffer: %dKB", USB_TX_RINGBUF_SIZE / 1024);
}

/* ========================================================== */
/*                    注册函数                                */
/* ========================================================== */

USBInstance* USBRegister(USB_Init_Config_s *init_config)
{
    if (usb_instance_count >= DEVICE_USB_CNT) {
        LOGERROR("[USB] Exceed max instance count!");
        while(1);
    }

    // 动态分配实例（也可以用静态数组）
    USBInstance *instance = (USBInstance *)pvPortMalloc(sizeof(USBInstance));
    if (instance == NULL) {
        LOGERROR("[USB] Malloc failed!");
        while(1);
    }
    memset(instance, 0, sizeof(USBInstance));

    // 保存配置
    instance->recv_buff = init_config->recv_buff;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usb_instance[usb_instance_count++] = instance;

    return instance;
}

/* ========================================================== */
/*                    发送接口                                */
/* ========================================================== */

/**
 * @brief  异步发送（推荐接口）
 */
uint8_t USBSend_Async(uint8_t *send_buf, uint16_t send_size)
{
    // 防呆检查
    if (send_buf == NULL || send_size == 0) {
        return 1;
    }

    // 尝试推入 RingBuffer
    if (RingBuffer_Push(&usb_tx_rb, send_buf, send_size)) {
        total_tx_bytes += send_size;
        return 0;  // 成功
    } else {
        // 缓冲区满，丢弃数据
        dropped_tx_bytes += send_size;

        // 节流日志：每丢失 1KB 打印一次
        static uint32_t last_drop_log = 0;
        if (dropped_tx_bytes - last_drop_log >= 1024) {
            LOGWARNING("[USB] TX Buffer full! Dropped %lu bytes total", dropped_tx_bytes);
            last_drop_log = dropped_tx_bytes;
        }

        return 1;  // 失败
    }
}

/**
 * @brief  同步发送（不推荐，仅调试用）
 */
uint8_t USBSend(USBInstance *_instance, uint8_t *send_buf, uint16_t send_size)
{
    // 等待硬件空闲（死等，危险！）
    uint32_t timeout = 10000;
    while (usb_tx_busy && timeout--) {
        // 空转等待
    }

    if (timeout == 0) {
        LOGERROR("[USB] Sync send timeout!");
        return 1;
    }

    // 直接调用底层发送
    usb_tx_busy = true;
    uint8_t result = CDC_Transmit_FS(send_buf, send_size);

    if (result != USBD_OK) {
        usb_tx_busy = false;  // 发送失败，清除忙标志
    }

    return result;
}

/**
 * @brief  格式化打印
 */
void usb_printf(const char *format, ...)
{
    static uint8_t usb_tx_buf[256];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf((char *)usb_tx_buf, sizeof(usb_tx_buf), format, args);
    va_end(args);

    if (len > 0) {
        USBSend_Async(usb_tx_buf, (uint16_t)len);
    }
}

/* ========================================================== */
/*                    接收处理                                */
/* ========================================================== */

/**
 * @brief  USB 接收中断处理（由 usbd_cdc_if.c 调用）
 */
void USB_ReceiveHandler(uint8_t *Buf, uint32_t *Len)
{
    // 分发给所有注册的实例
    for (uint8_t i = 0; i < usb_instance_count; i++)
    {
        if (usb_instance[i] != NULL && usb_instance[i]->module_callback != NULL)
        {
            usb_instance[i]->last_recv_len = (uint16_t)(*Len);
            usb_instance[i]->module_callback(Buf, (uint16_t)(*Len));
        }
    }
}

/* ========================================================== */
/*                    发送完成中断                             */
/* ========================================================== */

/**
 * @brief  USB 发送完成中断回调（由 usbd_cdc_if.c 调用）
 */
void USB_TxCplt_Callback(void)
{
    // 清除忙标志，允许下一次发送
    usb_tx_busy = false;
}

/* ========================================================== */
/*                    发送任务                                */
/* ========================================================== */

/**
 * @brief  USB 发送任务（从 RingBuffer 取数据并发送）
 */
void USB_Tx_Task(void *pvParameters)
{
    __attribute__((section(".dma_buffer")))
    static uint8_t tx_buf[256];
    uint32_t bytes_to_send;
    uint32_t idle_count = 0;

    // 等待 USB 枚举完成
    vTaskDelay(pdMS_TO_TICKS(1000));

    LOGINFO("[USB] Tx Task started.");

    for (;;)
    {
        // ========== ① 从 RingBuffer 取数据 ==========
        bytes_to_send = RingBuffer_Pop(&usb_tx_rb, tx_buf, sizeof(tx_buf));

        if (bytes_to_send > 0)
        {
            // ========== ② 等待硬件空闲 ==========
            uint32_t timeout = 1000;  // 最大等待时间（循环次数）
            while (usb_tx_busy && timeout--) {
                vTaskDelay(pdMS_TO_TICKS(1));  // 每 1ms 检查一次
            }

            if (timeout == 0) {
                LOGERROR("[USB] Hardware stuck! Resetting busy flag.");
                usb_tx_busy = false;  // 强制复位（可能是硬件异常）
            }

            // ========== ③ 启动发送 ==========
            usb_tx_busy = true;
            uint8_t result = CDC_Transmit_FS(tx_buf, bytes_to_send);

            if (result != USBD_OK) {
                // 发送失败（可能 USB 未连接）
                usb_tx_busy = false;
                vTaskDelay(pdMS_TO_TICKS(10));  // 延迟后重试
            }

            idle_count = 0;  // 重置空闲计数
        }
        else
        {
            // ========== ④ 没有数据，休眠让出 CPU ==========
            idle_count++;

            if (idle_count < 10) {
                vTaskDelay(pdMS_TO_TICKS(1));   // 短期空闲，快速轮询
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));  // 长期空闲，降低轮询频率
            }
        }
    }
}

/* ========================================================== */
/*                    统计接口                                */
/* ========================================================== */

void USB_GetTxStats(uint32_t *total_bytes, uint32_t *dropped_bytes, float *buffer_usage)
{
    if (total_bytes) {
        *total_bytes = total_tx_bytes;
    }
    if (dropped_bytes) {
        *dropped_bytes = dropped_tx_bytes;
    }
    if (buffer_usage) {
        uint32_t used = RingBuffer_GetUsed(&usb_tx_rb);
        *buffer_usage = (float)used / USB_TX_RINGBUF_SIZE;
    }
}