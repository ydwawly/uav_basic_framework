//
// Created by Administrator on 2026/1/10.
//

#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdint.h>
#include <stdbool.h>

/* ========================================================== */
/*                    配置参数                                */
/* ========================================================== */

// USB 设备最大数量（通常只有 1 个）
#define DEVICE_USB_CNT  1

// 发送 RingBuffer 大小（必须是 2 的幂！）
#define USB_TX_RINGBUF_SIZE  (4 * 1024)  // 4KB

/* ========================================================== */
/*                    数据结构                                */
/* ========================================================== */

/**
 * @brief USB 初始化配置结构体
 */
typedef struct
{
    uint8_t  *recv_buff;       // 接收缓冲区（可选，通常不需要）
    uint16_t recv_buff_size;   // 接收缓冲区大小

    /**
     * @brief 接收回调函数
     * @param data: 接收到的数据指针
     * @param len:  数据长度
     * @note  运行在 USB 中断上下文，必须快速返回！
     */
    void (*module_callback)(uint8_t *data, uint16_t len);

} USB_Init_Config_s;

/**
 * @brief USB 实例结构体
 */
typedef struct
{
    uint8_t  *recv_buff;
    uint16_t recv_buff_size;
    uint16_t last_recv_len;

    void (*module_callback)(uint8_t *data, uint16_t len);

} USBInstance;

/* ========================================================== */
/*                    API 接口                                */
/* ========================================================== */

/**
 * @brief  底层初始化（创建 RingBuffer）
 * @note   必须在 osKernelStart() 之前调用！
 */
void USB_Bsp_Init(void);

/**
 * @brief  注册 USB 实例
 * @param  init_config: 初始化配置
 * @return USBInstance*: 实例指针
 */
USBInstance* USBRegister(USB_Init_Config_s *init_config);

/**
 * @brief  异步发送接口（推荐）
 * @param  send_buf: 发送数据缓冲区
 * @param  send_size: 数据长度
 * @return 0: 成功推入队列, 1: 队列满，数据丢弃
 * @note   此函数不阻塞，耗时 <2μs
 */
uint8_t USBSend_Async(uint8_t *send_buf, uint16_t send_size);

/**
 * @brief  同步发送接口（不推荐，仅调试用）
 * @param  _instance: USB 实例指针（可为 NULL）
 * @param  send_buf: 发送缓冲区
 * @param  send_size: 数据长度
 * @return USBD_OK(0) 或 USBD_FAIL
 * @warning 此函数会阻塞，不要在高频任务中调用！
 */
uint8_t USBSend(USBInstance *_instance, uint8_t *send_buf, uint16_t send_size);

/**
 * @brief  格式化打印到 USB（异步）
 * @param  format: 格式化字符串
 * @note   内部调用 USBSend_Async，不阻塞
 */
void usb_printf(const char *format, ...);

/**
 * @brief  USB 接收处理函数
 * @note   由 usbd_cdc_if.c 中的 CDC_Receive_FS 调用
 */
void USB_ReceiveHandler(uint8_t *Buf, uint32_t *Len);

/**
 * @brief  USB 发送完成中断回调
 * @note   由 usbd_cdc_if.c 中的 CDC_TransmitCplt_FS 调用
 */
void USB_TxCplt_Callback(void);

/**
 * @brief  USB 发送任务（RTOS 任务函数）
 * @param  pvParameters: 任务参数（未使用）
 */
void USB_Tx_Task(void *pvParameters);

/**
 * @brief  获取发送统计信息
 * @param  total_bytes: 总发送字节数
 * @param  dropped_bytes: 丢弃字节数（缓冲区满）
 * @param  buffer_usage: 当前缓冲区使用率（0.0~1.0）
 */
void USB_GetTxStats(uint32_t *total_bytes, uint32_t *dropped_bytes, float *buffer_usage);

#endif // BSP_USB_H