//
// Created by Administrator on 2026/3/6.
//

#include "usart.h"
#include "usart_test.h"
#include "bsp_usart.h"
#include "bsp_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

// 【注意】请将这里的 huart1 替换为你实际连接 PC 的串口句柄 (比如 huart6)
// 串口实例指针
static USARTInstance *pc_uart_instance = NULL;

// ==========================================================
// 核心细节：发送缓冲区也必须放在 Non-Cacheable 区域！
// 如果用局部变量(在栈上)或普通全局变量(在RAM_D1)，用 DMA 发送时 PC 会收到乱码
// ==========================================================
#define PC_TX_BUFF_SIZE 256
__attribute__((section(".dma_buffer"))) static uint8_t pc_tx_buffer[PC_TX_BUFF_SIZE];

/**
 * @brief 接收回调函数 (由 bsp_usart 底层在中断里触发)
 */
static void PC_RxCallback(uint8_t *data_ptr, uint16_t data_len)
{
    // 安全检查，防止溢出
    if (data_len > PC_TX_BUFF_SIZE)
    {
        data_len = PC_TX_BUFF_SIZE;
    }

    // 将收到的数据拷贝到我们的专用无 Cache 发送缓冲区中
    memcpy(pc_tx_buffer, data_ptr, data_len);

    // 触发回显 (Echo) - 使用 DMA 发送
    USARTSend(pc_uart_instance, pc_tx_buffer, data_len, USART_TRANSFER_DMA);
}

/**
 * @brief 心跳发送任务
 */
static void PcTestTask(void *argument)
{
    uint32_t run_time = 0;

    // 给系统一点启动时间
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1)
    {
        // 格式化字符串到无 Cache 的发送缓冲区中
        snprintf((char *)pc_tx_buffer, PC_TX_BUFF_SIZE, "[STM32H743] System Running... Time: %lu s\r\n", run_time++);

        // 使用 DMA 发送心跳包
        USARTSend(pc_uart_instance, pc_tx_buffer, strlen((char *)pc_tx_buffer), USART_TRANSFER_DMA);

        // 阻塞 1000ms
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief 模块初始化
 */
void AppPcTest_Init(void)
{
    // 1. 配置并注册串口
    USART_Init_Config_s config;
    config.usart_handle    = &huart1;                    // 绑定底层 HAL 句柄
    config.recv_buff_size  = 128;                        // 环形缓冲单次处理的最大包长
    config.module_callback = PC_RxCallback;              // 绑定解析回调
    config.rx_mode         = USART_RX_MODE_RING_BUF;     // 环形缓冲模式，最适合 PC 调试

    pc_uart_instance = USARTRegister(&config);

    if (pc_uart_instance == NULL)
    {
        LOGERROR("[AppPcTest] USART Register Failed!");
        return;
    }

    // 2. 创建 RTOS 发送任务 (优先级可以设低一点，比如 2)
    xTaskCreate(PcTestTask, "PcTestTask", 256, NULL, 2, NULL);

    LOGINFO("[AppPcTest] Initialized successfully.");
}