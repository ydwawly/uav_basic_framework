//
// Created by Administrator on 2026/1/10.
//

#include "bsp_usart.h"
#include "bsp_log.h"
#include "FreeRTOS.h"
#include "string.h" // 用于 memset, memcpy

// ==========================================================
// H743 专属：静态分配一块放在 RAM_D2 (Non-Cacheable) 的专属 DMA 内存池
// ==========================================================
// 4096 字节，可根据你的实际串口数量和单包长度需求调大
#define DMA_MEM_POOL_SIZE 4096
__attribute__((section(".dma_buffer"))) static uint8_t dma_mem_pool[DMA_MEM_POOL_SIZE];
static size_t dma_mem_offset = 0;
/**
 * @brief 专用的 DMA 内存分配器
 * @note  保证分配的内存 32 字节对齐，且绝对位于 Non-Cacheable 的 RAM_D2 中
 */
void* DMA_Malloc(size_t size)
{
    // 强制 32 字节对齐，迎合 Cache Line 长度，防止硬件 Cache 越界踩踏
    size_t align_size = (size + 31) & ~31;

    if (dma_mem_offset + align_size > DMA_MEM_POOL_SIZE)
    {
        return NULL; // 内存池耗尽
    }

    void *ptr = &dma_mem_pool[dma_mem_offset];
    dma_mem_offset += align_size;
    return ptr;
}
// ==========================================================

static uint8_t idx = 0;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};
size_t free_heap = 0;

/**
 * @brief 启动串口服务
 */
void USARTServiceInit(USARTInstance *_instance)
{
    __HAL_UART_CLEAR_FLAG(_instance->usart_handle, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    // 把寄存器里的脏数据读出来丢掉
    volatile uint32_t fake_read = _instance->usart_handle->Instance->RDR;
    (void)fake_read;

    // 根据模式启动接收
    // 注意: 如果是 RING_BUF 模式，CubeMX 中必须将 DMA 设置为 Circular 模式
    //       如果是 NORMAL/DOUBLE 模式，CubeMX 中 DMA 设置为 Normal 模式
    if (_instance->rx_mode == USART_RX_MODE_DOUBLE_BUF)
    {
        // 双缓冲模式启动时，申请了双倍内存空间
        HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size * 2);
    }
    else
    {
        // 普通模式 或 环形模式
        HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
        // 关闭DMA半传输中断，防止频繁触发
        __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief 注册串口实例
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT)
        while (1) LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i <idx; i++)
    {
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1) LOGERROR("[bsp_usart] USART instance already registered!");
    }

    // 实例结构体本身由 CPU 访问，不参与 DMA 搬运，继续用 FreeRTOS 的普通堆 (DTCM)
    USARTInstance *instance = (USARTInstance *)pvPortMalloc(sizeof(USARTInstance));
    if (instance == NULL) {
        LOGERROR("[bsp_usart] Malloc failed!");
        while(1);
    }
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    instance->rx_mode = init_config->rx_mode;

    free_heap = xPortGetFreeHeapSize();

    // 1. 所有模式都需要主缓冲区 (使用专门的 DMA_Malloc 分配到 RAM_D2)
    uint16_t alloc_size = (instance->rx_mode == USART_RX_MODE_DOUBLE_BUF) ?
                          (init_config->recv_buff_size * 2) : (init_config->recv_buff_size);

    instance->recv_buff = (uint8_t *)DMA_Malloc(alloc_size);
    if (instance->recv_buff == NULL) {
        LOGERROR("[bsp_usart] DMA Malloc failed! Pool too small.");
        while(1);
    }
    memset(instance->recv_buff, 0, alloc_size);

    // 2. 环形模式: 需要拼接用的临时 buffer (因为会涉及 memcpy，放在无 Cache 区可防踩踏)
    if (instance->rx_mode == USART_RX_MODE_RING_BUF)
    {
        instance->process_buff = (uint8_t *)DMA_Malloc(init_config->recv_buff_size);
        if (instance->process_buff == NULL) {
            LOGERROR("[bsp_usart] DMA Malloc failed!");
            while(1);
        }
        memset(instance->process_buff, 0, init_config->recv_buff_size);
        instance->last_read_idx = 0;
    }

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/**
 * @brief 发送函数
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        // 注意：如果是用 DMA 发送，传入的 send_buf 也必须位于 Non-Cacheable 区域！
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        break;
    }
}

/**
 * @brief HAL库 IDLE中断/DMA完成中断 回调
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            USARTInstance *inst = usart_instance[i];

            // ============================================================
            // 模式 1: 普通模式 (单次DMA + IDLE)
            // ============================================================
            if (inst->rx_mode == USART_RX_MODE_NORMAL)
            {
                if (inst->module_callback != NULL)
                {
                    inst->module_callback(inst->recv_buff, Size);
                }
                HAL_UARTEx_ReceiveToIdle_DMA(inst->usart_handle, inst->recv_buff, inst->recv_buff_size);
                __HAL_DMA_DISABLE_IT(inst->usart_handle->hdmarx, DMA_IT_HT);
            }
            // ============================================================
            // 模式 2: 双缓冲模式 (Ping-Pong)
            // ============================================================
            else if (inst->rx_mode == USART_RX_MODE_DOUBLE_BUF)
            {
                uint16_t half_size = inst->recv_buff_size;
                uint16_t full_size = inst->recv_buff_size * 2;

                if (huart->RxEventType == HAL_UART_RXEVENT_HT)
                {
                    if (inst->module_callback != NULL)
                        inst->module_callback(inst->recv_buff, half_size);
                }
                else if (huart->RxEventType == HAL_UART_RXEVENT_TC)
                {
                    if (inst->module_callback != NULL)
                        inst->module_callback(inst->recv_buff + half_size, half_size);
                }
                else if (huart->RxEventType == HAL_UART_RXEVENT_IDLE)
                {
                    if (Size != half_size && Size != full_size && Size != 0)
                    {
                        // 错位恢复逻辑
                        HAL_UART_AbortReceive(inst->usart_handle); // 增加中止逻辑，防止底层总线锁死
                        USARTServiceInit(inst);
                    }
                }
                return;
            }
            // ============================================================
            // 模式 3: 环形缓冲模式 (Circular DMA)
            // ============================================================
            else if (inst->rx_mode == USART_RX_MODE_RING_BUF)
            {
                uint16_t dma_cnt = __HAL_DMA_GET_COUNTER(inst->usart_handle->hdmarx);
                uint16_t current_write_idx = inst->recv_buff_size - dma_cnt;

                if (current_write_idx == inst->last_read_idx)
                {
                    return;
                }
                else
                {
                    uint16_t copy_len = 0;

                    if (current_write_idx > inst->last_read_idx)
                    {
                        copy_len = current_write_idx - inst->last_read_idx;
                        memcpy(inst->process_buff, &inst->recv_buff[inst->last_read_idx], copy_len);
                    }
                    else
                    {
                        uint16_t tail_part = inst->recv_buff_size - inst->last_read_idx;
                        uint16_t head_part = current_write_idx;

                        memcpy(inst->process_buff, &inst->recv_buff[inst->last_read_idx], tail_part);
                        memcpy(&inst->process_buff[tail_part], &inst->recv_buff[0], head_part);

                        copy_len = tail_part + head_part;
                    }

                    if (inst->module_callback != NULL)
                    {
                        inst->module_callback(inst->process_buff, copy_len);
                    }

                    inst->last_read_idx = current_write_idx;
                }
                __HAL_DMA_DISABLE_IT(inst->usart_handle->hdmarx, DMA_IT_HT);
            }
            return;
        }
    }
}

// 错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            LOGWARNING("[bsp_usart] Error Callback, Restarting...");
            // [修正] 必须先 Abort，清除 HAL 库内部的锁死状态 (BUSY_RX)
            HAL_UART_AbortReceive(huart);
            USARTServiceInit(usart_instance[i]);
            return;
        }
    }
}