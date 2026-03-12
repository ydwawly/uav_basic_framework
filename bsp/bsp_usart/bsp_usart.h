//
// Created by Administrator on 2026/1/10.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_0_BSP_USART_H
#define UAV_BAICE_FRAMEWORK_V1_0_BSP_USART_H
#include <stdint.h>
#include "main.h"
#include "stm32h7xx_hal_uart.h"

#define DEVICE_USART_CNT 8          // MC02串口数量8个，其中
#define USART_RXBUFF_LIMIT 256      // 如果协议需要更大的buff,请修改这里

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)(uint8_t *data_ptr, uint16_t data_len);
// 接收模式枚举
typedef enum
{
    USART_RX_MODE_NORMAL = 0,     // 普通模式: 单缓冲 + IDLE中断 (有微小致盲区)
    USART_RX_MODE_DOUBLE_BUF,     // 双缓冲模式: 两个缓冲交替 (无致盲区，适合定长包)
    USART_RX_MODE_RING_BUF,       // 环形缓冲模式: 循环DMA + IDLE (无致盲区，适合不定长流数据)
} USART_RX_MODE;


/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct
{
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数

    USART_RX_MODE rx_mode;
    uint16_t recv_buff_size; // 模块接收一包数据的大小

    // 缓冲区指针
    uint8_t *recv_buff;      // 主接收缓冲区 (对外接口)
    uint8_t *recv_buff_back; // 备用缓冲区 (仅双缓冲模式用)
    uint8_t *process_buff;   // 处理缓冲区 (仅环形模式用，用于拼接数据)

    // 内部状态变量
    uint8_t *curr_buf;       // 双缓冲模式: 当前DMA正在写入的缓冲区
    uint16_t last_read_idx;  // 环形模式: 上次读取到的位置索引

}USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
    USART_RX_MODE rx_mode;                 //指定接收模式
} USART_Init_Config_s;


void* DMA_Malloc(size_t size);
/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief 启动串口服务,需要传入一个usart实例.一般用于lost callback的情况(使用串口的模块daemon)
 *
 * @param _instance
 */
void USARTServiceInit(USARTInstance *_instance);


/**
 * @brief 通过调用该函数可以发送一帧数据,需要传入一个usart实例,发送buff以及这一帧的长度
 * @note 在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 * @note 若希望连续使用DMA/IT进行发送,请配合USARTIsReady()使用,或自行为你的module实现一个发送队列和任务.
 * @todo 是否考虑为USARTInstance增加发送队列以进行连续发送?
 *
 * @param _instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size how many bytes to send
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,USART_TRANSFER_MODE mode);

/**
 * @brief 判断串口是否准备好,用于连续或异步的IT/DMA发送
 *
 * @param _instance 要判断的串口实例
 * @return uint8_t ready 1, busy 0
 */
uint8_t USARTIsReady(USARTInstance *_instance);

#endif //UAV_BAICE_FRAMEWORK_V1_0_BSP_USART_H