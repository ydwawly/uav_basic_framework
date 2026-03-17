//
// Created by Administrator on 2026/3/17.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_6_BSP_RINGBUFFER_H
#define UAV_BAICE_FRAMEWORK_V1_6_BSP_RINGBUFFER_H


#include <stdint.h>
#include <stdbool.h>

// 环形缓冲区控制块 (类似于类的属性)
typedef struct {
    uint8_t* buffer;        // 指向外部实际分配的内存池
    uint32_t size;          // 缓冲区大小 (警告：必须是 2 的 n 次方！)
    uint32_t mask;          // 掩码，用于替代取余运算 (size - 1)
    volatile uint32_t head; // 生产者写入指针
    volatile uint32_t tail; // 消费者读取指针
} LockFreeRingBuffer_t;

// API 接口声明 (类似于类的方法，都需要传入句柄的指针)
bool RingBuffer_Init(LockFreeRingBuffer_t* rb, uint8_t* pool, uint32_t size);
bool RingBuffer_Push(LockFreeRingBuffer_t* rb, const void* data, uint32_t len);
uint32_t RingBuffer_Pop(LockFreeRingBuffer_t* rb, void* out_data, uint32_t max_len);
uint32_t RingBuffer_GetUsed(LockFreeRingBuffer_t* rb); // 获取当前有多少数据待读
uint32_t RingBuffer_GetFree(LockFreeRingBuffer_t* rb); // 获取剩余可用空间



#endif //UAV_BAICE_FRAMEWORK_V1_6_BSP_RINGBUFFER_H