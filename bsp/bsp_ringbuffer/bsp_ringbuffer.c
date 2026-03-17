//
// Created by Administrator on 2026/3/17.
//

#include "bsp_ringbuffer.h"
#include <string.h>
#include "cmsis_compiler.h" // 包含 __DMB() 指令



/**
 * @brief  初始化环形缓冲区
 * @param  rb: 环形缓冲区句柄指针
 * @param  pool: 用户提供的数据内存池数组
 * @param  size: 内存池大小 (必须是 2、4...512、1024、2048、4096...)
 */
bool RingBuffer_Init(LockFreeRingBuffer_t* rb, uint8_t* pool, uint32_t size) {
    if (rb == NULL || pool == NULL || size == 0) return false;

    // 核心安全检查：确保 size 是 2 的幂次方！否则位与运算会彻底跑飞！
    if ((size & (size - 1)) != 0) {
        return false;
    }

    rb->buffer = pool;
    rb->size = size;
    rb->mask = size - 1; // 预先算好掩码，极致压榨性能
    rb->head = 0;
    rb->tail = 0;

    memset(rb->buffer, 0, size);
    return true;
}

uint32_t RingBuffer_GetUsed(LockFreeRingBuffer_t* rb) {
    return rb->head - rb->tail;
}

uint32_t RingBuffer_GetFree(LockFreeRingBuffer_t* rb) {
    return rb->size - (rb->head - rb->tail);
}

#include <string.h>

// ---------------------------------------------------------
// 高效 Push：按块内存拷贝
// ---------------------------------------------------------
bool RingBuffer_Push(LockFreeRingBuffer_t* rb, const void* data, uint32_t len) {
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;

    // 检查剩余空间 (注意：这里的无符号减法天然处理了 head 溢出回绕的问题)
    uint32_t free_space = rb->size - (head - tail);
    if (free_space < len) return false;

    uint32_t offset = head & rb->mask;           // 当前写入的物理起始位置
    uint32_t space_to_end = rb->size - offset;   // 到达物理数组末尾的连续空间
    const uint8_t* ptr = (const uint8_t*)data;

    if (len <= space_to_end) {
        // 情况 1：剩余连续空间足够，一次性拷贝
        memcpy(rb->buffer + offset, ptr, len);
    } else {
        // 情况 2：跨越了物理边界，分两次拷贝
        memcpy(rb->buffer + offset, ptr, space_to_end);                   // 拷到末尾
        memcpy(rb->buffer, ptr + space_to_end, len - space_to_end);       // 从头开始拷剩余部分
    }

    __DMB(); // 数据内存屏障，确保 memcpy 的数据真正落入 SRAM，再更新 head

    rb->head = head + len; // 允许溢出，配合 2 的幂次方 mask 完美运行
    return true;
}

// ---------------------------------------------------------
// 高效 Pop：按块内存拷贝
// ---------------------------------------------------------
uint32_t RingBuffer_Pop(LockFreeRingBuffer_t* rb, void* out_data, uint32_t max_len) {
    uint32_t tail = rb->tail;
    uint32_t head = rb->head;

    uint32_t available = head - tail;
    if (available == 0) return 0;

    uint32_t read_len = (available > max_len) ? max_len : available;
    uint32_t offset = tail & rb->mask;
    uint32_t space_to_end = rb->size - offset;
    uint8_t* ptr = (uint8_t*)out_data;

    if (read_len <= space_to_end) {
        memcpy(ptr, rb->buffer + offset, read_len);
    } else {
        memcpy(ptr, rb->buffer + offset, space_to_end);
        memcpy(ptr + space_to_end, rb->buffer, read_len - space_to_end);
    }

    __DMB(); // 数据内存屏障，确保数据已经被读出，再释放空间

    rb->tail = tail + read_len;
    return read_len;
}