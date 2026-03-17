//
// Created by Administrator on 2026/2/20.
//

#include "SD_Card.h"
#include "fatfs.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "bsp_ringbuffer.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

/* ========================================================== */
/*                 全局变量与缓冲区                            */
/* ========================================================== */

// // FatFS 对象（必须在 DMA 安全区）
// __attribute__((section(".dma_buffer"))) static FATFS SDFatFS;
// __attribute__((section(".dma_buffer"))) static FIL SDFile;

// 无锁环形缓冲区实例
__attribute__((section(".dma_buffer"))) static uint8_t sd_rb_pool[LOG_RB_SIZE];
static LockFreeRingBuffer_t sd_log_rb;

// SD 卡写入缓冲区（512字节 = 1个扇区，写入效率最高）
#define SD_WRITE_CHUNK_SIZE  512
__attribute__((section(".dma_buffer"))) static uint8_t sd_write_chunk[SD_WRITE_CHUNK_SIZE];

// 状态标志
static bool is_file_opened = false;
static char current_filename[32];

// 统计信息
static volatile uint32_t total_frames = 0;
static volatile uint32_t dropped_frames = 0;

/* ========================================================== */
/*                    初始化函数                              */
/* ========================================================== */

bool SD_Log_Init(void)
{
    FRESULT res;
    FILINFO fno;
    uint16_t log_num = 1;

    // 1. 初始化无锁环形缓冲区
    if (!RingBuffer_Init(&sd_log_rb, sd_rb_pool, LOG_RB_SIZE)) {
        LOGERROR("[SD] RingBuffer init failed!");
        return false;
    }

    // 2. 挂载文件系统
    res = f_mount(&SDFatFS, "0:/", 1);
    if (res != FR_OK) {
        LOGERROR("[SD] Mount failed! Code: %d", res);
        return false;
    }

    // 3. 自动寻找下一个可用的文件名 (LOG001.BIN ~ LOG999.BIN)
    for (log_num = 1; log_num <= 999; log_num++) {
        snprintf(current_filename, sizeof(current_filename), "0:/LOG%03d.BIN", log_num);
        res = f_stat(current_filename, &fno);
        if (res == FR_NO_FILE) {
            break;  // 找到空闲编号
        }
    }

    if (log_num > 999) {
        LOGWARNING("[SD] Log full! Overwriting LOG999.BIN");
        snprintf(current_filename, sizeof(current_filename), "0:/LOG999.BIN");
    }

    // 4. 创建全新的日志文件
    res = f_open(&SDFile, current_filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        LOGERROR("[SD] Create [%s] failed! Code: %d", current_filename, res);
        f_mount(NULL, "0:/", 1);
        return false;
    }

    is_file_opened = true;
    total_frames = 0;
    dropped_frames = 0;

    LOGINFO("[SD] Logging to: %s (Buffer: %dKB)", current_filename, LOG_RB_SIZE / 1024);
    return true;
}

/* ========================================================== */
/*              生产者接口（供 INS 任务调用）                   */
/* ========================================================== */

// 假设这是在 SD_Card.c 中

bool SD_Log_PushData(const void* data, uint32_t len)
{
    if (!is_file_opened || data == NULL || len == 0) return false;

    total_frames++;

    if (RingBuffer_Push(&sd_log_rb, data, len)) {
        // 关键新增：检查缓冲区数据量
        // 如果积攒的数据 >= 一个扇区(512)，则唤醒消费者任务
        if (RingBuffer_GetUsed(&sd_log_rb) >= SD_WRITE_CHUNK_SIZE) {
            // 从中断或普通任务中发送通知
            // 注意：如果 PushData 是在中断里调用的，需使用 osThreadFlagsSet 等中断安全 API
            osThreadFlagsSet(UAV_DataLog_Task_Handle, NOTIFY_BIT_SD_WRITE_READY);
        }
        return true;
    } else {
        dropped_frames++;
        return false;
    }
}

/* ========================================================== */
/*                   内部写卡函数                             */
/* ========================================================== */

static bool SD_Log_WriteInternal(const void* data, uint32_t len)
{
    if (!is_file_opened || data == NULL || len == 0) {
        return false;
    }

    UINT bytes_written;
    FRESULT res = f_write(&SDFile, data, len, &bytes_written);

    if (res == FR_OK && bytes_written == len) {
        return true;
    } else {
        LOGERROR("[SD] Write failed! Code: %d, Wrote: %u/%u", res, bytes_written, len);
        return false;
    }
}

/* ========================================================== */
/*                   同步与关闭                               */
/* ========================================================== */

void SD_Log_Sync(void)
{
    if (is_file_opened) {
        f_sync(&SDFile);
    }
}

void SD_Log_Close(void)
{
    if (is_file_opened) {
        f_close(&SDFile);
        f_mount(NULL, "0:/", 1);
        is_file_opened = false;
        LOGINFO("[SD] Log closed. Total: %lu, Dropped: %lu", total_frames, dropped_frames);
    }
}

/* ========================================================== */
/*                   统计接口                                 */
/* ========================================================== */

void SD_Log_GetStats(uint32_t *total, uint32_t *dropped, float *usage)
{
    if (total)   *total   = total_frames;
    if (dropped) *dropped = dropped_frames;
    if (usage) {
        uint32_t used = RingBuffer_GetUsed(&sd_log_rb);
        *usage = (float)used / LOG_RB_SIZE;
    }
}

/* ========================================================== */
/*            消费者任务：SD 卡写入线程                        */
/* ========================================================== */

void UAV_DataLog_Task(void *argument)
{
    uint32_t sync_timer = DWT_GetTimeline_ms(); // 假设这是你的高精度时间

    osDelay(1000); // 等待系统挂载

    if (!SD_Log_Init()) {
        LOGERROR("[SD] Init failed! Task suspended.");
        vTaskSuspend(NULL);
        return;
    }

    for (;;)
    {
        // 1. 死等唤醒信号 (参数: 等待的标志位, 退出时清除标志, 超时时间)
        // 设置 500ms 超时，是为了顺便处理定时 f_sync 的逻辑
        uint32_t flags = osThreadFlagsWait(NOTIFY_BIT_SD_WRITE_READY,
                                           osFlagsWaitAny,
                                           500);

        // 无论是因为收到数据被唤醒，还是因为 500ms 超时被唤醒，都尝试取数据
        uint32_t bytes_popped;

        // 核心：使用 while 循环将缓冲区里所有凑够 512 字节的块一次性掏空！
        // 因为底层积攒速度极快，不能只写一次就回去睡觉。
        while ((bytes_popped = RingBuffer_Pop(&sd_log_rb, sd_write_chunk, SD_WRITE_CHUNK_SIZE)) == SD_WRITE_CHUNK_SIZE)
        {
            SD_Log_WriteInternal(sd_write_chunk, bytes_popped);
        }

        // 处理尾部不足 512 字节但需要落盘的情况（可选）
        // 如果你需要极端的实时性，可以把不够一个扇区的数据也写进去，但会降低 SD 卡寿命
        // 这里建议维持现状，让不足 512 字节的数据留在环形缓冲区里等下一次凑满。

        // 定期强制同步，防止突然断电导致文件系统损坏
        uint32_t now = DWT_GetTimeline_ms();
        if (now - sync_timer >= 500) {
            SD_Log_Sync();
            sync_timer = now;
        }
    }
}