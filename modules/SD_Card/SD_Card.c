//
// Created by Administrator on 2026/2/20.
//

#include "SD_Card.h"
#include "fatfs.h"
#include "bsp_log.h" // 您的自定义 LOG 宏
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "bsp_dwt.h"

// /* ========================================================== */
// /* 核心对象与缓冲区：强制放在 RAM_D2 (Non-Cacheable) 区域         */
// /* ========================================================== */
// __attribute__((section(".dma_buffer"))) static FATFS SDFatFS;
// __attribute__((section(".dma_buffer"))) static FIL SDFile;

// 专用的日志格式化/中转缓冲区，避免频繁 Malloc 导致内存耗尽
#define SD_LOG_BUF_SIZE 512
__attribute__((section(".dma_buffer"))) static char sd_log_buf[SD_LOG_BUF_SIZE];

static bool is_file_opened = false;
static BlackboxData_t log_packet;
static char current_filename[32];
/* ========================================================== */

#include <stdio.h> // 需要用到 snprintf

// 在文件顶部声明一个全局或静态数组，用于记录当前正在写入的文件名
static char current_filename[32];

bool SD_Log_Init(void)
{
    FRESULT res;
    FILINFO fno;
    uint16_t log_num = 1;

    // 1. 挂载文件系统
    res = f_mount(&SDFatFS, "0:/", 1);
    if (res != FR_OK) {
        LOGERROR("SD Mount Failed! Code: %d", res);
        return false;
    }

    // 2. 自动寻找下一个可用的文件名 (LOG001.BIN ~ LOG999.BIN)
    for (log_num = 1; log_num <= 999; log_num++) {
        // 格式化生成待测试的文件名
        snprintf(current_filename, sizeof(current_filename), "0:/LOG%03d.BIN", log_num);

        // 使用 f_stat 检查该文件是否存在
        res = f_stat(current_filename, &fno);
        if (res == FR_NO_FILE) {
            // 文件不存在，说明这个序号是空闲的，可以使用！
            break;
        }
    }

    // 兜底保护：如果 SD 卡里已经存满了 999 个日志
    if (log_num > 999) {
        LOGERROR("SD Card log full! Overwriting LOG999.BIN");
        snprintf(current_filename, sizeof(current_filename), "0:/LOG999.BIN");
    }

    // 3. 创建这个全新的日志文件
    // 务必使用 FA_CREATE_ALWAYS：强制创建一个全新的空文件，绝不和旧数据混淆
    res = f_open(&SDFile, current_filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        LOGERROR("SD Create File [%s] Failed! Code: %d", current_filename, res);
        f_mount(NULL, "0:/", 1); // 失败则卸载
        return false;
    }

    is_file_opened = true;
    LOGINFO("SD Log Ready. Logging to: %s", current_filename);
    return true;
}

bool SD_Log_Printf(const char* format, ...)
{
    if (!is_file_opened) return false;

    va_list args;
    va_start(args, format);
    // vsnprintf 安全地将数据格式化到无 Cache 的专属缓冲区中
    int len = vsnprintf(sd_log_buf, SD_LOG_BUF_SIZE, format, args);
    va_end(args);

    if (len <= 0) return false;

    UINT bytes_written;
    FRESULT res = f_write(&SDFile, sd_log_buf, len, &bytes_written);

    if (res == FR_OK && bytes_written == len) {
        return true;
    } else {
        LOGERROR("SD Printf Write Failed! Code: %d", res);
        return false;
    }
}

bool SD_Log_Write(const void* data, uint32_t len)
{
    if (!is_file_opened || data == NULL || len == 0) return false;

    UINT bytes_written;
    FRESULT res;

    // 【极其关键的安全中转机制】
    // 外部传入的 data 指针可能位于有 Cache 的栈(局部变量)或堆区。
    // 如果数据长度小于缓冲区，我们将其拷贝到无 Cache 的 sd_log_buf 中再执行 f_write，确保 DMA 安全。
    if (len <= SD_LOG_BUF_SIZE) {
        memcpy(sd_log_buf, data, len);
        res = f_write(&SDFile, sd_log_buf, len, &bytes_written);
    } else {
        // 如果数据过大，只能直接写，但这要求调用者必须保证传入的 data 在 RAM_D2 中！
        res = f_write(&SDFile, data, len, &bytes_written);
    }

    return (res == FR_OK && bytes_written == len);
}

void SD_Log_Sync(void)
{
    // f_sync 会将缓存中的数据强制刷入物理 SD 卡并更新文件大小表。
    // 即使下一秒无人机掉电，已经 sync 过的数据也不会丢失。
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
        LOGINFO("SD Log Closed.");
    }
}

void UAV_DataLog_Task(void *argument)
{
    // 1. 系统上电稳定后，初始化并创建飞行日志文件
    osDelay(1000);
    // 后缀改叫 .BIN 或者 .LOG 都可以
    SD_Log_Init();
    uint32_t sync_timer = 0;

    for(;;)
    {
        // 1. 填充结构体数据
        log_packet.timestamp = DWT_GetTimeline_ms();
        log_packet.roll = log_packet.roll + 1.0f;
        log_packet.pitch = log_packet.pitch + 1.0f;
        log_packet.yaw = log_packet.yaw + 1.0f;
        log_packet.gyro_x =  log_packet.gyro_x + 1.0f;
        log_packet.gyro_y = log_packet.gyro_y + 2.0f;
        log_packet.gyro_z = log_packet.gyro_z + 3.0f;
        log_packet.throttle = log_packet.throttle + 10.0f;
        // ... 填充其他数据 ...

        // 2. 直接把这块内存以二进制拍进 SD 卡！极其高效！
        SD_Log_Write(&log_packet, sizeof(BlackboxData_t));
        // 3. 定期同步 (防炸机)
        // 频繁调用 f_sync 会影响性能，所以建议采用定时同步，比如每隔 500ms 强制刷入一次物理卡
        if (log_packet.timestamp - sync_timer >= 500) {
            SD_Log_Sync();
            sync_timer = log_packet.timestamp;
        }

        // 假设任务以 50Hz (20ms) 运行
        vTaskDelay(20);
    }
}