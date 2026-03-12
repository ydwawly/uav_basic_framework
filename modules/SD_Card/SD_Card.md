#include "bsp_sd_log.h"
#include "cmsis_os.h" // 包含 FreeRTOS 延迟等

void UAV_DataLog_Task(void *argument)
{
// 1. 系统上电稳定后，初始化并创建飞行日志文件
osDelay(1000);
SD_Log_Init("0:/FLIGHT_LOG.CSV");

    // 写入 CSV 表头
    SD_Log_Printf("Time(ms), Roll, Pitch, Yaw, Throttle\n");
    SD_Log_Sync(); // 写完表头立刻保存

    uint32_t sync_timer = 0;

    for(;;)
    {
        // 假设这里获取到了当前的姿态数据和时间戳
        uint32_t current_time = HAL_GetTick();
        float roll = 10.5f, pitch = -2.3f, yaw = 45.0f; // 示例数据
        uint16_t throttle = 1500;

        // 2. 高频写入格式化数据
        SD_Log_Printf("%lu, %.2f, %.2f, %.2f, %d\n", current_time, roll, pitch, yaw, throttle);

        // 3. 定期同步 (防炸机)
        // 频繁调用 f_sync 会影响性能，所以建议采用定时同步，比如每隔 500ms 强制刷入一次物理卡
        if (current_time - sync_timer >= 500) {
            SD_Log_Sync();
            sync_timer = current_time;
        }

        // 假设任务以 50Hz (20ms) 运行
        osDelay(20); 
    }
}