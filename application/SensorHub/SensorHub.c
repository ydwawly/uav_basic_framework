//
// Created by Administrator on 2026/2/20.
//

#include "SensorHub.h"

#include <stdbool.h>

#include "BMI088driver.h"
#include "BMI270driver.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "i2c.h"
#include "QMC5883L.h"
#include "Sbus.h"
#include "spi.h"
#include "SPL06.h"
#include "TFmini_Plus.h"
#include "Remote/ibus/ibus.h"
// 引入两个状态标志
static bool spl06_pending = false;
static bool qmc_pending = false;
static bool i2c_bus_busy = false;
static uint8_t last_device = 0; // 0=无, 1=SPL06, 2=QMC5883L
void SensorHub_Init(void) {
    // 创建高优先级任务，确保传感器数据能被即时处理
    BMI088_Init(&hspi2);
    SPL06_Init_DMA(&hi2c2);
    BMI270_Driver_Init(&hspi3);
    // TFmini_Init();
    // Sbus_Init();
    // Ibus_Init();
    QMC5883L_Init();
}

void SensorHub_Task(void *pvParameters)
{
    uint32_t notifyBits = 0;
    static uint32_t qmc_timer = 0;


    // --- 性能分析专用变量 ---
    static  uint64_t start_time_us = 0;
    static  uint32_t cost_time_us = 0;
    static uint32_t max_cost_time_us = 0; // 记录历史最大耗时 (飞控最关心这个数据)

    while (1)
    {
        /*
               * xTaskNotifyWait 参数说明：
               *   ulBitsToClearOnEntry = 0          : 进入等待前不清任何 bit
               *   ulBitsToClearOnExit  = 0xFFFFFFFF : 读取后清除所有 bit
               *   pulNotificationValue = &notifyBits: 拿到所有置位的 bit
               *   xTicksToWait         = 20ms       : 超时（兜底轮询 QMC5883L）
               */
        xTaskNotifyWait(0x00,0xFFFFFFFF,&notifyBits,pdMS_TO_TICKS(20)); // 等待最多20ms，确保能及时响应
        // 1. 任务醒来，立刻按下“秒表”开始计时
        start_time_us = DWT_GetTimeline_us();

        // ==================== 原有的处理逻辑 START ====================
        // IMU 最高优先级，1kHz 心跳
        if (notifyBits & NOTIFY_BIT_BMI088)
        {
            BMI088_DMA_Callback();
        }

        if(notifyBits & NOTIFY_BIT_BMI270)
        {
            BMI270_DMA_Callback();
        }

        // --- 2. SBUS 遥控器处理 (UART) ---
        if (notifyBits & NOTIFY_BIT_REMOTE)
        {
            Sbus_Task_Handler(); // 内部直接解析记录好的指针
            // Ibus_Task_Handler();
        }

        if (notifyBits & NOTIFY_BIT_TFMINI)
        {
            TFmini_Task_Handler();
        }
        if (notifyBits & NOTIFY_BIT_SPL06_TRIG)
        {
            // SPL06 触发中断，准备读取
            spl06_pending = true;
        }

        if (DWT_GetTimeline_ms() - qmc_timer >= 100) // 每100ms发起一次 QMC5883L 读取
        {
            qmc_timer = DWT_GetTimeline_ms();
            qmc_pending = true;
        }

        if (notifyBits & NOTIFY_BIT_I2C_DMA_DONE)
        {
            i2c_bus_busy = false;
            // 根据当前完成的传感器处理数据
            // 这里可以增加逻辑判断刚才是谁在读，或者简单地在 Callback 里处理完
            // 建议：在 Task 里解析数据，保持中断简洁
            if (last_device == DEVICE_SPL06) SPL06_Data_Handler();
            else if (last_device == DEVICE_QMC) QMC5883L_DMA_Callback();
        }

        if (!i2c_bus_busy)
        {
            if (spl06_pending)
            {
                spl06_pending = false;
                i2c_bus_busy = true;
                last_device = DEVICE_SPL06;
                SPL06_Read_DMA_Start();
            }
            else if (qmc_pending)
            {
                qmc_pending = false;
                i2c_bus_busy = true;
                last_device = DEVICE_QMC;
                QMC5883L_Read_DMA_Start();
            }
        }

        // ==================== 原有的处理逻辑 END ====================

        // 2. 逻辑执行完毕，准备回去睡觉前，再次看表
        cost_time_us = (uint32_t)(DWT_GetTimeline_us() - start_time_us);

        // 3. 捕捉“最坏情况” (WCET)
        if (cost_time_us > max_cost_time_us)
        {
            max_cost_time_us = cost_time_us;
        }

        // 4. 超时报警拦截 (假设我们规定 SensorHub_Task 的处理绝对不能超过 500us)
        // 如果 BMI088 是 1kHz 运行的，那留给这个任务的时间绝对不能超过 1ms (1000us)
        if (cost_time_us > 500)
        {
            // 可以在这里打断点，或者用串口打印出来看看是谁拖慢了系统
            LOGWARNING("SensorHub Overrun: %d us!", cost_time_us);
        }
    }
}