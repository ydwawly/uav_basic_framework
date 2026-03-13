/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088driver.h"
#include "BMI270driver.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "fatfs.h"
#include "ff.h"
#include "i2c.h"
#include "ins_task.h"
#include "mavlink_user.h"
#include "Monitor.h"
#include "sdmmc.h"
#include "SD_Card.h"
#include "SEGGER_SYSVIEW.h"
#include "SensorHub.h"
#include "spi.h"
#include "SPL06.h"
#include "stm32h7xx_it.h"
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*
 * SD 卡读写及 Cache �??致�?�测试函�?? (基于自定�?? Log 系统)
 */
void SD_Cache_DMA_Test(void)
{
    FRESULT res;
    UINT bytesWritten, bytesRead;

    // 2. 准备测试字符�??
    const char *test_str = "Hello STM32H7! SDMMC DMA with Non-Cacheable RAM test passed!";
    uint32_t test_len = strlen(test_str);

    // 3. 使用您的 DMA_Malloc 分配读写 Buffer (确保�?? RAM_D2)
    uint8_t *write_buf = (uint8_t *)DMA_Malloc(512);
    uint8_t *read_buf  = (uint8_t *)DMA_Malloc(512);

    if (write_buf == NULL || read_buf == NULL) {
        LOGERROR("DMA_Malloc failed! Check DMA_MEM_POOL_SIZE.");
        return;
    }

    // 格式�?? Buffer 并填充要写入的数�??
    memset(write_buf, 0, 512);
    memset(read_buf, 0, 512);
    memcpy(write_buf, test_str, test_len);

    LOGINFO("--- SD Card Write/Read Test Start ---");

    // 4. 挂载 SD �??
    res = f_mount(&SDFatFS, "0:/", 1);
    if (res != FR_OK) {
        LOGERROR("f_mount error! Code: %d (Did you fix BSP_SD_IsDetected?)", res);
        return;
    }
    LOGINFO("f_mount successful.");

    // 5. 创建并写入测试文�??
    res = f_open(&SDFile, "0:/h7_test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res == FR_OK) {
        res = f_write(&SDFile, write_buf, test_len, &bytesWritten);
        f_close(&SDFile); // 必须 close 才能刷入物理 SD �??

        if (res == FR_OK && bytesWritten == test_len) {
            LOGINFO("Write successful. Wrote %d bytes.", bytesWritten);
        } else {
            LOGERROR("f_write error! Code: %d", res);
        }
    } else {
        LOGERROR("f_open (write) error! Code: %d", res);
    }

    // 6. 重新打开并读取测试文�??
    res = f_open(&SDFile, "0:/h7_test.txt", FA_READ);
    if (res == FR_OK) {
        res = f_read(&SDFile, read_buf, test_len, &bytesRead);
        f_close(&SDFile);

        if (res == FR_OK && bytesRead == test_len) {
            LOGINFO("Read successful. Read %d bytes.", bytesRead);

            // 7. 【终极校验�?�比对写入和读取的数�??
            if (memcmp(write_buf, read_buf, test_len) == 0) {
                LOGINFO("========================================");
                LOGINFO("[SUCCESS] DATA MATCHES PERFECTLY!");
                LOGINFO("Cache and DMA are working properly!");
                LOGINFO("Read Content: %s", read_buf);
                LOGINFO("========================================");
            } else {
                LOGERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                LOGERROR("[ERROR] DATA MISMATCH!");
                LOGERROR("D-Cache issue is NOT resolved!");
                LOGERROR("Expected: %s", write_buf);
                LOGERROR("Got:      %s", read_buf);
                LOGERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
        } else {
            LOGERROR("f_read error! Code: %d", res);
        }
    } else {
        LOGERROR("f_open (read) error! Code: %d", res);
    }

    // 8. 测试完毕，卸载文件系�??
    f_mount(NULL, "0:/", 1);
    LOGINFO("--- SD Card Test End ---");
}


void SD_BSP_Direct_Test(void)
{
    LOGINFO("=== BSP Layer Direct Test Start ===");

    // 1. 测试 BSP 初始�?
    LOGINFO("Testing BSP_SD_Init()...");
    if (BSP_SD_Init() != MSD_OK) {
        LOGERROR("[FAIL] BSP_SD_Init error! (Hardware/Mode Switch Failed)");
        return;
    }
    LOGINFO("[OK] BSP_SD_Init successful.");

    // 2. 测试 DMA/中断数据读取逻辑
    LOGINFO("Testing BSP_SD_ReadBlocks()...");

    // 使用您之前写好的 RAM_D2 �? Cache 内存
    uint8_t *sector_buf = (uint8_t *)DMA_Malloc(512);
    if (sector_buf == NULL) {
        LOGERROR("DMA_Malloc failed!");
        return;
    }
    memset(sector_buf, 0xAA, 512); // 填充垃圾数据用于观察

    // 尝试读取�? 0 扇区 (FATFS 挂载就是死在这一�?)
    // 参数: 缓冲区指�?, 起始扇区(0), 扇区数量(1), 超时时间(1000ms)
    uint8_t read_status = BSP_SD_ReadBlocks((uint32_t*)sector_buf, 0, 1, 1000);

    if (read_status != MSD_OK) {
        LOGERROR("[FAIL] BSP_SD_ReadBlocks error! Code: %d", read_status);
        LOGERROR("Reason: DMA transfer failed, or FreeRTOS semaphore timeout.");
    } else {
        LOGINFO("[OK] Sector 0 read successfully!");
        LOGINFO("Data[0-3]: %02X %02X %02X %02X", sector_buf[0], sector_buf[1], sector_buf[2], sector_buf[3]);
        LOGINFO("BSP layer is perfect. Issue is inside FATFS config.");
    }
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
TaskHandle_t IMU_Task_Handle;
static TaskHandle_t Control_Task_Handle;
static TaskHandle_t Comm_Task_Handle;
TaskHandle_t SensorHub_Task_Handle;
TaskHandle_t UAV_DataLog_Task_Handle;
static TaskHandle_t Monitor_Task_Handle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// static void IMU_Task(void *argument);
static void Control_Task(void *argument);
// static void Comm_Task(void *argument);
static void Monitor_Task(void *argument);
// static void SensorHub_Task(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(INS_Task,"IMU_Task",1024,NULL,osPriorityHigh2,&IMU_Task_Handle);
  xTaskCreate(Control_Task,"Control_Task",512,NULL,osPriorityHigh,&Control_Task_Handle);
  xTaskCreate(Comm_Task,"Comm_Task",512,NULL,osPriorityNormal,&Comm_Task_Handle);
  xTaskCreate(SensorHub_Task,"SensorHub_Task",1024,NULL,osPriorityHigh1,&SensorHub_Task_Handle);
    xTaskCreate(UAV_DataLog_Task,"UAV_DataLog_Task",512,NULL,osPriorityNormal,&UAV_DataLog_Task_Handle);
    xTaskCreate(Monitor_Task,"Monitor_Task",256,NULL,osPriorityLow,&Monitor_Task_Handle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    // SEGGER_SYSVIEW_Conf();  // 配置 SystemView
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
    // SEGGER_SYSVIEW_Start(); // <<<<< 放在这里！在 RTOS 启动前强制开启记录！
    // ✅ 在任务中初始化 SystemView
    // osDelay(5);                    // 可选：等待系统稳定
    // SEGGER_SYSVIEW_Stop();         // 清除残留状态
    // SEGGER_SYSVIEW_Conf();         // 配置
    // SEGGER_SYSVIEW_Start();        // 启动
    osDelay(100);

    SEGGER_SYSVIEW_Stop();
    SEGGER_SYSVIEW_Conf();
    // 连接成功后启动
    SEGGER_SYSVIEW_Start();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// float time_start, time_end;
// float time_blocking, time_dma;
// void IMU_Task(void *argument)
// {
//   /* USER CODE BEGIN IMU_Task */
//   INS_Init();
//   /* 1. �???????�??????? I2C 通信是否正常 (阻塞模式) */
//
//   // �???机静止时，获取起飞地面的基准气压
//   // 使用绝对延时，保证严格的 1kHz
//   TickType_t xLastWakeTime;
//   const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms
//   xLastWakeTime = xTaskGetTickCount();
//   /* Infinite loop */
//   for(;;)
//   {
//
//     // 1. 确保任务以准确的 1kHz 周期唤醒
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     // [调试用] 记录�????????????始时�????????????
//     time_start = DWT_GetTimeline_us();
//
//     INS_Task();
//     time_end = DWT_GetTimeline_us();
//     time_dma = time_end - time_start;
//     // vTaskDelay(1);
//     // IMU data acquisition and processing code goes here
//   }
//   /* USER CODE END IMU_Task */
// }

void Control_Task(void *argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    // Control algorithm code goes here

    osDelay(1); // Adjust delay as needed
  }
  /* USER CODE END Control_Task */
}

// static void Comm_Task(void *argument)
// {
//   /* USER CODE BEGIN Control_Task */
//   /* Infinite loop */
//   for(;;)
//   {
//     // Control algorithm code goes here
//
//     osDelay(20); // Adjust delay as needed
//   }
//   /* USER CODE END Control_Task */
// }


static void Monitor_Task(void *argument)
{
    /* USER CODE BEGIN Control_Task */
    /* Infinite loop */
    for(;;)
    {
        // Control algorithm code goes here
        MonitorTask();
        osDelay(20); // Adjust delay as needed
    }
    /* USER CODE END Control_Task */
}
/* USER CODE END Application */

