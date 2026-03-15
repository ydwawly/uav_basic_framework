//
// Created by Administrator on 2026/3/13.
//

#include "bsp_usb.h"
#include "bsp_log.h"
#include "message_center.h"
#include "ins_task.h"
#include "uav_cmd.h"
#include "common/mavlink.h"
#include "stream_buffer.h"
#include "BMI088driver.h"
// FreeRTOS 组件
#include "queue.h"
#include "bsp_usb.h"
#include "mavlink_user.h"


// ==========================================
// 模块私有变量
// ==========================================
static USBInstance *mavlink_usb_instance = NULL;
static QueueHandle_t mavlink_rx_queue = NULL;

// 数据总线订阅者指针
static Subscriber_t *imu_data_sub = NULL;
static StreamBufferHandle_t mavlink_rx_stream = NULL;
// ==========================================
// 1. 接收回调 (运行在底层 USB 中断/回调上下文中)
// ==========================================
static void Mavlink_USB_RxCallback(uint8_t *data_ptr, uint16_t data_len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 工业级做法：ISR 里绝对不做协议解析！
    // 只做一件事：把底层收到的一大坨原始数据，原封不动地砸进 StreamBuffer
    if (mavlink_rx_stream != NULL)
    {
        // 瞬间完成内存拷贝，立刻放行 CPU
        xStreamBufferSendFromISR(mavlink_rx_stream, data_ptr, data_len, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ==========================================
// 2. 初始化函数
// ==========================================
void Mavlink_Init(void)
{
    // 1. 初始化 MAVLink 接收队列 (深度为10，足以应对地面站突发的指令)
    mavlink_rx_stream = xStreamBufferCreate(2048, 1);

    // 2. 注册 USB 驱动，挂载接收回调
    USB_Init_Config_s usb_conf;
    memset(&usb_conf, 0, sizeof(usb_conf));
    usb_conf.module_callback = Mavlink_USB_RxCallback;
    // recv_buff 如果在 bsp_usb 里不需要显式分配，可以填 NULL；如果需要，请分配静态数组
    mavlink_usb_instance = USBRegister(&usb_conf);

    // 3. 注册需要用到的总线订阅/发布
    imu_data_sub = SubRegister("imu_data", sizeof(IMU_Data_t));

    LOGINFO("[MAVLINK] Init Success. USB Linked.");
}

// ==========================================
// 3. 发送辅助函数 (改造为纯异步投递)
// ==========================================
static void mavlink_send_usb(mavlink_message_t *msg)
{
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];

    // 将结构体序列化为最终的字节流
    uint16_t len = mavlink_msg_to_send_buffer(tx_buf, msg);

    // 【核心改造】：不再调用阻塞的 USBSend，而是调用异步的 USBSend_Async
    // 直接把打包好的字节流扔进 StreamBuffer (出餐台)，瞬间返回！
    USBSend_Async(tx_buf, len);
}

// ==========================================
// 4. 接收指令处理逻辑 (由 Comm_Task 调用)
// ==========================================
static void Mavlink_Process_Rx_Msg(mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // 收到地面站的心跳包 (通常不用管，除非你要做双向失控保护)
            break;

        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            // 收到地面站的虚拟摇杆数据 (未来可实现电脑控制飞机)
            // mavlink_rc_channels_override_t rc_override;
            // mavlink_msg_rc_channels_override_decode(msg, &rc_override);
            break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            // 地面站请求读取所有参数 (PID调参必备，后续开发)
            LOGINFO("[MAVLINK] GCS requested parameter list.");
            break;

        default:
            // 其他未处理的消息包
            break;
    }
}

// ==========================================
// 5. 核心任务：Comm_Task (50Hz)
// ==========================================
void Comm_Task(void *pvParameters)
{
    // 任务启动前进行初始化
    Mavlink_Init();

    IMU_Data_t current_imu = {0};
    mavlink_message_t tx_msg;
    mavlink_message_t rx_msg;
    mavlink_status_t rx_status;
    uint8_t process_buf[128];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

    uint16_t loop_counter = 0;

    while (1)
    {
        // ----------------------------------------------------
        // Step 1: 处理所有来自地面站的指令 (非阻塞提取)
        // ----------------------------------------------------
        size_t bytes_received;

        // 【优化点】将读取函数直接放入 while 条件中
        // 逻辑：尝试读取 -> 如果读取到的字节数 > 0 -> 进入循环处理 -> 再次回到条件尝试读取
        // 如果一上来就没数据 (返回0)，直接跳过循环，0 阻塞！
        while ((bytes_received = xStreamBufferReceive(mavlink_rx_stream, process_buf, sizeof(process_buf), 0)) > 0)
        {
            // 慢慢在 Task 里用状态机吃字节，就算执行很久，也不会影响高优先级的 IMU 任务！
            for (size_t i = 0; i < bytes_received; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, process_buf[i], &rx_msg, &rx_status))
                {
                    Mavlink_Process_Rx_Msg(&rx_msg);
                }
            }
        }
        // ----------------------------------------------------
        // Step 2: 获取飞控内部最新状态
        // ----------------------------------------------------
        SubCopyMessage(imu_data_sub, &current_imu, 0);

        // ----------------------------------------------------
        // Step 3: 高频发送区 (50Hz) - ATTITUDE 姿态
        // ----------------------------------------------------
        mavlink_msg_attitude_pack(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
            xTaskGetTickCount() * portTICK_PERIOD_MS, // 时间戳(ms)
            current_imu.Roll * DEG_TO_RAD, current_imu.Pitch * DEG_TO_RAD, current_imu.Yaw * DEG_TO_RAD, // 必须是弧度 rad
            current_imu.Gyro[IMU_X], current_imu.Gyro[IMU_Y], current_imu.Gyro[IMU_Z] // 必须是弧度每秒 rad/s
        );
        mavlink_send_usb(&tx_msg);

        // ----------------------------------------------------
        // Step 4: 低频发送区 (1Hz) - HEARTBEAT 心跳
        // ----------------------------------------------------
        if (loop_counter % 50 == 0) // 每50次进入一次 = 1Hz
        {
            mavlink_msg_heartbeat_pack(
                MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                MAV_TYPE_QUADROTOR,          // 飞行器类型
                MAV_AUTOPILOT_GENERIC,       // 飞控类型
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0,                           // 自定义模式
                MAV_STATE_ACTIVE             // 系统状态
            );
            mavlink_send_usb(&tx_msg);
        }

        loop_counter++;

        // ----------------------------------------------------
        // Step 5: 严格时间休眠
        // ----------------------------------------------------
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}