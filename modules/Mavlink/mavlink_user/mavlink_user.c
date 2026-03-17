#include "mavlink_user.h"
#include "bsp_usb.h"
#include "bsp_log.h"
#include "bsp_ringbuffer.h"
#include "message_center.h"
#include "BMI088driver.h"
#include <string.h>
#include "ins_task.h"
#include "common/mavlink.h"


// ========== 全局变量 ==========

// RX RingBuffer
__attribute__((section(".dma_buffer")))
static uint8_t mavlink_rx_pool[MAVLINK_RX_BUF_SIZE];
static LockFreeRingBuffer_t mavlink_rx_rb;

// USB 实例
static USBInstance *mavlink_usb_instance = NULL;

// 数据总线订阅者
static Subscriber_t *imu_data_sub = NULL;

// 统计信息
static Mavlink_Stats_t stats = {0};

// ========== 接收回调（ISR）==========

static void Mavlink_USB_RxCallback(uint8_t *data_ptr, uint16_t data_len)
{
    if (!RingBuffer_Push(&mavlink_rx_rb, data_ptr, data_len)) {
        stats.rx_buffer_drops++;
    }
}

// ========== 初始化 ==========

void Mavlink_Init(void)
{
    // 1. 初始化 RX RingBuffer
    if (!RingBuffer_Init(&mavlink_rx_rb, mavlink_rx_pool, MAVLINK_RX_BUF_SIZE)) {
        LOGERROR("[MAVLINK] RX RingBuffer init failed!");
        while(1);
    }

    // 3. 注册 USB
    USB_Init_Config_s usb_conf = {0};
    usb_conf.module_callback = Mavlink_USB_RxCallback;
    mavlink_usb_instance = USBRegister(&usb_conf);

    // 4. 订阅数据总线
    imu_data_sub = SubRegister("imu_data", sizeof(IMU_Data_t));

    LOGINFO("[MAVLINK] Init OK. RX: %dKB, TX: %dKB",
            MAVLINK_RX_BUF_SIZE/1024, MAVLINK_TX_BUF_SIZE/1024);
}

// ========== 发送函数（异步）==========

static void mavlink_send_usb(mavlink_message_t *msg)
{
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(tx_buf, msg);

    // 方案A：直接用你的异步发送（已有）
    USBSend_Async(tx_buf, len);

    stats.tx_packets++;
}

// ========== 接收处理 ==========

static void Mavlink_Process_Rx_Msg(mavlink_message_t *msg)
{
    stats.rx_packets++;

    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // 收到地面站心跳
            break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            LOGINFO("[MAVLINK] Param request");
            break;

        default:
            break;
    }
}

// ========== 主任务 ==========

void Comm_Task(void *pvParameters)
{
    Mavlink_Init();

    IMU_Data_t current_imu = {0};
    mavlink_message_t tx_msg;
    mavlink_message_t rx_msg;
    mavlink_status_t rx_status;

    uint8_t process_buf[256];
    uint16_t loop_counter = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

    while (1)
    {
        // ========== 接收处理 ==========
        uint32_t bytes_received = RingBuffer_Pop(
            &mavlink_rx_rb,
            process_buf,
            sizeof(process_buf)
        );

        if (bytes_received > 0)
        {
            for (uint32_t i = 0; i < bytes_received; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, process_buf[i], &rx_msg, &rx_status))
                {
                    Mavlink_Process_Rx_Msg(&rx_msg);
                }
            }

            // 记录错误
            stats.rx_errors += rx_status.packet_rx_drop_count;
        }

        // ========== 获取飞控数据 ==========
        SubCopyMessage(imu_data_sub, &current_imu, 0);

        // ========== 高频发送：姿态 (50Hz) ==========
        mavlink_msg_attitude_pack(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
            xTaskGetTickCount() * portTICK_PERIOD_MS,
            current_imu.Roll * DEG_TO_RAD,
            current_imu.Pitch * DEG_TO_RAD,
            current_imu.Yaw * DEG_TO_RAD,
            current_imu.Gyro[IMU_X],
            current_imu.Gyro[IMU_Y],
            current_imu.Gyro[IMU_Z]
        );
        mavlink_send_usb(&tx_msg);

        // ========== 低频发送：心跳 (1Hz) ==========
        if (loop_counter % 50 == 0)
        {
            mavlink_msg_heartbeat_pack(
                MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                MAV_TYPE_QUADROTOR,
                MAV_AUTOPILOT_GENERIC,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0,
                MAV_STATE_ACTIVE
            );
            mavlink_send_usb(&tx_msg);

            // 定期打印统计
            LOGINFO("[MAVLINK] RX: %lu pkts, %lu drops | TX: %lu pkts",
                    stats.rx_packets, stats.rx_buffer_drops, stats.tx_packets);
        }

        loop_counter++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ========== 统计接口 ==========

void Mavlink_GetStats(Mavlink_Stats_t *out)
{
    if (out) {
        memcpy(out, &stats, sizeof(Mavlink_Stats_t));
    }
}