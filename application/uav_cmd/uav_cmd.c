// uav_cmd.c
#include "uav_cmd.h"
#include "message_center.h"
#include "Sbus.h"
#include "FreeRTOS.h"
#include "task.h"

// 通道映射宏定义 (假设是 AETR 顺序，请根据你的遥控器实际情况修改)
#define CH_ROLL     0
#define CH_PITCH    1
#define CH_THROTTLE 2
#define CH_YAW      3
#define CH_SW_ARM   4   // 拨杆A: 控制解锁/上锁
#define CH_SW_MODE  5   // 拨杆B: 控制飞行模式

static Subscriber_t *remote_data_sub;
static Publisher_t  *uav_cmd_pub;

static Sbus_RC_Data_t rc_data;
static Uav_Cmd_Data_t uav_cmd;

// 辅助函数：遥控器摇杆映射计算 (带死区)
// raw_val: 遥控器原始值 (例如 300~1700，中位 1000)
// out_min / out_max: 期望物理量输出范围 (例如 -30.0f ~ 30.0f 度)
static float RC_Map_With_Deadband(uint16_t raw_val, uint16_t in_min, uint16_t in_mid, uint16_t in_max,
                                  float out_min, float out_max, uint16_t deadband)
{
    float out = 0.0f;
    // 1. 死区处理 (防止摇杆在中位时轻微抖动导致飞机乱动)
    if (raw_val > (in_mid - deadband) && raw_val < (in_mid + deadband)) {
        raw_val = in_mid;
    }

    // 2. 线性映射
    if (raw_val <= in_mid) {
        // 下半段映射
        out = (float)(raw_val - in_min) / (in_mid - in_min) * (0.0f - out_min) + out_min;
    } else {
        // 上半段映射
        out = (float)(raw_val - in_mid) / (in_max - in_mid) * (out_max - 0.0f);
    }

    // 3. 边界限幅防飞车
    if (out < out_min) out = out_min;
    if (out > out_max) out = out_max;

    return out;
}

void Uav_Cmd_init()
{
    remote_data_sub = SubRegister("remote_data", sizeof(Sbus_RC_Data_t));
    uav_cmd_pub = PubRegister("uav_cmd_data", sizeof(Uav_Cmd_Data_t));

    // 初始化安全状态
    uav_cmd.arm_state = UAV_STATE_DISARMED;
    uav_cmd.flight_mode = UAV_MODE_STABILIZE;
}

void Uav_Cmd_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz 运行频率 (SBUS通常是14ms或22ms一帧)

    while (1)
    {
        // 1. 获取最新遥控器数据 (非阻塞获取最新值)
        uint8_t has_new_rc = SubCopyMessage(remote_data_sub, &rc_data, 0);

        // 2. 核心安全判断：失控保护 (Failsafe)
        // 如果接收机断电、信号丢失，或者长时间没收到新数据，必须强制上锁并归零！
        if (rc_data.failsafe || rc_data.frame_lost || !has_new_rc) {
            uav_cmd.arm_state = UAV_STATE_DISARMED;
            uav_cmd.throttle_base = 0.0f;
            uav_cmd.roll_ref = 0.0f;
            uav_cmd.pitch_ref = 0.0f;
            uav_cmd.roll_ref =  0.0f;
            uav_cmd.yaw_rate_ref = 0.0f;
            goto PUBLISH_CMD; // 直接跳到发布指令，跳过后续逻辑
        }

        // 3. 状态机：解析解锁拨杆 (假设 CH_SW_ARM 大于 1500 为解锁)
        if (rc_data.ch_raw[CH_SW_ARM] > 1500 && rc_data.ch_raw[CH_THROTTLE] < 500) {
            // 注意安全逻辑：只有当油门在最低处时，才允许解锁，防止上电直接飞走！
            uav_cmd.arm_state = UAV_STATE_ARMED;
        } else if (rc_data.ch_raw[CH_SW_ARM] < 1000) {
            uav_cmd.arm_state = UAV_STATE_DISARMED;
        }

        // 4. 状态机：解析飞行模式拨杆
        if (rc_data.ch_raw[CH_SW_MODE] < 700)       uav_cmd.flight_mode = UAV_MODE_MANUAL;
        else if (rc_data.ch_raw[CH_SW_MODE] < 1300) uav_cmd.flight_mode = UAV_MODE_STABILIZE;
        else                                        uav_cmd.flight_mode = UAV_MODE_ALTHOLD;

        // 5. 摇杆数据转换为期望物理量
        if (uav_cmd.arm_state == UAV_STATE_ARMED)
        {
            // 例如：期望横滚/俯仰最大倾角为 35 度
            uav_cmd.roll_ref  = RC_Map_With_Deadband(rc_data.ch_raw[CH_ROLL], 200, 1000, 1800, -35.0f, 35.0f, 20);
            uav_cmd.pitch_ref = RC_Map_With_Deadband(rc_data.ch_raw[CH_PITCH], 200, 1000, 1800, -35.0f, 35.0f, 20); // 注意有些遥控器俯仰是反的，需要看情况调换 out_min/max

            // 期望偏航角速度最大为 150 度/秒
            uav_cmd.yaw_ref = RC_Map_With_Deadband(rc_data.ch_raw[CH_YAW], 200, 1000, 1800, -35.0f, 35.0f, 20);

            // 油门映射 (0.0 ~ 1.0)
            // 油门没有死区，不需要回中
            uav_cmd.throttle_base = (float)(rc_data.ch_raw[CH_THROTTLE] - 200) / (1800 - 200);
            if (uav_cmd.throttle_base < 0.0f) uav_cmd.throttle_base = 0.0f;
            if (uav_cmd.throttle_base > 1.0f) uav_cmd.throttle_base = 1.0f;
        }
        else
        {
            // 上锁状态下，所有控制量清零
            uav_cmd.throttle_base = 0.0f;
            uav_cmd.roll_ref = 0.0f;
            uav_cmd.pitch_ref = 0.0f;
            uav_cmd.yaw_rate_ref = 0.0f;
        }

PUBLISH_CMD:
        // 6. 发布最终设定值给控制任务！
        PubPushFromPool(uav_cmd_pub, &uav_cmd);

        // 7. 绝对延时：保证本任务严格按照 50Hz (20ms) 运行
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}