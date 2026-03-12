//
// Created by Administrator on 2026/3/10.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_6_MONITOR_H
#define UAV_BAICE_FRAMEWORK_V1_6_MONITOR_H

#include <stdint.h>
#include "string.h"

#define DAEMON_MAX_CNT 64  // 系统最大支持的守护实例数量

typedef enum
{
    DAEMON_STATE_OFFLINE = 0,// 模块已离线/超时
    DAEMON_STATE_ONLINE = 1,// 模块在线且正常工作
}Monitor_State_t;

/* 模块离线/异常处理回调函数指针 */
typedef void (*offline_callback)(void *owner_id);

/* Monitor 初始化配置结构体 */
typedef struct
{
    uint16_t reload_count;     // 正常运行时的喂狗重载值 (代表超时阈值)
    uint16_t init_count;       // 系统刚上电或刚恢复时的初始等待时间 (给慢速设备预留启动时间)
    offline_callback callback; // 异常处理函数，仅在状态由 Online -> Offline 瞬间触发一次

    void *owner_id;            // 拥有者的上下文指针 (如 UARTInstance*, MotorInstance*)
} Monitor_Init_Config_t;

/* Daemon 实例结构体 (对外半隐藏，尽量通过指针操作) */
typedef struct daemon_ins
{
    uint16_t reload_count;
    uint16_t temp_count;
    Monitor_State_t current_state; // 内部状态机：记录当前在线状态

    offline_callback callback;
    void *owner_id;
} MonitorInstance;

/**
 * @brief 注册一个 Monitor 实例
 * @param config 初始化配置指针
 * @return MonitorInstance* 成功返回实例指针，失败返回 NULL
 */
MonitorInstance *MonitorRegister(Monitor_Init_Config_t *config);

/**
 * @brief 喂狗函数 (任务级，非中断调用)
 * @param instance Monitor 实例指针
 */
void MonitorReload(MonitorInstance *instance);

/**
 * @brief 喂狗函数 (中断级，专供 ISR 调用)
 * @note 比如在串口 RX_DMA 完成中断中调用
 * @param instance Monitor实例指针
 */
void MonitorReloadFromISR(MonitorInstance *instance);

/**
 * @brief 确认模块是否在线
 * @param instance Monitor 实例指针
 * @return 1:在线, 0:离线
 */
uint8_t MonitorIsOnline(MonitorInstance *instance);

/**
 * @brief 守护任务核心逻辑，需放入 RTOS 低优先级定时任务中周期调用
 */
void MonitorTask(void);

#endif //UAV_BAICE_FRAMEWORK_V1_6_MONITOR_H