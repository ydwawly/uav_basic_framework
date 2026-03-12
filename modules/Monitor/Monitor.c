//
// Created by Administrator on 2026/3/10.
//

#include "Monitor.h"

#include "bsp_log.h"
#include "FreeRTOS.h"
#include "task.h"

static MonitorInstance *monitor_Instance[DAEMON_MAX_CNT] = {NULL};
static uint8_t monitor_idx = 0;

MonitorInstance *MonitorRegister(Monitor_Init_Config_t *config)
{
    if (monitor_idx >= DAEMON_MAX_CNT)
    {
        // 你的异常拦截写得很棒
        LOGERROR("[Monitor] Monitor exceed max instance count!");
        while (1);
        return NULL;
    }

    MonitorInstance *instance = (MonitorInstance *)pvPortMalloc(sizeof(MonitorInstance));
    if (instance == NULL) {
        LOGERROR("[Monitor] Malloc failed!");
        while(1);
        return NULL;
    }

    memset(instance, 0, sizeof(MonitorInstance));

    instance->owner_id = config->owner_id;
    instance->callback = config->callback;

    // 1. 正常运行时的超时重载值
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count;

    // 2. 修复：系统刚上电时的初始倒计时，必须用 init_count！
    // 如果用户没配置 init_count，默认让它等于 reload_count
    instance->temp_count = config->init_count == 0 ? instance->reload_count : config->init_count;

    instance->current_state = DAEMON_STATE_ONLINE;

    taskENTER_CRITICAL();
    monitor_Instance[monitor_idx++] = instance;
    taskEXIT_CRITICAL();

    // 3. 修复致命 Bug：把分配好并初始化完毕的实例指针返回给调用者！
    return instance;
}

void MonitorReload(MonitorInstance *instance)
{
    if (instance == NULL) return;
    taskENTER_CRITICAL();

    instance->temp_count = instance->reload_count;

    // 状态机翻转：如果原本离线，现在重连了，状态恢复
    if (instance->current_state == DAEMON_STATE_OFFLINE) {
        instance->current_state = DAEMON_STATE_ONLINE;
        // 可选扩展：在这里调用一个 online_callback 通知系统设备已恢复
    }

    taskEXIT_CRITICAL();
}

void MonitorReloadFromISR(MonitorInstance *instance)
{
    if (instance == NULL) return;

    // 中断专用的临界区保护 API
    uint32_t isr_status = taskENTER_CRITICAL_FROM_ISR();

    instance->temp_count = instance->reload_count;

    if (instance->current_state == DAEMON_STATE_OFFLINE) {
        instance->current_state = DAEMON_STATE_ONLINE;
    }

    taskEXIT_CRITICAL_FROM_ISR(isr_status);
}

uint8_t MonitorIsOnline(MonitorInstance *instance)
{
    if (instance == NULL) return 0;
    return (instance->current_state == DAEMON_STATE_ONLINE) ? 1 : 0;
}

void MonitorTask(void)
{
    MonitorInstance *dins;

    for (size_t i = 0; i < monitor_idx; ++i)
    {
        dins = monitor_Instance[i];
        if (dins == NULL) continue;

        // 每个实例的判断使用独立的临界区，避免长时间关闭全局中断
        taskENTER_CRITICAL();

        if (dins->temp_count > 0)
        {
            dins->temp_count--; // 正常倒计时
        }
        else
        {
            // 倒计时归零，且当前仍被标记为在线，说明【刚刚】掉线！
            // 边缘触发 (Edge-Triggered)：保证回调函数只执行一次！
            if (dins->current_state == DAEMON_STATE_ONLINE)
            {
                dins->current_state = DAEMON_STATE_OFFLINE; // 更新状态，封锁回调

                // 暂时退出临界区去执行回调，防止回调函数里有阻塞操作卡死内核
                taskEXIT_CRITICAL();

                if (dins->callback) {
                    dins->callback(dins->owner_id);
                }

                taskENTER_CRITICAL(); // 重新进入临界区
            }
        }

        taskEXIT_CRITICAL();
    }
}
