使用说明文档 (User Manual)
简介
Monitor 模块是一个轻量级、线程安全、中断安全的软件看门狗系统。它专门用于飞控系统的硬件防掉线检测（如遥控器断联、GPS 拔出）和软件任务防死锁检测（如姿态控制任务卡死）。

核心机制
倒计时机制： 模块内部维护一个周期性递减的任务。如果外设/任务在规定时间内没有调用 Reload（喂狗），系统判定其为离线。

边沿触发（Edge-Triggered）： 报警回调函数仅仅会在设备掉线的那一瞬间触发一次，绝不会因为持续离线而疯狂占用 CPU 资源。

中断隔离： 提供了 DaemonReloadFromISR，允许你在 DMA 完成中断、串口空闲中断中直接安全喂狗。

使用步骤范例：监控 SBUS 遥控器是否失控
Step 1: 编写掉线回调函数
(注意：该函数不要使用 vTaskDelay 等阻塞操作，需尽快返回)

C
void RC_Offline_Callback(void *owner_id) {
USARTInstance *usart = (USARTInstance *)owner_id;

    // 1. 打印日志
    LOG_ERROR("CRITICAL: 遥控器失控！串口: %x", usart);
    
    // 2. 触发飞控失控保护逻辑
    Trigger_Failsafe(FAILSAFE_RC_LOST);
    
    // 3. 蜂鸣器长鸣
    Buzzer_Set_Mode(BUZZER_ALARM);
}
Step 2: 在外设初始化时注册 Daemon

C
DaemonInstance *rc_daemon;

void SBUS_Init(void) {
// ... 你的串口初始化逻辑 ...

    Daemon_Init_Config_s config = {
        .init_count = 100,      // 上电时给遥控器 100个tick 的连接时间
        .reload_count = 10,     // 正常运行时，10个tick不喂狗即视为掉线
        .callback = RC_Offline_Callback,
        .owner_id = sbus_usart_instance // 传入上下文
    };
    
    rc_daemon = DaemonRegister(&config);
}
Step 3: 在接收中断/回调中喂狗

C
void Sbus_Rx_Callback(uint8_t *buf, uint16_t len) {
// 遥控器数据成功解析！

    // 因为这里通常由 HAL 库的 DMA/UART 中断触发，所以必须用 FromISR 版本！
    DaemonReloadFromISR(rc_daemon); 
}
Step 4: 将 DaemonTask 挂载到 RTOS 的定时任务中
在你的飞控中新建一个低优先级的监控任务，或者直接放到现有的慢速任务（如 50Hz 的 Cmd_Task 或专门的 Monitor_Task）中。

C
// 假设这个任务以 50Hz (20ms) 运行
void System_Monitor_Task(void *argument) {
while(1) {
// 核心监控轮询
DaemonTask();

        // 也就是说，之前 rc_daemon 的 reload_count=10
        // 代表 10 * 20ms = 200ms 内没收到 SBUS 数据，就会触发掉线保护。
        
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}