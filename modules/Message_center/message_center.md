这份文档的结构非常清晰，非常适合直接写入你的 `message_center.md` 文件中。

为了让团队成员（或者未来的你）能够快速上手这套飞控级的发布/订阅（Pub/Sub）机制，文档分为**核心设计理念**、**快速上手（四步走）以及高级用法（零拷贝模式）**。

你可以直接复制以下 Markdown 内容作为你的使用文档：

---

# Message Center (消息中心) 使用指南

## 1. 简介

`Message Center` 是专为高频、高实时性嵌入式系统（如飞控、机器人）设计的一套基于 FreeRTOS 的进程间通信（IPC）框架。
它采用了**静态内存池**与**引用计数**机制，具备以下核心优势：

* **无内存碎片**：底层使用静态池，杜绝了长时间运行导致的堆内存碎片化。
* **ISR 安全**：发布（Publish）操作自动识别当前运行上下文，完美支持在普通任务和硬件中断（如串口 DMA 接收、定时器中断）中发布数据。
* **防拥塞机制**：针对飞控姿态控制等场景，当订阅者处理缓慢导致队列溢出时，底层会自动“挤出最旧数据，保留最新状态”，确保控制链路不卡死。
* **零拷贝潜力**：支持直接传递内存指针，适合图像或庞大矩阵等大体积载荷的高效流转。

---

## 2. 核心 API 总览

| 函数名 | 功能描述 |
| --- | --- |
| `MsgCenter_Init()` | 初始化消息中心系统，分配静态池（必须在调度器启动前调用）。 |
| `PubRegister()` | 注册/创建一个发布者（Topic）。 |
| `SubRegister()` | 注册/创建一个订阅者。 |
| `PubPushFromPool()` | 将数据拷贝到内存池，并向所有订阅者分发通知。支持在中断中调用。 |
| `SubCopyMessage()` | （推荐用法）阻塞获取消息，自动拷贝到本地缓存，并自动回收底层内存。 |
| `SubGetMessage()` / `SubReleaseMessage()` | （高级用法）获取底层数据指针实现零拷贝，使用完毕后必须手动调用释放函数。 |

---

## 3. 快速上手指南（以 IMU 数据传输为例）

在飞控系统中，经常需要将高频读取的传感器数据流转给姿态解算任务。以下是标准使用流程：

### 第一步：定义数据载荷结构体

在全局可见的头文件中定义你要传输的数据格式。

> **⚠️ 注意：** 结构体的总大小不得超过 `DATA_BUF_SIZE`（默认 128 字节）。

```c
typedef struct {
    float accel[3];
    float gyro[3];
    uint32_t timestamp;
} Sensor_IMU_t;

```

### 第二步：系统初始化与主题注册

在 `main()` 函数中，FreeRTOS 调度器启动前，初始化消息中心并注册发布者和订阅者。保存好返回的句柄供后续任务使用。

```c
// 定义全局句柄
Publisher_t  *g_PubIMU = NULL;
Subscriber_t *g_SubIMU = NULL;

void System_Init(void) {
    // 1. 初始化底层内存池
    MsgCenter_Init();
    
    // 2. 注册主题 "topic_sensor_imu"
    // 注意：发布者和订阅者的数据长度参数必须严格一致！
    g_PubIMU = PubRegister("topic_sensor_imu", sizeof(Sensor_IMU_t));
    g_SubIMU = SubRegister("topic_sensor_imu", sizeof(Sensor_IMU_t));
}

```

### 第三步：发布数据 (Publisher Task)

在你的高频任务（或外部中断服务函数）中，准备好数据并调用 `PubPushFromPool`。

```c
void Task_Sensor_Read(void *pvParameters) {
    Sensor_IMU_t imu_data;
    
    while(1) {
        // 获取硬件数据...
        imu_data.accel[0] = 9.8f;
        imu_data.timestamp = xTaskGetTickCount();
        
        // 发布数据。该函数会自动向内存池申请空间并分发给所有订阅了该主题的任务。
        PubPushFromPool(g_PubIMU, &imu_data);
        
        vTaskDelay(pdMS_TO_TICKS(2)); // 500Hz
    }
}

```

### 第四步：接收数据 (Subscriber Task)

对于绝大多数应用场景，强烈推荐使用高度集成的 `SubCopyMessage` 函数。它能确保你安全地拿到数据，并且**绝不会发生内存泄漏**。

```c
void Task_Attitude_Ctrl(void *pvParameters) {
    Sensor_IMU_t local_imu; // 本地缓存区
    
    while(1) {
        // 阻塞等待，最大超时时间设为 10ms
        // 函数成功返回 1 时，数据已安全拷贝进 local_imu 中
        if (SubCopyMessage(g_SubIMU, &local_imu, pdMS_TO_TICKS(10))) {
            
            // 进行姿态解算等业务处理
            // Run_Mahony_AHRS(&local_imu);
            
        } else {
            // 处理超时逻辑（传感器离线等）
        }
    }
}

```

---

## 4. 高级用法：Zero-Copy (零拷贝模式)

当需要传输体积极大、或者对极速处理有苛刻要求的结构体时，使用上述的 `SubCopyMessage` 会产生一次内存拷贝开销。此时可以使用原生的“三步走”接收方法：获取指针 -> 直接使用 -> 手动释放。

> **🛑 危险警告：** > 使用此方法时，无论业务逻辑执行了哪个 `return` 或 `break` 分支，**必须保证 `SubReleaseMessage` 被调用**。否则该数据块将永远不会回收到内存池中，最终导致整个消息总线瘫痪！

```c
void Task_HighPerformance_Receiver(void *pvParameters) {
    MsgNotice_t notice;
    
    while(1) {
        // 1. 获取包含指针的信封
        if (SubGetMessage(g_SubIMU, &notice, pdMS_TO_TICKS(10))) {
            
            // 2. 直接将内存池的数据指针强转为业务结构体指针使用 (零拷贝)
            Sensor_IMU_t *p_imu = (Sensor_IMU_t *)(notice.pEnvelope->pData);
            
            // 执行只读操作 (禁止修改 p_imu 里面的内容，因为其他订阅者可能也在读取)
            // Process_Data(p_imu->accel);
            
            // 3. 【绝对不能忘记】手动释放引用计数，归还信封和内存！
            SubReleaseMessage(&notice);
        }
    }
}

```

## 5. 调试与监控

系统运行中，可以通过调用 `MsgCenter_PrintStatus()` 打印当前数据池和信封池的剩余量。建议将其放置在 1Hz 的低频心跳任务中，用于监控系统是否存在潜在的内存泄漏风险。

---

如果未来你的项目中加入了摄像头或者需要直接透传超长不定长数组，你需要我为你补充一套 `PubPushPointer` (直接分发动态内存指针，不使用定长数据池) 的扩展接口文档吗？