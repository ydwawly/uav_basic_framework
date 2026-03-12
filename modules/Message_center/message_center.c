#include "message_center.h"

#include "bsp_log.h"
#include "string.h"
#include "task.h"


/* --- 静态内存池定义 --- */
static uint8_t        g_DataArea[DATA_POOL_SIZE][DATA_BUF_SIZE];
static MsgEnvelope_t  g_EnvArea[ENV_POOL_SIZE];
static QueueHandle_t  xFreeDataQueue = NULL;
static QueueHandle_t  xFreeEnvQueue  = NULL;
static Publisher_t    g_TopicListRoot = { .topic_name = "Root" };

static void CheckName(char *name) {
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) > MAX_TOPIC_NAME_LEN) {
        LOGERROR("EVENT NAME TOO LONG:%s", name);
        while (1); // 飞控初始化阶段直接死机断言
    }
}

void MsgCenter_Init(void) {
    xFreeDataQueue = xQueueCreate(DATA_POOL_SIZE, sizeof(void *));
    for (int i = 0; i < DATA_POOL_SIZE; i++) {
        void *p = &g_DataArea[i][0];
        xQueueSend(xFreeDataQueue, &p, 0);
    }

    xFreeEnvQueue = xQueueCreate(ENV_POOL_SIZE, sizeof(void *));
    for (int i = 0; i < ENV_POOL_SIZE; i++) {
        void *p = &g_EnvArea[i];
        xQueueSend(xFreeEnvQueue, &p, 0);
    }
}

/* --- 安全回收机制 (兼容 ISR) --- */
void SubReleaseMessage(MsgNotice_t *pNotice) {
    if (!pNotice || !pNotice->pEnvelope) return;
    MsgEnvelope_t* env = pNotice->pEnvelope;
    uint16_t count;

    uint8_t is_isr = xPortIsInsideInterrupt();
    UBaseType_t uxSavedInterruptStatus = 0;

    // 1. 临界区保护引用计数减一
    if (is_isr) {
        uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        env->refCount--;
        count = env->refCount;
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    } else {
        taskENTER_CRITICAL();
        env->refCount--;
        count = env->refCount;
        taskEXIT_CRITICAL();
    }

    // 2. 引用归零，归还资源到队列
    if (count == 0) {
        BaseType_t xWoken = pdFALSE;

        // 归还载荷
        if (env->isFromPool && env->pData) {
            void *pData = env->pData;
            if (is_isr) xQueueSendFromISR(xFreeDataQueue, &pData, &xWoken);
            else xQueueSend(xFreeDataQueue, &pData, 0);
        }

        // 归还信封
        if (is_isr) {
            xQueueSendFromISR(xFreeEnvQueue, &env, &xWoken);
            portYIELD_FROM_ISR(xWoken);
        } else {
            xQueueSend(xFreeEnvQueue, &env, 0);
        }
    }
}

/* --- 高效分发机制 (带溢出覆盖) --- */
uint8_t PubPushFromPool(Publisher_t *pub, void *input_data) {
    if (!pub || !input_data) return 0;

    uint16_t sub_cnt = 0;
    Subscriber_t *iter = pub->first_subs;
    while (iter) { sub_cnt++; iter = iter->next_subs_queue; }
    if (sub_cnt == 0) return 0; // 无人订阅，直接返回，不消耗内存

    uint8_t is_isr = xPortIsInsideInterrupt();
    BaseType_t xWoken = pdFALSE;
    void *pPayload = NULL;
    MsgEnvelope_t* env = NULL;

    // 1. 申请载荷内存
    if (is_isr) {
        if (xQueueReceiveFromISR(xFreeDataQueue, &pPayload, &xWoken) != pdPASS) return 0;
    } else {
        if (xQueueReceive(xFreeDataQueue, &pPayload, 0) != pdPASS) return 0;
    }

    // 2. 申请信封
    if (is_isr) {
        if (xQueueReceiveFromISR(xFreeEnvQueue, &env, &xWoken) != pdPASS) env = NULL;
    } else {
        if (xQueueReceive(xFreeEnvQueue, &env, 0) != pdPASS) env = NULL;
    }

    // 申请信封失败，回退载荷内存
    if (!env) {
        if (is_isr) xQueueSendFromISR(xFreeDataQueue, &pPayload, &xWoken);
        else xQueueSend(xFreeDataQueue, &pPayload, 0);
        return 0;
    }

    // 3. 装填数据
    memcpy(pPayload, input_data, pub->data_len);
    env->pData = pPayload;
    env->dataLen = pub->data_len;
    env->refCount = sub_cnt;
    env->isFromPool = 1;

    // 4. 分发给所有订阅者
    MsgNotice_t notice = { .pEnvelope = env };
    iter = pub->first_subs;
    while (iter) {
        BaseType_t xStatus;
        if (is_isr) {
            xStatus = xQueueSendFromISR(iter->xQueue, &notice, &xWoken);
        } else {
            xStatus = xQueueSend(iter->xQueue, &notice, 0);
        }

        // [核心修改]：如果队列满了，挤出最老的数据，保证飞控姿态实时性
        if (xStatus != pdPASS) {
            MsgNotice_t old_notice;
            if (is_isr) {
                if (xQueueReceiveFromISR(iter->xQueue, &old_notice, &xWoken) == pdPASS) {
                    SubReleaseMessage(&old_notice); // 释放老数据的引用
                    xQueueSendFromISR(iter->xQueue, &notice, &xWoken); // 塞入新数据
                }
            } else {
                if (xQueueReceive(iter->xQueue, &old_notice, 0) == pdPASS) {
                    SubReleaseMessage(&old_notice);
                    xQueueSend(iter->xQueue, &notice, 0);
                }
            }
        }
        iter = iter->next_subs_queue;
    }

    if (is_isr) portYIELD_FROM_ISR(xWoken);
    return 1;
}

Publisher_t* PubRegister(char *name, uint16_t data_len) {
    if (strlen(name) > MAX_TOPIC_NAME_LEN) return NULL;
    CheckName(name);

    vTaskSuspendAll();
    Publisher_t *node = &g_TopicListRoot;
    while (node->next_topic_node) {
        node = node->next_topic_node;
        if (strcmp(node->topic_name, name) == 0) {
            if (node->data_len != data_len) {
                xTaskResumeAll(); return NULL;
            }
            xTaskResumeAll(); return node;
        }
    }

    Publisher_t *new_pub = (Publisher_t*)pvPortMalloc(sizeof(Publisher_t));
    if (new_pub) {
        memset(new_pub, 0, sizeof(Publisher_t));
        strncpy(new_pub->topic_name, name, MAX_TOPIC_NAME_LEN);
        new_pub->data_len = data_len;
        node->next_topic_node = new_pub;
    }
    xTaskResumeAll();
    return new_pub;
}

Subscriber_t* SubRegister(char *name, uint16_t data_len) {
    Publisher_t *pub = PubRegister(name, data_len);
    if (!pub) return NULL;

    vTaskSuspendAll();
    Subscriber_t *sub = (Subscriber_t*)pvPortMalloc(sizeof(Subscriber_t));
    if (sub) {
        sub->xQueue = xQueueCreate(SUB_QUEUE_LEN, sizeof(MsgNotice_t));
        sub->next_subs_queue = pub->first_subs;
        pub->first_subs = sub;
    }
    xTaskResumeAll();
    return sub;
}

uint8_t SubGetMessage(Subscriber_t *sub, MsgNotice_t *pNotice, TickType_t xWait) {
    if (!sub) return 0;
    return (xQueueReceive(sub->xQueue, pNotice, xWait) == pdTRUE);
}

void MsgCenter_PrintStatus(void) {
    if (!xFreeDataQueue || !xFreeEnvQueue) return;
    UBaseType_t data_left = uxQueueMessagesWaiting(xFreeDataQueue);
    UBaseType_t env_left  = uxQueueMessagesWaiting(xFreeEnvQueue);

    LOGINFO("[MsgCenter] Data Pool: %lu/%d, Env Pool: %lu/%d",
            (unsigned long)data_left, DATA_POOL_SIZE,
            (unsigned long)env_left, ENV_POOL_SIZE);
}

/* * 高度集成的接收接口 (推荐应用层使用)
 * 自动完成：获取信封 -> 拷贝数据到本地缓存 -> 释放信封引用
 */
uint8_t SubCopyMessage(Subscriber_t *sub, void *out_buffer, TickType_t xWait) {
    MsgNotice_t notice;

    if (!sub || !out_buffer) return 0;

    // 1. 等待并获取消息
    if (SubGetMessage(sub, &notice, xWait)) {

        // 2. 检查信封和数据指针是否合法
        if (notice.pEnvelope && notice.pEnvelope->pData) {
            // 将池中的数据拷贝到用户提供的本地缓存中
            memcpy(out_buffer, notice.pEnvelope->pData, notice.pEnvelope->dataLen);
        }

        // 3. 自动释放信封 (即使数据不合法，也要释放拿到的信封，防止内存泄漏)
        SubReleaseMessage(&notice);

        return 1; // 成功读取并拷贝
    }

    return 0; // 超时或失败
}