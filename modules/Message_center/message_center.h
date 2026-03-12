#ifndef UAV_BAICE_FRAMEWORK_V1_2_MESSAGE_CENTER_H
#define UAV_BAICE_FRAMEWORK_V1_2_MESSAGE_CENTER_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"

#define MAX_TOPIC_NAME_LEN  32
#define DATA_POOL_SIZE      16    // 数据载荷池大小
#define DATA_BUF_SIZE       128   // 每个载荷的最大字节数
#define ENV_POOL_SIZE       16    // 信封池大小
#define SUB_QUEUE_LEN       5     // 订阅者队列深度

// 消息信封：全静态管理
typedef struct {
    void* pData;         // 指向数据载荷池的指针
    uint16_t dataLen;    // 实际数据长度
    uint16_t refCount;   // 引用计数
    uint8_t  isFromPool; // 标记是否来自静态池
} MsgEnvelope_t;

// 订阅者拿到的通知结构
typedef struct {
    MsgEnvelope_t* pEnvelope;
} MsgNotice_t;

typedef struct mqt {
    QueueHandle_t xQueue;
    struct mqt *next_subs_queue;
} Subscriber_t;

typedef struct ent {
    char topic_name[MAX_TOPIC_NAME_LEN + 1];
    uint16_t data_len;
    Subscriber_t *first_subs;
    struct ent *next_topic_node;
} Publisher_t;

/* --- API 接口 --- */
void MsgCenter_Init(void);
Publisher_t* PubRegister(char *name, uint16_t data_len);
Subscriber_t* SubRegister(char *name, uint16_t data_len);

// 高效分发：数据入池 -> 申请信封 -> 引用计数分发
uint8_t PubPushFromPool(Publisher_t *pub, void *input_data);
// 安全回收：引用计数减一 -> 归还载荷池 -> 归还信封池 (支持中断上下文)
void SubReleaseMessage(MsgNotice_t *pNotice);
// 获取消息
uint8_t SubGetMessage(Subscriber_t *sub, MsgNotice_t *pNotice, TickType_t xWait);
// 监控内存池状态 (建议在低频心跳任务中调用)
void MsgCenter_PrintStatus(void);

uint8_t SubCopyMessage(Subscriber_t *sub, void *out_buffer, TickType_t xWait);

#endif //UAV_BAICE_FRAMEWORK_V1_2_MESSAGE_CENTER_H