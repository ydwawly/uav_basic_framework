//
// Created by Administrator on 2026/3/3.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_5_TFMINI_PLUS_H
#define UAV_BAICE_FRAMEWORK_V1_5_TFMINI_PLUS_H

#include "stdint.h"

#define TFMINI_FRAME_SIZE 9
#define TFMINI_HEADER     0x59

typedef struct {
    uint16_t distance;    // 测量距离，单位: cm
    uint16_t strength;    // 信号强度
    float temperature;    // 芯片温度，单位: 摄氏度
    uint8_t is_valid;     // 数据有效性标志位 (1:有效, 0:无效)
} TFminiPlus_Data_t;

void TFmini_Init(void);
void TFmini_Task_Handler(void);


#endif //UAV_BAICE_FRAMEWORK_V1_5_TFMINI_PLUS_H