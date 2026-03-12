//
// Created by Administrator on 2026/1/30.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_3_PT1_FILTER_H
#define UAV_BAICE_FRAMEWORK_V1_3_PT1_FILTER_H

// filter.h
typedef struct {
    float state;        // 上一次的滤波输出
    float cutoff_freq;  // 设定的截止频率，例如 90Hz
} PT1_Filter_t;

// 初始化
void PT1_Filter_Init(PT1_Filter_t *filter, float cutoff_freq);
// 应用 (注意：传入 dt)
float PT1_Filter_Apply(PT1_Filter_t *filter, float input, float dt);

#endif //UAV_BAICE_FRAMEWORK_V1_3_PT1_FILTER_H