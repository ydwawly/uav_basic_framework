//
// Created by Administrator on 2026/1/20.
//

#ifndef UAV_BAICE_FRAMEWORK_V1_2_INS_TASK_H
#define UAV_BAICE_FRAMEWORK_V1_2_INS_TASK_H

typedef struct
{
    float Gyro[3];
    float Accel[3];
    float Roll;
    float Pitch;
    float Yaw;

    // === 新增：加速度相关 ===
    float MotionAccel_b[3]; // 机体系运动加速度 (去重力后)
    float MotionAccel_n[3]; // 绝对系(导航系)运动加速度

    // === 新增：绝对系下的基向量映射 (xn, yn, zn) ===
    float xn[3]; // 机体X轴在绝对系下的指向
    float yn[3]; // 机体Y轴在绝对系下的指向
    float zn[3]; // 机体Z轴在绝对系下的指向
} IMU_Data_t;

void INS_Init(void);
/* 注意以1kHz的频率运行此任务 */
void INS_Task(void);
#endif //UAV_BAICE_FRAMEWORK_V1_2_INS_TASK_H