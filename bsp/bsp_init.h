//
// Created by ydwzxcv on 2025/9/25.
//

#ifndef MY_DM_BASIC_FRAMEWORK_BSP_INIT_H
#define MY_DM_BASIC_FRAMEWORK_BSP_INIT_H
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "message_center.h"
#include "Sbus.h"

/**
 * @brief bsp层初始化统一入口,这里仅初始化必须的bsp组件,其他组件的初始化在各自的模块中进行
 *        需在实时系统启动前调用,目前由RobotoInit()调用
 *
 * @note 其他实例型的外设如CAN和串口会在注册实例的时候自动初始化,不注册不初始化
  */
//
void BSPInit()
{
    DWT_Init(240);
    BSPLogInit();
    MsgCenter_Init();

}
#endif //MY_DM_BASIC_FRAMEWORK_BSP_INIT_H
