#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "chassis_3.h"




typedef enum {
    CHASSIS_MODE_VOFA = 0,   // 0: VOFA 串口调试控制
    CHASSIS_MODE_RC   = 1,   // 1: 遥控器手控
    CHASSIS_MODE_AUTO = 2,   // 2: 串口2 雷达 / 上位机自主导航
} chassis_control_mode_enum;

// 全局控制模式与各通道速度缓存
extern volatile chassis_control_mode_enum g_chassis_ctrl_mode;





extern chassis_struct chassis;
extern void chassis_task(void *argument);


#endif //CHASSIS_CONTROL_H