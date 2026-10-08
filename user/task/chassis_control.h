#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "chassis_3.h"

typedef enum {
    // --- 速度控制族 (输入期望 vx, vy, vw) ---
    CHASSIS_MODE_VOFA_SPEED = 0,       // VOFA 调参/上位机调试速度控制
    CHASSIS_MODE_RC_SPEED,         // 遥控器手控速度
    CHASSIS_MODE_LIDAR_SPEED,      // 上位机/雷达自主导航直接下发速度指令

    // --- 位置控制族 (输入目标世界/局部坐标 X, Y, Yaw) ---
    CHASSIS_MODE_VOFA_POSITION,    // VOFA 串口下发目标点位控
    CHASSIS_MODE_LIDAR_POSITION,    // 自动航路巡线/多点导航位控
} chassis_control_mode_enum;

/**
 * @brief 底盘反馈位姿数据源状态枚举
 */
typedef enum {
    POS_SOURCE_ODOM_ONLY = 0,      // 无雷达或雷达掉线，降级为纯编码器正解推算
    POS_SOURCE_LIDAR_CALIBRATING,  // 雷达在线但正在执行开机零位标定（暂不可用于位控）
    POS_SOURCE_LIDAR_GLOBAL,       // 雷达正常工作且标定完成，使用绝对世界坐标
} chassis_pos_source_enum;

// 全局控制模式与各通道速度缓存
extern volatile chassis_control_mode_enum g_chassis_ctrl_mode;
extern volatile chassis_pos_source_enum   g_chassis_pos_source;

extern chassis_struct chassis;
extern void chassis_task(void *argument);


#endif //CHASSIS_CONTROL_H