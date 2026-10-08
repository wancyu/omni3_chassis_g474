#ifndef CHASSIS_3_H
#define CHASSIS_3_H


#include "dvc_motor_2006.h"


#define SQRT3_OVER_2 0.86602540378f

#define dt  0.001f

typedef enum {
    CHASSIS_MODE_SPEED = 0,
    CHASSIS_MODE_POSITION,
    CHASSIS_MODE_STOP,
} chassis_status_enum;

typedef struct {
    /* 几何参数 (固定物理属性) */
    float wheel_r;        // 轮子半径
    float wheel_r_inv;    // 1.0f / wheel_r
    float chassis_r;      // 底盘旋转中心半径
    float chassis_r_inv;  // 1.0f / chassis_r

    /* 底盘运行状态 */
    chassis_status_enum status;

    // float current_speed_x; // 正运动学推算出的机体当前速度 x
    // float current_speed_y; // 正运动学推算出的机体当前速度 y
    // float current_speed_w; // 正运动学推算出的机体当前速度 w

    /* ---------------- 1. 物理反馈 (传感器层写入) ---------------- */
    float world_x;              // 世界绝对坐标 X (m)
    float world_y;              // 世界绝对坐标 Y (m)
    float world_w;              // 世界绝对航向角 Yaw (rad, 闭区间 [-PI, PI])

    /* ---------------- 2. 输入指令缓存 (仅由 set 函数写入) ---------------- */
    // 速度模式下的用户意图
    float cmd_vx;               // 期望机体线速度 X (m/s)
    float cmd_vy;               // 期望机体线速度 Y (m/s)
    float cmd_vw;               // 期望机体角速度 W (rad/s)



    // 位置模式下的目标位姿
    float target_pose_x;        // 目标世界绝对坐标 X (m)
    float target_pose_y;        // 目标世界绝对坐标 Y (m)
    float target_pose_w;        // 目标世界绝对航向角 Yaw (rad)

    /* ---------------- 3. 控制器内部状态与最终执行设定值 ---------------- */
    float integrated_target_w;  // 速度模式下由数学积分器维护的目标航向 (rad)

    // 最终送入逆运动学解算的机体执行速度 (Setpoint)
    float target_speed_x;       // 最终线速度 X (m/s)
    float target_speed_y;       // 最终线速度 Y (m/s)
    float target_speed_w;       // 最终角速度 W (rad/s)


    /* 6. 外环控制器 */
    pid_struct pid_x;          // 世界 X 坐标误差 -> 期望速度
    pid_struct pid_y;          // 世界 Y 坐标误差 -> 期望速度
    pid_struct pid_w;          // 航向角误差 -> 期望自转角速度


    /* 底盘执行器硬件句柄 */
    motor_2006_struct motors[3]; // 3个动力电机


} chassis_struct;
void chassis_init(chassis_struct *chassis);
void chassis_set_target_speed(chassis_struct *chassis, float vx, float vy, float vw);
void chassis_calculate_inverse_kinematics(chassis_struct *chassis);
void chassis_update_speed_pid(chassis_struct *chassis);
void chassis_send_can_data(chassis_struct *chassis);
void chassis_stop(chassis_struct *chassis);
uint8_t chassis_process_can_feedback(chassis_struct *chassis, uint32_t id, const uint8_t *rx_data);


void chassis_set_target_pose(chassis_struct *chassis, float target_x, float target_y, float target_w);
void chassis_position_pid_update(chassis_struct *chassis);
void chassis_heading_controller_update(chassis_struct *chassis);
#endif /* CHASSIS_3_H */