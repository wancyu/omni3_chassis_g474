#ifndef CHASSIS_3_H
#define CHASSIS_3_H


#include <stdbool.h>

#include "dvc_motor_2006.h"


#define SQRT3_OVER_2 0.86602540378f

#define dt  0.001f

typedef enum {
    CHASSIS_MODE_SPEED = 0,
    CHASSIS_MODE_STOP,
} enum_chassis_status;

typedef struct {
    /* 几何参数 (固定物理属性) */
    float wheel_r;        // 轮子半径
    float wheel_r_inv;    // 1.0f / wheel_r
    float chassis_r;      // 底盘旋转中心半径
    float chassis_r_inv;  // 1.0f / chassis_r

    /* 底盘运行状态 */
    enum_chassis_status status;

    /* 目标与当前线/角速度 (注意：速度始终指【机体自身坐标系】) */
    float target_speed_x; // 前向期望速度
    float target_speed_y; // 横向期望速度
    float target_speed_w; // 自转期望角速度

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

#endif /* CHASSIS_3_H */