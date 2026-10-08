#include "chassis_3.h"
#include <string.h>
#include "alg_pid.h"
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef SQRT3_OVER_2
#define SQRT3_OVER_2 0.8660254037844386f
#endif

/* 工具函数：将角度限制在 -PI 到 PI 之间，保证走最近的转角 */
static inline float wrap_angle_rad(float angle)
{
    while (angle > (float)M_PI)  angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/**
 * @brief 底盘初始化
 */
void chassis_init(chassis_struct *chassis)
{
    if (chassis == NULL) return;

    memset(chassis, 0, sizeof(chassis_struct));

    chassis->wheel_r       = 0.060f;        // 轮半径 60mm -> 0.060m
    chassis->wheel_r_inv   = 1.0f / chassis->wheel_r;

    chassis->chassis_r     = 0.160f;        // 中心到轮距 16cm -> 0.160m
    chassis->chassis_r_inv = 1.0f / chassis->chassis_r;

    chassis->status = CHASSIS_MODE_SPEED;

    /* ----------------- 2. 外环位置环 (输入: m/rad -> 输出: m/s, rad/s) ----------------- */
    // 平移位置外环 (X/Y)



    pid_init(&chassis->pid_x, 5.0f, 0.0f, 0.10f, 0.0f, 1.0f, 0.001f); // 最大输出 1.5 m/s
    pid_init(&chassis->pid_y, 5.0f, 0.0f, 0.10f, 0.0f, 1.0f, 0.001f); // 最大横移 1.5 m/s
    pid_init(&chassis->pid_w, 4.0f, 0.0f, 0.10f, 0.0f, 3.14f, 0.001f); // 最大自转 3.14 rad/s

    // pid_init(&chassis->pid_x, 4.00f, 0.00f, 0.20f, 0.00f, 1.50f, 0.001f);
    // pid_init(&chassis->pid_y, 4.00f, 0.00f, 0.20f, 0.00f, 1.50f, 0.001f);
    //
    // // Yaw 航向外环 (自转与锁头共用)
    // pid_init(&chassis->pid_w, 4.00f, 0.00f, 0.50f, 0.00f, 6.0f, 0.001f);

    /* ----------------- 3. 子电机与内环速度 PID 配置 ----------------- */
    const m2006_can_motor_id_enum motor_ids[3] = {
        M2006_CAN_Motor_ID_0x201,
        M2006_CAN_Motor_ID_0x202,
        M2006_CAN_Motor_ID_0x203
    };

    for (int i = 0; i < 3; i++)
    {
        motor_2006_init(&chassis->motors[i],
                        &fdcan1_manage_object,
                        motor_ids[i],
                        M2006_Control_Method_OMEGA,
                        M2006_GEARBOX_RATE,
                        10.0f);

        // 电机内环转速 PID (rad/s 误差 -> C610 控制电流)
        pid_init(&chassis->motors[i].pid_omega,
                 1600.0f,   // Kp: 响应刚度
                 300.0f,    // Ki: 克服静态摩擦
                 0.0f,      // Kd: 速度环通常给 0
                 3000.0f,   // MAX_I: 积分限幅
                 10000.0f,  // MAX_OUT: 满幅输出
                 0.001f);
    }
}


/**
 * @brief 设定速度模式的期望输入指令
 * @param chassis 底盘句柄指针
 * @param vx 机体期望线速度 X (m/s)
 * @param vy 机体期望线速度 Y (m/s)
 * @param vw 期望角速度 W (rad/s)
 */
void chassis_set_target_speed(chassis_struct *chassis, float vx, float vy, float vw)
{
    if (chassis == NULL) return;

    chassis->status          = CHASSIS_MODE_SPEED;
    chassis->cmd_vx          = vx;
    chassis->cmd_vy          = vy;
    chassis->cmd_vw          = vw;
}


/**
 * @brief 设定位置模式的目标绝对世界坐标
 * @param chassis 底盘句柄指针
 * @param target_x 目标绝对坐标 X (m)
 * @param target_y 目标绝对坐标 Y (m)
 * @param target_w 目标绝对航向角 Yaw (rad)
 */
void chassis_set_target_pose(chassis_struct *chassis, float target_x, float target_y, float target_w)
{
    if (chassis == NULL) return;

    chassis->status        = CHASSIS_MODE_POSITION;
    chassis->target_pose_x = target_x;
    chassis->target_pose_y = target_y;
    chassis->target_pose_w = wrap_angle_rad(target_w);
}

/**
 * @brief 航向角闭环控制器更新 (替代原 chassis_heading_lock_update)
 * @param chassis 底盘句柄指针
 */
void chassis_heading_controller_update(chassis_struct *chassis)
{
    if (chassis == NULL) return;

    // 平移线速度直接透传给最终执行值
    chassis->target_speed_x = chassis->cmd_vx;
    chassis->target_speed_y = chassis->cmd_vy;

    // 1. 如果未开启航向锁定（纯开环模式）
    if (1)
    {
        // 直接透传摇杆指令
        chassis->target_speed_w = chassis->cmd_vw;
        // 同步积分器，避免后续切入闭环时出现大步跃变
        chassis->integrated_target_w = chassis->world_w;
        chassis->pid_w.integral = 0.0f;
        return;
    }

    // 2. 航向锁定启用：目标积分器逻辑
    if (fabsf(chassis->cmd_vw) > 0.01f)
    {
        // 摇杆打方向：目标角在数学空间中按设定角速度步进
        chassis->integrated_target_w += chassis->cmd_vw * dt;
        chassis->integrated_target_w  = wrap_angle_rad(chassis->integrated_target_w);
    }
    // 摇杆回中时，integrated_target_w 绝对静止不变，死死咬住目标朝向

    // 3. 计算航向跟踪误差
    float err_yaw = wrap_angle_rad(chassis->integrated_target_w - chassis->world_w);

    // 4. PID 闭环计算纠偏速度
    float feedback_vw = pid_calculate_once(&chassis->pid_w, err_yaw, 0.0f);

    // 5. 前馈 (Feedforward) + 反馈 (Feedback) 复合输出
    // 转弯时叠加 cmd_vw 保证电机瞬时响应；平移回中时 cmd_vw = 0，退化为纯抗扰锁头
    chassis->target_speed_w = chassis->cmd_vw + feedback_vw;
}


/**
 * @brief 位置外环控制：将世界绝对坐标误差转换为机体期望速度
 */
void chassis_position_pid_update(chassis_struct *chassis)
{
    if (chassis == NULL) return;

    if (chassis->status != CHASSIS_MODE_POSITION) return;

    // 1. 计算世界坐标系下的绝对偏差
    float err_x_world = chassis->target_pose_x - chassis->world_x;
    float err_y_world = chassis->target_pose_y - chassis->world_y;
    float err_yaw     = wrap_angle_rad(chassis->target_pose_w - chassis->world_w);

    // 2. 坐标逆旋转：将世界大地系的误差向量，投影回机体坐标系 (消除车身旋转导致的坐标系混淆)
    float cos_yaw = cosf(chassis->world_w);
    float sin_yaw = sinf(chassis->world_w);

    // 标准 World -> Body 变换 (旋转 -yaw 角度)
    float err_x_body =  err_x_world * cos_yaw + err_y_world * sin_yaw;
    float err_y_body = -err_x_world * sin_yaw + err_y_world * cos_yaw;

    // 3. 计算 PID 速度输出
    chassis->target_speed_x = pid_calculate_once(&chassis->pid_x, err_x_body, 0.0f);
    chassis->target_speed_y = pid_calculate_once(&chassis->pid_y, err_y_body, 0.0f);
    chassis->target_speed_w = pid_calculate_once(&chassis->pid_w, err_yaw,    0.0f);
}

/**
 * @brief 底盘统一接收分发函数
 */
uint8_t chassis_process_can_feedback(chassis_struct *chassis, uint32_t id, const uint8_t *rx_data)
{
    if (chassis == NULL || rx_data == NULL) return 0;

    if (id >= 0x201 && id <= 0x203)
    {
        uint32_t index = id - 0x201;
        motor_2006_fdcan_rxcpltcallback(&chassis->motors[index], rx_data);
        return 1;
    }

    return 0;
}


/**
 * @brief 底盘逆运动学解算（标准 120° 三轮全向底盘）
 */
void chassis_calculate_inverse_kinematics(chassis_struct *chassis)
{
    if (chassis == NULL) return;

    float vx = chassis->target_speed_x;
    float vy = chassis->target_speed_y;
    float vw = chassis->target_speed_w;

    float rw_inv = chassis->wheel_r_inv;
    float r_w = vw * chassis->chassis_r;

    float target_omega[3];
    // 标准对称 120° 三轮解算公式
    target_omega[0] = ( -vy                            - r_w ) * rw_inv;
    target_omega[1] = (  0.5f * vy + SQRT3_OVER_2 * vx - r_w ) * rw_inv;
    target_omega[2] = (  0.5f * vy - SQRT3_OVER_2 * vx - r_w ) * rw_inv;

    for (int i = 0; i < 3; i++)
    {
        motor_2006_set_control_method(&chassis->motors[i], M2006_Control_Method_OMEGA);
        motor_2006_set_target_user_omega(&chassis->motors[i], target_omega[i]);
    }
}

/**
 * @brief 周期性执行电机速度 PID
 */
void chassis_update_speed_pid(chassis_struct *chassis)
{
    if (chassis == NULL) return;

    for (int i = 0; i < 3; i++)
    {
        motor_2006_tim_pid_periodelapsedcallback(&chassis->motors[i]);
    }
}

/**
 * @brief CAN 控制帧发送
 */
void chassis_send_can_data(chassis_struct *chassis)
{
    for (int i = 0; i < 3; i++)
    {
        motor_2006_send_buffer(&chassis->motors[i]);
    }

    motor_2006_group_send_data(&fdcan1_manage_object, 0x200);
}

/**
 * @brief 底盘完全失能 / 急停断电
 */
void chassis_stop(chassis_struct *chassis)
{
    if (chassis == NULL) return;

    chassis->status = CHASSIS_MODE_STOP;
    for (int i = 0; i < 3; i++)
    {
        motor_2006_set_control_method(&chassis->motors[i], M2006_Control_Method_OPENLOOP);
        motor_2006_set_out(&chassis->motors[i], 0.0f);
    }
}