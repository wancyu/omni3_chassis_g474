#include "chassis_3.h"
#include <string.h>
#include "alg_pid.h"
#include "math.h"

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



    // 在底盘内统一完成子电机初始化
    const m2006_can_motor_id_enum motor_ids[3] = {
        M2006_CAN_Motor_ID_0x201,
        M2006_CAN_Motor_ID_0x202,
        M2006_CAN_Motor_ID_0x203
    };

    for (int i = 0; i < 3; i++)
    {
        // 绑定所属总线与 ID，并在内部挂接好发送缓冲区切片
        motor_2006_init(&chassis->motors[i], &fdcan1_manage_object, motor_ids[i],M2006_Control_Method_OMEGA,M2006_GEARBOX_RATE, 10.0f);

        // 统一配置电机内环（速度环）PID 参数
        pid_init(&chassis->motors[i].pid_omega, 1200.0f, 500.0f, 0.0f, 500.0f, 10000.0f,0.001f);

    }
}

/**
* @brief 底盘统一接收分发函数
 * @param chassis 底盘对象指针
 * @param id CAN 标准 ID (0x201 ~ 0x208)
 * @param rx_data 8 字节反馈数据指针
 * @return 1 匹配成功且处理; 0 非本底盘电机报文
 */
uint8_t chassis_process_can_feedback(chassis_struct *chassis, uint32_t id, const uint8_t *rx_data)
{
    if (chassis == NULL || rx_data == NULL) return 0;

    // 3 个电机 ID 分别是 0x201, 0x202, 0x203
    if (id >= 0x201 && id <= 0x203)
    {
        uint32_t index = id - 0x201; // 0x201 -> 0, 0x202 -> 1, 0x203 -> 2
        motor_2006_fdcan_rxcpltcallback(&chassis->motors[index], rx_data);
        return 1;
    }

    return 0; // 不是底盘电机
}






void chassis_set_target_speed(chassis_struct *chassis, float vx, float vy, float vw)
{
    if (chassis == NULL) return;
    chassis->status = CHASSIS_MODE_SPEED; // 切换回速度模式
    chassis->target_speed_x = vx;
    chassis->target_speed_y = vy;
    chassis->target_speed_w = vw;
}




/**
 * @brief 底盘逆运动学解算（标准 X型/O型 麦克纳姆轮）
 * 将底盘平移及旋转速度解算为 3 个电机的期望角速度 (rad/s)
 */
void chassis_calculate_inverse_kinematics(chassis_struct *chassis)
{
    if (chassis == NULL) return;

    float vx = chassis->target_speed_x;
    float vy = chassis->target_speed_y;
    float vw = chassis->target_speed_w;

    // 几何分量 (线速度转化为轮角速度 rad/s)
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
 * @brief 周期性执行：更新轮组 PID 测量值并计算闭环输出
 * 建议在 1ms 控制中断或任务中调用
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
 * @brief can发送函数
 */
void chassis_send_can_data(chassis_struct *chassis)
{
    for (int i = 0; i < 3; i++)
    {
        motor_2006_send_buffer(&chassis->motors[i]);
    }

    motor_2006_group_send_data(&fdcan1_manage_object,0x200);

}



/**
 * @brief 底盘完全失能 / 急停断电
 */
void chassis_stop(chassis_struct *chassis)
{
    if (chassis == NULL) return;

    for (int i = 0; i < 3; i++)
    {
        chassis->status = CHASSIS_MODE_STOP;
        motor_2006_set_control_method(&chassis->motors[i], M2006_Control_Method_OPENLOOP);
        motor_2006_set_out(&chassis->motors[i], 0.0f);
    }
}




