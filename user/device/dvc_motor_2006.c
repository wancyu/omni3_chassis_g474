/**
 * @file dvc_motor_2006.c
 * @author wancyu
 * @brief  2006 电调 (M2006 电机) 驱动实现
 * @version 0.1
 * @date 2026-10-02
 */

#include "dvc_motor_2006.h"




uint8_t m2006_fdcan1_0x1ff_tx_data[8];
uint8_t m2006_fdcan1_0x200_tx_data[8];
uint8_t m2006_fdcan2_0x1ff_tx_data[8];
uint8_t m2006_fdcan2_0x200_tx_data[8];






/**
 * @brief 分配 CAN 发送缓冲区 (0x201~0x204 对应 0x200 帧；0x205~0x208 对应 0x1FF 帧)
 */
static uint8_t *motor_2006_allocate_tx_data(const fdcan_manage_object_struct *fdcan_manager, m2006_can_motor_id_enum fdcan_id)
{
    if (fdcan_manager == NULL || fdcan_id < M2006_CAN_Motor_ID_0x201 || fdcan_id > M2006_CAN_Motor_ID_0x208)
    {
        return NULL;
    }

    uint8_t *buf_0x200 = NULL;
    uint8_t *buf_0x1ff = NULL;

    if (fdcan_manager == &fdcan1_manage_object)
    {
        buf_0x200 = m2006_fdcan1_0x200_tx_data;
        buf_0x1ff = m2006_fdcan1_0x1ff_tx_data;
    }
    else if (fdcan_manager == &fdcan2_manage_object)
    {
        buf_0x200 = m2006_fdcan2_0x200_tx_data;
        buf_0x1ff = m2006_fdcan2_0x1ff_tx_data;
    }
    else
    {
        return NULL;
    }

    // 线性偏移计算：0x201~0x204 映射到 0x200 报文，0x205~0x208 映射到 0x1FF 报文
    if (fdcan_id <= M2006_CAN_Motor_ID_0x204)
    {
        return &buf_0x200[(fdcan_id - M2006_CAN_Motor_ID_0x201) * 2];
    }
    else
    {
        return &buf_0x1ff[(fdcan_id - M2006_CAN_Motor_ID_0x205) * 2];
    }
}

/**
 * @brief 2006 电机初始化
 */
void motor_2006_init(motor_2006_struct *motor,
                     fdcan_manage_object_struct *fdcan_manager,
                     m2006_can_motor_id_enum fdcan_id,
                     m2006_control_method_enum control_method,
                     float gearbox_rate,
                     float max_torque)
{
    if (motor == NULL) return;

    motor->fdcan_manage_object   = fdcan_manager;
    motor->fdcan_id              = fdcan_id;
    motor->control_method        = control_method;
    motor->gearbox_rate          = (gearbox_rate <= 0.0f) ? M2006_GEARBOX_RATE : gearbox_rate;
    motor->torque_max            = max_torque;
    motor->encoder_num_per_round = 8192;
    motor->max_output            = M2006_MAX_OUTPUT; // 2006 输出限幅 [-10000, 10000]
    motor->fdcan_motor_status    = M2006_CAN_Motor_Status_DISABLE;

    motor->fdcan_tx_data         = motor_2006_allocate_tx_data(fdcan_manager, fdcan_id);

    motor->max_acceleration      = 0.0f;
    motor->target_user_omega     = 0.0f;
    motor->target_omega          = 0.0f;
    motor->target_angle          = 0.0f;
    motor->target_torque         = 0.0f;
    motor->out                   = 0.0f;

    motor->msg_count             = 0;
    motor->pre_msg_count         = 0;
    motor->total_round           = 0;
    motor->total_encoder         = 0;
}

/**
 * @brief 输出控制量写入发送缓冲区槽位 (大端序)
 */
void motor_2006_send_buffer(motor_2006_struct *motor)
{
    if (motor != NULL && motor->fdcan_tx_data != NULL)
    {
        int16_t send_val = (int16_t)motor->out;
        motor->fdcan_tx_data[0] = (uint8_t)(send_val >> 8);
        motor->fdcan_tx_data[1] = (uint8_t)(send_val & 0xFF);
    }
}

/**
 * @brief 触发物理帧发送 (0x200 或 0x1FF 集中发送帧)
 */
void motor_2006_group_send_data(fdcan_manage_object_struct *fdcan_manage_object, uint16_t can_id)
{
    if (fdcan_manage_object == NULL) return;

    uint8_t *tx_buf = NULL;
    if (fdcan_manage_object == &fdcan1_manage_object)
    {
        tx_buf = (can_id == 0x200) ? m2006_fdcan1_0x200_tx_data : ((can_id == 0x1ff) ? m2006_fdcan1_0x1ff_tx_data : NULL);
    }
    else if (fdcan_manage_object == &fdcan2_manage_object)
    {
        tx_buf = (can_id == 0x200) ? m2006_fdcan2_0x200_tx_data : ((can_id == 0x1ff) ? m2006_fdcan2_0x200_tx_data : NULL);
    }

    if (tx_buf != NULL)
    {
        fdcan_send_classic_data(fdcan_manage_object, can_id, tx_buf, 8);
    }
}

/**
 * @brief 设定电机控制方式（平滑切换与就地归零）
 */
void motor_2006_set_smooth_control_method(motor_2006_struct *motor, m2006_control_method_enum control_method)
{
    if (motor == NULL || motor->control_method == control_method) return;

    // 清空双环积分历史
    pid_set_integral(&motor->pid_angle, 0.0f);
    pid_set_integral(&motor->pid_omega, 0.0f);

    if (control_method == M2006_Control_Method_ANGLE)
    {
        // 角度就地归零
        motor->total_round = 0;
        motor->total_encoder = 0;
        motor->now_angle = 0.0f;
        motor->target_angle = 0.0f;

        // 斜坡起点对齐当前转速
        motor->target_omega = motor->now_omega;
        motor->target_user_omega = 0.0f;
    }
    else if (control_method == M2006_Control_Method_OMEGA)
    {
        motor->target_user_omega = motor->now_omega;
        motor->target_omega = motor->now_omega;
    }
    else
    {
        motor->target_torque = 0.0f;
        motor->out = 0.0f;
    }

    motor->control_method = control_method;
}

/**
 * @brief 速度斜坡规划计算
 */
static void motor_2006_omega_ramp_step(motor_2006_struct *motor)
{
    if (motor->max_acceleration <= 0.0f)
    {
        motor->target_omega = motor->target_user_omega;
        return;
    }

    float dt = motor->pid_omega.dt;
    float max_delta = motor->max_acceleration * dt;
    float delta_v = motor->target_user_omega - motor->target_omega;

    if (delta_v > max_delta)
    {
        motor->target_omega += max_delta;
    }
    else if (delta_v < -max_delta)
    {
        motor->target_omega -= max_delta;
    }
    else
    {
        motor->target_omega = motor->target_user_omega;
    }
}

/* ================= 周期回调与数据处理 ================= */

/**
 * @brief CAN 接收中断回调函数
 */
void motor_2006_fdcan_rxcpltcallback(motor_2006_struct *motor, const uint8_t *Rx_Data)
{
    if (motor == NULL || Rx_Data == NULL) return;

    motor->msg_count++;

    motor->rx_encoder     = (uint16_t)((Rx_Data[0] << 8) | Rx_Data[1]);
    motor->rx_omega       = (int16_t)((Rx_Data[2] << 8) | Rx_Data[3]);
    motor->rx_torque      = (int16_t)((Rx_Data[4] << 8) | Rx_Data[5]);
    motor->rx_temperature = Rx_Data[6];

    // 多圈编码器过零跳变判定
    if (motor->msg_count == 1)
    {
        motor->pre_encoder = motor->rx_encoder;
        motor->total_round = 0;
    }
    else
    {
        int16_t delta_encoder = (int16_t)(motor->rx_encoder - motor->pre_encoder);
        if (delta_encoder < -4096)
        {
            motor->total_round++;
        }
        else if (delta_encoder > 4096)
        {
            motor->total_round--;
        }
    }
    motor->pre_encoder = motor->rx_encoder;

    motor->total_encoder = motor->total_round * motor->encoder_num_per_round + motor->rx_encoder;

    // 物理量换算 (经过减速比折算到减速箱输出轴)
    motor->now_angle = (float)motor->total_encoder / (float)motor->encoder_num_per_round * 2.0f * PI / motor->gearbox_rate;
    motor->now_omega = (float)motor->rx_omega * RPM_TO_RADPS / motor->gearbox_rate;
    motor->now_torque = (float)motor->rx_torque;
    motor->now_temperature = motor->rx_temperature;
}

/**
 * @brief 定时器心跳检测 (掉线保护)
 */
void motor_2006_tim_alive_periodelapsedcallback(motor_2006_struct *motor)
{
    if (motor == NULL) return;

    if (motor->msg_count == motor->pre_msg_count)
    {
        motor->fdcan_motor_status = M2006_CAN_Motor_Status_DISABLE;
        motor_2006_set_out(motor, 0.0f);
        pid_set_integral(&motor->pid_angle, 0.0f);
        pid_set_integral(&motor->pid_omega, 0.0f);
        motor->target_omega = motor->now_omega;
        motor->target_user_omega = motor->now_omega;
    }
    else
    {
        motor->fdcan_motor_status = M2006_CAN_Motor_Status_ENABLE;
    }
    motor->pre_msg_count = motor->msg_count;
}

/**
 * @brief 周期性 PID 控制计算
 */
void motor_2006_tim_pid_periodelapsedcallback(motor_2006_struct *motor)
{
    if (motor == NULL) return;

    switch (motor->control_method)
    {
    case M2006_Control_Method_OPENLOOP:
    case M2006_Control_Method_TORQUE:
    {
        if (motor->torque_max != 0.0f)
        {
            motor_2006_set_out(motor, motor->target_torque / motor->torque_max * motor->max_output);
        }
        else
        {
            motor_2006_set_out(motor, 0.0f);
        }
        break;
    }
    case M2006_Control_Method_OMEGA:
    {
        motor_2006_omega_ramp_step(motor);

        pid_set_target(&motor->pid_omega, motor->target_omega);
        pid_set_current(&motor->pid_omega, motor->now_omega);
        pid_calculate(&motor->pid_omega);

        motor_2006_set_out(motor, pid_get_output(&motor->pid_omega));
        break;
    }
    case M2006_Control_Method_ANGLE:
    {
        pid_set_target(&motor->pid_angle, motor->target_angle);
        pid_set_current(&motor->pid_angle, motor->now_angle);
        pid_calculate(&motor->pid_angle);

        motor->target_user_omega = pid_get_output(&motor->pid_angle);

        motor_2006_omega_ramp_step(motor);

        pid_set_target(&motor->pid_omega, motor->target_omega);
        pid_set_current(&motor->pid_omega, motor->now_omega);
        pid_calculate(&motor->pid_omega);

        motor_2006_set_out(motor, pid_get_output(&motor->pid_omega));
        break;
    }
    default:
    {
        motor_2006_set_out(motor, 0.0f);
        break;
    }
    }

    // 每次计算完毕后即时将数值压入发送缓冲区
    motor_2006_send_buffer(motor);
}