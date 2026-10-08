/**
 * @file dvc_motor_2006.h
 * @author wancyu
 * @brief  2006 电调 (M2006 电机) 底层驱动头文件
 * @version 0.1
 * @date 2026-10-02
 */

#ifndef DVC_MOTOR_2006_H
#define DVC_MOTOR_2006_H

#include "alg_pid.h"
#include "drv_fdcan.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// 角度转换系数
#define DEG_TO_RAD          (PI / 180.0f)           // deg -> rad
#define RAD_TO_DEG          (180.0f / PI)           // rad -> deg

// 转速转换系数
#define RPM_TO_RADPS        (2.0f * PI / 60.0f)     // RPM -> rad/s
#define RADPS_TO_RPM        (60.0f / (2.0f * PI))   // rad/s -> RPM

/* ==================== 1. 2006 / M2006 硬件参数 ==================== */
#define M2006_GEARBOX_RATE   (36.0f)                 // 原装减速箱减速比 36:1
#define M2006_MAX_OUTPUT      (10000)                 // 控制电流输出限幅 [-10000, 10000] 对应 [-10A, 10A]

// 共享的 CAN 报文发送缓冲区引用
extern uint8_t m2006_fdcan1_0x1ff_tx_data[];
extern uint8_t m2006_fdcan1_0x200_tx_data[8];
extern uint8_t m2006_fdcan2_0x1ff_tx_data[8];
extern uint8_t m2006_fdcan2_0x200_tx_data[8];







/**
 * @brief 电机状态
 */
typedef enum
{
    M2006_CAN_Motor_Status_DISABLE = 0,
    M2006_CAN_Motor_Status_ENABLE,
} m2006_can_motor_status_enum;

/**
 * @brief CAN电机的ID枚举类型
 */
typedef enum
{
    M2006_CAN_Motor_ID_UNDEFINED = 0,
    M2006_CAN_Motor_ID_0x201,
    M2006_CAN_Motor_ID_0x202,
    M2006_CAN_Motor_ID_0x203,
    M2006_CAN_Motor_ID_0x204,
    M2006_CAN_Motor_ID_0x205,
    M2006_CAN_Motor_ID_0x206,
    M2006_CAN_Motor_ID_0x207,
    M2006_CAN_Motor_ID_0x208,
} m2006_can_motor_id_enum;

/**
 * @brief 电机控制方式
 */
typedef enum
{
    M2006_Control_Method_OPENLOOP = 0,
    M2006_Control_Method_TORQUE,
    M2006_Control_Method_OMEGA,
    M2006_Control_Method_ANGLE,
} m2006_control_method_enum;

/* ================== 电机控制结构体 ================== */

typedef struct
{
    /* 1. PID 算法实例 */
    pid_struct pid_angle;                 // 角度环
    pid_struct pid_omega;                 // 速度环

    /* 2. 通信与总线配置 */
    fdcan_manage_object_struct *fdcan_manage_object;
    m2006_can_motor_id_enum fdcan_id;
    uint8_t *fdcan_tx_data;               // 指向集中发送缓冲区的指针
    float gearbox_rate;                   // 减速比
    float torque_max;                     // 最大额定扭矩限幅 (用于开环/力矩模式归一化)

    uint16_t encoder_num_per_round;       // 编码器单圈分辨率 (8192)
    uint16_t max_output;                  // 电流控制输出上限 (10000)

    /* 3. 原始反馈与解码中间量 */
    uint16_t rx_encoder;                  // 原始机械角度反馈 [0, 8191]
    int16_t rx_omega;                     // 原始转速反馈 (RPM)
    int16_t rx_torque;                    // 原始转矩/电流反馈 [-10000, 10000]
    uint8_t rx_temperature;               // 原始温度反馈 (℃, M2006反馈固定或保留)

    uint16_t pre_encoder;                 // 上一次机械角度
    int32_t total_round;                  // 累计旋转圈数
    int32_t total_encoder;                // 累计连续编码器刻度

    uint32_t msg_count;                   // 报文计数器 (心跳检测)
    uint32_t pre_msg_count;               // 上次检测周期的报文计数

    /* 4. 物理状态输出量 */
    m2006_can_motor_status_enum fdcan_motor_status; // 电机在线状态
    float now_angle;                      // 当前输出轴角度 (rad)
    float now_omega;                      // 当前输出轴角速度 (rad/s)
    float now_torque;                     // 当前转矩反馈量
    uint8_t now_temperature;              // 当前温度 (℃)

    /* 5. 目标控制量 */
    m2006_control_method_enum control_method;   // 当前控制方式
    float target_angle;                   // 目标角度 (rad)
    float target_omega;                   // 步进角速度 (rad/s)
    float target_torque;                  // 目标转矩
    float out;                            // 最终输出控制量 (CAN发送数值)

    float target_user_omega;              // 用户设定最终目标角速度 (rad/s)
    float max_acceleration;               // 最大允许角加速度限制 (rad/s^2), 0 表示不限制
} motor_2006_struct;

/* 初始化与配置接口 */
void motor_2006_init(motor_2006_struct *motor,
                     fdcan_manage_object_struct *fdcan_manager,
                     m2006_can_motor_id_enum fdcan_id,
                     m2006_control_method_enum control_method,
                     float gearbox_rate,
                     float max_torque);

/* 闭环与通信周期处理接口 */
void motor_2006_send_buffer(motor_2006_struct *motor);
void motor_2006_group_send_data(fdcan_manage_object_struct *fdcan_manage_object, uint16_t can_id);
void motor_2006_fdcan_rxcpltcallback(motor_2006_struct *motor, const uint8_t *Rx_Data);
void motor_2006_tim_alive_periodelapsedcallback(motor_2006_struct *motor);
void motor_2006_tim_pid_periodelapsedcallback(motor_2006_struct *motor);
void motor_2006_set_smooth_control_method(motor_2006_struct *motor, m2006_control_method_enum control_method);

/* ==================== Getter 内联函数 ==================== */

static inline uint16_t motor_2006_get_max_output(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->max_output : 0;
}

static inline m2006_can_motor_status_enum motor_2006_get_motor_status(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->fdcan_motor_status : M2006_CAN_Motor_Status_DISABLE;
}

static inline float motor_2006_get_now_angle(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->now_angle : 0.0f;
}

static inline float motor_2006_get_now_omega(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->now_omega : 0.0f;
}

static inline float motor_2006_get_now_torque(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->now_torque : 0.0f;
}

static inline uint8_t motor_2006_get_now_temperature(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->now_temperature : 0;
}

static inline m2006_control_method_enum motor_2006_get_control_method(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->control_method : M2006_Control_Method_OPENLOOP;
}

static inline float motor_2006_get_target_angle(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->target_angle : 0.0f;
}

static inline float motor_2006_get_target_omega(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->target_omega : 0.0f;
}

static inline float motor_2006_get_target_torque(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->target_torque : 0.0f;
}

static inline float motor_2006_get_out(const motor_2006_struct *motor)
{
    return motor != NULL ? motor->out : 0.0f;
}

/* ==================== Setter 内联函数 ==================== */

static inline void motor_2006_set_control_method(motor_2006_struct *motor, m2006_control_method_enum control_method)
{
    if (motor == NULL) return;
    motor->control_method = control_method;
}

static inline void motor_2006_set_target_angle(motor_2006_struct *motor, float target_angle)
{
    if (motor == NULL) return;
    motor->target_angle = target_angle;
}

static inline void motor_2006_set_target_omega(motor_2006_struct *motor, float target_omega)
{
    if (motor == NULL) return;
    motor->target_omega = target_omega;
    motor->target_user_omega = target_omega;
}

static inline void motor_2006_set_target_user_omega(motor_2006_struct *motor, float target_user_omega)
{
    if (motor == NULL) return;
    motor->target_user_omega = target_user_omega;
}

static inline void motor_2006_set_target_torque(motor_2006_struct *motor, float target_torque)
{
    if (motor == NULL) return;
    motor->target_torque = target_torque;
}

static inline void motor_2006_set_out(motor_2006_struct *motor, float out)
{
    if (motor == NULL) return;
    motor->out = out;
}

static inline void motor_2006_set_max_acceleration(motor_2006_struct *motor, float max_acceleration)
{
    if (motor == NULL) return;
    motor->max_acceleration = max_acceleration;
}

#endif // DVC_MOTOR_2006_H