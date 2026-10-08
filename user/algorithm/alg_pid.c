/**
 * @file alg_pid.c
 * @author wancyu 3388409784@qq.com
 * @brief pid基本算法
 * @version 0.1
 * @date 2026_08_17
 *
 */

#include "alg_pid.h"


/**
 * @brief 数值区间限幅函数
 *
 * @param val 待限幅的输入浮点数
 * @param min 允许的最小值
 * @param max 允许的最大值
 * @return float 限幅裁剪后的数值
 */
static float constrain(const float val, const float min, const float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * @brief 初始化 PID 控制器参数及安全限制
 *
 * @param pid 指向 PID 控制器结构体实例的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param max_i_out 积分项最大输出限幅幅值 (正数生效，<= 0 表示不限制)
 * @param max_out 总输出最大限幅幅值 (正数生效，<= 0 表示不限制)
 * @param dt 周期计时/s
 */
void pid_init(pid_struct *pid, const float kp, const float ki, const float kd, const float max_i_out, const float max_out, const float dt){
    if (pid == 0) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_i_out = max_i_out;
    pid->max_out = max_out;
    pid->dt = dt;
    pid_reset(pid);

}

/**
 * @brief 重置 PID 控制器的运行状态量
 *
 * @param pid 指向 PID 控制器结构体实例的指针
 */
void pid_reset(pid_struct *pid)
{
    if (pid == 0) return;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief PID 控制周期计算函数
 *
 * @param pid 指向 PID 控制器结构体实例的指针
 * @return float 经限幅计算后的最终控制量输出
 */
float pid_calculate(pid_struct *pid)
{
    if (pid == 0) return 0.0f;

    pid->error = pid->target - pid->current;

    //比例项p
    const float p_out = pid->kp * pid->error;

    //积分项i
    pid->integral +=  pid->error * pid->dt;
    float i_out = pid->ki * pid->integral;
    if (pid->max_i_out > 0.0f) {
        i_out = constrain(i_out, -pid->max_i_out, pid->max_i_out);
        if (pid->ki != 0.0f) {
            pid->integral = i_out / pid->ki;
        }
    }

    //微分项
    const float d_out = pid->kd * (pid->error - pid->last_error) / pid->dt;

    pid->output = p_out + i_out + d_out;
    if (pid->max_out > 0.0f) {
        pid->output = constrain(pid->output, -pid->max_out, pid->max_out);
    }

    pid->last_error = pid->error;

    return pid->output;

}

/**
 * @brief PID 控制周期计算函数(带传参)
 *
 * @param pid 指向 PID 控制器结构体实例的指针
 * @param target 目标值
 * @param current 实际值
 * @return float 经限幅计算后的最终控制量输出
 */
float pid_calculate_once(pid_struct *pid, float target, float current)
{
    pid->target = target;
    pid->current = current;
    return pid_calculate(pid);
}








