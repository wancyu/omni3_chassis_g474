/**
 * @file alg_pid.c
 * @author wancyu 3388409784@qq.com
 * @brief pid基本算法
 * @version 1.1
 * @date 2026_08_17
 *
 */

#include "alg_pid.h"
#include <math.h>

/**
 * @brief 数值区间限幅函数
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
    pid->dt = (dt > 1e-6f) ? dt : 0.001f;

    // 默认高级配置：开启微分先行与滤波，关闭死区与变速积分
    pid->deadband = 0.0f;
    pid->thresh_a = 0.0f;
    pid->thresh_b = 0.0f;
    pid->d_lpf_alpha = 0.8f;
    pid->d_on_measurement = 1;

    pid_reset(pid);

}

/**
 * @brief 高级参数配置
 */
void pid_config_advanced(pid_struct *pid, const float deadband, const float thresh_a,
                         const float thresh_b, const float d_lpf_alpha, const uint8_t d_on_measurement)
{
    if (pid == 0) return;

    pid->deadband = deadband;
    pid->thresh_a = thresh_a;
    pid->thresh_b = thresh_b;
    pid->d_lpf_alpha = constrain(d_lpf_alpha, 0.0f, 0.99f);
    pid->d_on_measurement = (d_on_measurement != 0) ? 1 : 0;
}

/**
 * @brief 复位 PID 控制器内部状态
 */
void pid_reset(pid_struct *pid)
{
    if (pid == 0) return;

    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_current = 0.0f;

    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->k_v = 0.0f;
    pid->ff_out = 0.0f;
    pid->output = 0.0f;

    // 标记初始化状态，用于初次微分防冲
    pid->is_initialized = 0;
}

/**
 * @brief 计算变速积分的平滑衰减系数
 */
static float calculate_integral_factor(const pid_struct *pid, const float abs_error)
{
    if (pid->thresh_b <= 0.0f || pid->thresh_b <= pid->thresh_a) {
        return 1.0f;
    }

    if (abs_error <= pid->thresh_a) {
        return 1.0f;
    } else if (abs_error < pid->thresh_b) {
        return (pid->thresh_b - abs_error) / (pid->thresh_b - pid->thresh_a);
    } else {
        return 0.0f;
    }
}



/**
 * @brief PID 控制周期计算函数
 *
 * @param pid 指向 PID 控制器结构体实例的指针
 * @return float 经限幅计算后的最终控制量输出
 */
/**
 * @brief PID 控制周期计算函数
 */
float pid_calculate(pid_struct *pid)
{
    if (pid == 0) return 0.0f;

    // 首次运行防冲击处理：第一拍将历史值对齐当前输入，杜绝首拍微分突变
    if (pid->is_initialized == 0) {
        pid->last_current = pid->current;
        pid->last_error = pid->target - pid->current;
        pid->is_initialized = 1;
    }

    float raw_error = pid->target - pid->current;

    //平滑死区处理
    if (pid->deadband > 0.0f) {
        if (raw_error > pid->deadband) {
            pid->error = raw_error - pid->deadband;
        } else if (raw_error < -pid->deadband) {
            pid->error = raw_error + pid->deadband;
        } else {
            pid->error = 0.0f;
        }
    } else {
        pid->error = raw_error;
    }

    // 比例项 (P)
    pid->p_out = pid->kp * pid->error;

    //分项 (I) + 变速积分 + 输出饱和冻结
    if (pid->ki != 0.0f) {
        uint8_t is_saturated = 0;
        if (pid->max_out > 0.0f) {
            if ((pid->output >= pid->max_out && pid->error > 0.0f) ||
                (pid->output <= -pid->max_out && pid->error < 0.0f)) {
                is_saturated = 1;
            }
        }

        if (!is_saturated) {
            float i_factor = calculate_integral_factor(pid, fabsf(pid->error));
            pid->i_out += i_factor * (pid->ki * pid->error * pid->dt);

            if (pid->max_i_out > 0.0f) {
                pid->i_out = constrain(pid->i_out, -pid->max_i_out, pid->max_i_out);
            }
        }
    } else {
        pid->i_out = 0.0f;
    }

    /* 4. 微分项 (D) - 微分先行 / 传统微分 + 一阶低通滤波 */
    float d_raw;
    if (pid->d_on_measurement) {
        d_raw = -pid->kd * (pid->current - pid->last_current) / pid->dt;
    } else {
        d_raw = pid->kd * (pid->error - pid->last_error) / pid->dt;
    }

    pid->d_out = pid->d_lpf_alpha * pid->d_out + (1.0f - pid->d_lpf_alpha) * d_raw;

    /* 5. 前馈项 (Feed-Forward) = 自动静摩擦前馈 + 外部显式前馈 */
    float auto_friction_ff = 0.0f;
    if (pid->k_friction > 0.0f) {
        if (pid->target > 0.05f) {
            auto_friction_ff = pid->k_friction;
        } else if (pid->target < -0.05f) {
            auto_friction_ff = -pid->k_friction;
        }
    }

    pid->ff_out = auto_friction_ff + pid->k_v *pid->target;

    //输出合成与全局限幅
    float total = pid->p_out + pid->i_out + pid->d_out + pid->ff_out;
    if (pid->max_out > 0.0f) {
        pid->output = constrain(total, -pid->max_out, pid->max_out);
    } else {
        pid->output = total;
    }

    //更新历史记录
    pid->last_error = pid->error;
    pid->last_current = pid->current;

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








