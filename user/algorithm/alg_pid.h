#ifndef ALG_PID_H
#define ALG_PID_H
#include <stdint.h>

//使用前馈要在pid calc 之前调用哦


typedef struct
{
    // 核心控制参数
    float kp;
    float ki;
    float kd;

    // 高级特性参数
    float deadband;           // 误差死区，<= 0 表示不启用
    float thresh_a;           // 变速积分满速阈值 (小误差区)
    float thresh_b;           // 变速积分零速阈值 (大误差区，超出时冻结积分)，<= 0 表示关闭变速积分
    float d_lpf_alpha;        // 微分一阶低通滤波系数 [0.0f, 1.0f)，0 表示不滤波
    uint8_t  d_on_measurement;   // true: 微分先行; false: 传统微分 (对误差求导)

    //标志
    float k_friction;         // 克服静摩擦所需的基底输出 (<= 0 表示关闭)
    uint8_t is_initialized;   // 初始化运行标志位 (0: 首次运行, 1: 正常运行)


    // 限幅参数
    float max_i_out;  // 积分限幅
    float max_out;    // 输出限幅
    float dt;  // PID计时周期, s



    // 运行状态量
    float target;             // 目标值
    float current;            // 反馈值
    float error;              // 当前误差
    float last_error;         // 上一次误差
    float last_current;       // 上一次反馈值 (微分先行用)

    // 输出状态量
    float p_out;              // 本拍比例项输出
    float i_out;              // 本拍积分项输出
    float d_out;              // 本拍微分项输出 (已滤波)
    float k_v;              // 外部设置的单次前馈量
    float ff_out;             // 本拍前馈项输出
    float output;             // 本拍最终限幅输出



} pid_struct;


//pid初始化
void pid_init(pid_struct *pid, float kp, float ki, float kd, float i_max_out, float max_out, float dt);
void pid_reset(pid_struct *pid);
float pid_calculate(pid_struct *pid);
float pid_calculate_once(pid_struct *pid, float target, float current);



/* ========================================================================= */
/*                              Get 接口 (观测与回传)                         */
/* ========================================================================= */

static inline float pid_get_target(const pid_struct *pid) {
    return (pid != 0) ? pid->target : 0.0f;
}

static inline float pid_get_current(const pid_struct *pid) {
    return (pid != 0) ? pid->current : 0.0f;
}

static inline float pid_get_error(const pid_struct *pid) {
    return (pid != 0) ? pid->error : 0.0f;
}

static inline float pid_get_output(const pid_struct *pid) {
    return (pid != 0) ? pid->output : 0.0f;
}

/* 内部各分项输出 (用于上位机波形观测与分析) */
static inline float pid_get_p_out(const pid_struct *pid) {
    return (pid != 0) ? pid->p_out : 0.0f;
}

static inline float pid_get_i_out(const pid_struct *pid) {
    return (pid != 0) ? pid->i_out : 0.0f;
}

static inline float pid_get_d_out(const pid_struct *pid) {
    return (pid != 0) ? pid->d_out : 0.0f;
}

static inline float pid_get_ff_out(const pid_struct *pid) {
    return (pid != 0) ? pid->ff_out : 0.0f;
}

/* 参数读取 */
static inline float pid_get_kp(const pid_struct *pid) {
    return (pid != 0) ? pid->kp : 0.0f;
}

static inline float pid_get_ki(const pid_struct *pid) {
    return (pid != 0) ? pid->ki : 0.0f;
}

static inline float pid_get_kd(const pid_struct *pid) {
    return (pid != 0) ? pid->kd : 0.0f;
}

static inline float pid_get_deadband(const pid_struct *pid) {
    return (pid != 0) ? pid->deadband : 0.0f;
}

static inline float pid_get_friction_comp(const pid_struct *pid) {
    return (pid != 0) ? pid->k_friction : 0.0f;
}


/* ========================================================================= */
/*                              Set 接口 (参数与运行量)                       */
/* ========================================================================= */

/* 核心输入与目标设置 */
static inline void pid_set_target(pid_struct *pid, const float target) {
    if (pid != 0) pid->target = target;
}

static inline void pid_set_current(pid_struct *pid, const float current) {
    if (pid != 0) pid->current = current;
}

/* 核心增益单项修改 */
static inline void pid_set_kp(pid_struct *pid, const float kp) {
    if (pid != 0) pid->kp = kp;
}

static inline void pid_set_ki(pid_struct *pid, const float ki) {
    if (pid != 0) pid->ki = ki;
}

static inline void pid_set_kd(pid_struct *pid, const float kd) {
    if (pid != 0) pid->kd = kd;
}

static inline void pid_set_gains(pid_struct *pid, const float kp, const float ki, const float kd) {
    if (pid != 0) {
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
    }
}

/* 积分项干预与清零 */
static inline void pid_set_integral(pid_struct *pid, const float i_out) {
    if (pid != 0) pid->i_out = i_out;
}

/* 限制与限幅修改 */
static inline void pid_set_max_i_out(pid_struct *pid, const float max_i_out) {
    if (pid != 0) pid->max_i_out = max_i_out;
}

static inline void pid_set_max_out(pid_struct *pid, const float max_out) {
    if (pid != 0) pid->max_out = max_out;
}

/* 高级特性参数修改 */
static inline void pid_set_deadband(pid_struct *pid, const float deadband) {
    if (pid != 0) pid->deadband = (deadband > 0.0f) ? deadband : 0.0f;
}

static inline void pid_set_variable_integral(pid_struct *pid, const float thresh_a, const float thresh_b) {
    if (pid != 0) {
        pid->thresh_a = thresh_a;
        pid->thresh_b = thresh_b;
    }
}

static inline void pid_set_d_filter(pid_struct *pid, const float alpha) {
    if (pid == 0) return;
    if (alpha < 0.0f) {
        pid->d_lpf_alpha = 0.0f;
    } else if (alpha > 0.99f) {
        pid->d_lpf_alpha = 0.99f;
    } else {
        pid->d_lpf_alpha = alpha;
    }
}

static inline void pid_set_d_on_measurement(pid_struct *pid, const uint8_t d_on_measurement) {
    if (pid != 0) pid->d_on_measurement = (d_on_measurement != 0) ? 1 : 0;
}

static inline void pid_set_friction_comp(pid_struct *pid, const float k_friction) {
    if (pid != 0) pid->k_friction = (k_friction > 0.0f) ? k_friction : 0.0f;
}

static inline void pid_set_kv(pid_struct *pid, const float kv) {
    if (pid != 0) pid->k_v = kv;
}

#endif //ALG_PID_H