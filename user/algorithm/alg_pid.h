#ifndef ALG_PID_H
#define ALG_PID_H



typedef struct
{
    float kp;
    float ki;
    float kd;

    float max_i_out;  // 积分限幅
    float max_out;    // 输出限幅


    float error;    // 当前误差
    float last_error;   //上一次误差

    float current;  // 当前值
    float target;   // 目标值

    float integral; //积分累加
    float output;   //输出值

    float dt;  // PID计时周期, s


} pid_struct;


//pid初始化
void pid_init(pid_struct *pid, float kp, float ki, float kd, float i_max_out, float max_out, float dt);
void pid_reset(pid_struct *pid);
//pid计算
float pid_calculate(pid_struct *pid);
float pid_calculate_once(pid_struct *pid, float target, float current);



/* Get 接口 */
static inline float pid_get_current(const pid_struct *pid) {
    return pid->current;
}

static inline float pid_get_target(const pid_struct *pid) {
    return pid->target;
}

static inline float pid_get_integral(const pid_struct *pid) {
    return pid->integral;
}

static inline float pid_get_output(const pid_struct *pid) {
    return pid->output;
}

/* Set 接口 */
static inline void pid_set_kp(pid_struct *pid, float kp) {
    pid->kp = kp;
}

static inline void pid_set_ki(pid_struct *pid, float ki) {
    pid->ki = ki;
}

static inline void pid_set_kd(pid_struct *pid, float kd) {
    pid->kd = kd;
}

static inline void pid_set_max_i_out(pid_struct *pid, float i_max_out) {
    pid->max_i_out = i_max_out;
}

static inline void pid_set_max_out(pid_struct *pid, float max_out) {
    pid->max_out = max_out;
}

static inline void pid_set_current(pid_struct *pid, float current) {
    pid->current = current;
}

static inline void pid_set_target(pid_struct *pid, float target) {
    pid->target = target;
}

static inline void pid_set_integral(pid_struct *pid, float integral) {
    pid->integral = integral;
}

#endif //ALG_PID_H