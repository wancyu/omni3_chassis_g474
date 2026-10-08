#ifndef VOFA_CONTROL_H
#define VOFA_CONTROL_H

#include <stdint.h>

// VOFA 接收到的全局目标速度 (m/s, rad/s) 与心跳时间戳
extern volatile float    g_vofa_vx;
extern volatile float    g_vofa_vy;
extern volatile float    g_vofa_vw;
extern volatile uint32_t g_vofa_last_time;


// 提供给 CubeMX 自动 extern 的任务入口
void vofa_task(void *argument);


#endif //VOFA_CONTROL_H