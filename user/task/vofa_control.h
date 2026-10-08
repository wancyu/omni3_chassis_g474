#ifndef VOFA_CONTROL_H
#define VOFA_CONTROL_H

#include <stdint.h>

/**
 * @brief VOFA 接收指令聚合结构体
 */
typedef struct {
    /* 速度控制指令 (m/s, rad/s) */
    struct {
        float vx;
        float vy;
        float vw;
    } speed;

    /* 世界坐标系位置控制指令 (m, rad) */
    struct {
        float x;
        float y;
        float yaw;
    } target_pose;

    /* 状态与时间戳 */
    volatile uint32_t last_update_time; // 上次成功更新数据的时间戳 (ms)
    volatile uint8_t  is_new_cmd;        // 是否有新指令标志 (1: 有新指令, 0: 已读取)
} vofa_cmd_struct;

/* 全局变量声明 */
extern vofa_cmd_struct vofa_cmd;
// 提供给 CubeMX 自动 extern 的任务入口
void vofa_task(void *argument);


#endif //VOFA_CONTROL_H