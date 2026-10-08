#ifndef __DVC_LIDAR_H
#define __DVC_LIDAR_H

#include <stdint.h>
#include "dvc_vofa.h"

#define LIDAR_FRAME_LEN     14
#define LIDAR_FRAME_HEADER  0x7E
#define LIDAR_FRAME_TAIL    0x21

/**
 * @brief 雷达自标定与数据健康状态枚举
 */
typedef enum {
    LIDAR_STATUS_OFFLINE = 0,    // 雷达未通信/离线
    LIDAR_STATUS_WARMING_UP,     // 雷达上电初期，等待稳定输出
    LIDAR_STATUS_CALIBRATING,    // 正在采样前若干帧计算静态零位偏置
    LIDAR_STATUS_READY           // 标定完成，已成功校零并就绪，可供外环位控闭环
} LidarStatus_e;

/* 雷达/位姿全局接收结构体 */
typedef struct {
    uart_manage_object_struct *uart_manage_object; // 绑定的 UART 管理对象
    uint8_t tx_buffer[UART_BUFFER_SIZE];          // 发送缓冲区

    float x;                     // 经零位校准后的相对大地坐标 X (m)
    float y;                     // 经零位校准后的相对大地坐标 Y (m)
    float w;                     // 经零位校准后的相对航向角 Yaw (rad, 逆时针为正)
    uint32_t last_update_time;   // 刷新时间戳（用于超时失联保护）
    LidarStatus_e status;        // 雷达当前工作与标定状态
} lidar_command_struct;

extern lidar_command_struct lidar_cmd;

/* 函数声明 */
void    lidar_init(uart_manage_object_struct *uart_manage, UART_HandleTypeDef *huart, uint16_t rx_buf_size);
void    lidar_recalibrate(void); // 触发重新零位标定（如 VOFA 指令调用）
uint8_t lidar_is_ready(void);    // 查询雷达是否标定完成且可进入位控
uint8_t lidar_parse_pose_frame(const uint8_t *buffer, uint16_t length, float *x, float *y, float *yaw);
void    lidar_uart_callback(uint8_t *buffer, uint16_t length);

#endif /* __DVC_LIDAR_H */