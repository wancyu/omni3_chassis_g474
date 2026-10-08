#ifndef DVC_LIDAR_H
#define DVC_LIDAR_H

#include <stdint.h>
#include "usart.h"

#pragma pack(push, 1)
// 之前讨论的 11 字节定长压缩帧协议结构体
typedef struct {
    uint8_t  header[2]; // 0x5A, 0xA5
    uint8_t  cmd_id;    // 0x01
    int16_t  vx;        // 放大 1000 倍，单位 m/s
    int16_t  vy;        // 放大 1000 倍，单位 m/s
    int16_t  vw;        // 放大 1000 倍，单位 rad/s
    uint8_t  checksum;  // 校验码 (如前 9 字节累加和)
    uint8_t  tail;      // 0xED
} LidarSpeedPacket_t;
#pragma pack(pop)

// 雷达下发给底盘的目标速度包
typedef struct {
    float vx;                  // m/s
    float vy;                  // m/s
    float vw;                  // rad/s
    uint32_t last_update_time; // 时间戳，用于 200ms 断链超时检测
} LidarCommand_t;

extern LidarCommand_t g_lidar_cmd;

/* 接口函数声明 */
void lidar_init(void);
void lidar_uart_callback(uint8_t *buffer, uint16_t length);
uint8_t lidar_parse_speed_frame(const uint8_t *buffer, uint16_t length, float *vx, float *vy, float *vw);

#endif /* DVC_LIDAR_H */