#include "dvc_lidar.h"
#include <string.h>
#include "cmsis_os2.h"
#include "drv_uart.h"

LidarCommand_t g_lidar_cmd = {0};

/**
 * @brief 解析雷达/工控机发送的定长二进制速度帧
 * @return 1: 解析成功; 0: 数据校验失败或长度不对
 */
uint8_t lidar_parse_speed_frame(const uint8_t *buffer, uint16_t length, float *vx, float *vy, float *vw)
{
    if (buffer == NULL || length < sizeof(LidarSpeedPacket_t))
    {
        return 0;
    }

    // 滑窗查找帧头 0x5A, 0xA5
    for (uint16_t i = 0; i <= length - sizeof(LidarSpeedPacket_t); i++)
    {
        if (buffer[i] == 0x5A && buffer[i + 1] == 0xA5)
        {
            const LidarSpeedPacket_t *pkt = (const LidarSpeedPacket_t *)&buffer[i];

            // 1. 确认帧尾
            if (pkt->tail != 0xED) continue;

            // 2. 校验和计算 (前 9 字节求和低 8 位)
            uint8_t sum = 0;
            const uint8_t *raw_bytes = &buffer[i];
            for (uint8_t j = 0; j < 9; j++)
            {
                sum += raw_bytes[j];
            }

            if (sum == pkt->checksum)
            {
                // 3. 校验通过，定点化还原为物理量 (除以 1000.0f)
                *vx = (float)pkt->vx / 1000.0f;
                *vy = (float)pkt->vy / 1000.0f;
                *vw = (float)pkt->vw / 1000.0f;
                return 1;
            }
        }
    }

    return 0;
}

/**
 * @brief UART2 接收完成回调函数
 */
void lidar_uart_callback(uint8_t *buffer, uint16_t length)
{
    float rx_vx = 0.0f, rx_vy = 0.0f, rx_vw = 0.0f;

    uart_send_data(&uart1_manage_object, buffer, length);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
    if (lidar_parse_speed_frame(buffer, length, &rx_vx, &rx_vy, &rx_vw))
    {
        g_lidar_cmd.vx = rx_vx;
        g_lidar_cmd.vy = rx_vy;
        g_lidar_cmd.vw = rx_vw;
        g_lidar_cmd.last_update_time = osKernelGetTickCount(); // 刷新心跳时间戳

    }
}

/**
 * @brief 雷达通信初始化（绑定串口管理对象）
 */
void lidar_init(void)
{
    memset(&g_lidar_cmd, 0, sizeof(g_lidar_cmd));
    uart_manage_init(&uart2_manage_object, &huart2, lidar_uart_callback, 128);
}