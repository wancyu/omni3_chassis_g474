/**
 * @file    Control_slave.h
 * @brief   车 ← 遥控器 的 NRF 控制协议（从机）
 *
 * 与遥控器 Control_master 同一份格式。固定 16 字节控制包、8 字节应答。
 * 从机保持接收。应答载荷在收到一包之后装入，随下一包的硬件应答发出，
 * 所以载荷里的序号是上一包，不是正在到达的这一包。
 *
 * 超过 200 ms 没有新控制包，ControlSlave_Get 返回 0，调用方应把摇杆命令清零。
 * 摇杆单位是遥控器 ADC 码 0~4095。
 *
 * RADIO_VOFA_LINK_TEST 为 1 时，RadioTask 每收到一包就向 USART1 打一行
 * FireWater，TelemetryTask 不再发电机波形。两行叠在一起时，VOFA 通道数
 * 会来回变，看不出遥控器有没有丢包。核对完把这个宏改成 0。
 *
 * 通道（都是整数，VOFA 里按这个顺序命名）：
 *   ch0 seq        采样序号。遥控器每 10 ms 加 1，但每 50 ms 才发最新的一包，
 *                  所以相邻两包正常相差大约 5，不是 1。uint8 到 255 后回到 0。
 *   ch1 l_x        左摇杆 X，ADC 码 0~4095，未减中点
 *   ch2 l_y        左摇杆 Y
 *   ch3 r_y        右摇杆 Y
 *   ch4 r_x        右摇杆 X
 *   ch5 keys       按键位。bit0~3 对应键 17~20，0 表示都没按
 *   ch6 battery    电池分压的 ADC 码
 *   ch7 age_ms     这包到达后过了多少毫秒。刚到接近 0，超过 200 视为断链
 *   ch8 link       1 有新包，0 已经超过 200 ms
 *   ch9 seq_delta  本包 seq 减上一包，按 uint8 回绕。大约 5 为正常；
 *                  长期是 0 说明序号卡住；突然到几十说明中间丢了几包
 */
#ifndef RADIO_VOFA_LINK_TEST
#define RADIO_VOFA_LINK_TEST  1
#endif

#ifndef CONTROL_SLAVE_H
#define CONTROL_SLAVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_HEAD            0x02U
#define CONTROL_TYPE_CTRL       0x01U
#define CONTROL_TYPE_ACK        0x81U
#define CONTROL_PKT_LEN         16U
#define CONTROL_ACK_LEN         8U

#define CONTROL_ACK_STATUS_LINK 0x01U
#define CONTROL_LINK_TIMEOUT_MS 200U

typedef struct
{
  uint16_t l_x;
  uint16_t l_y;
  uint16_t r_y;
  uint16_t r_x;
  uint16_t keys;
  uint16_t battery;
  uint8_t seq;
} ControlCommand;

/**
 * @brief 校验并解开 16 字节控制包
 * @return 1 通过；0 帧头、类型或校验不对
 * @note  不发 SPI，任务上下文
 */
uint8_t ControlSlave_Parse(const uint8_t pkt[CONTROL_PKT_LEN], ControlCommand *out);

/**
 * @brief 组 8 字节应答。seq 填刚收下的那一包，供下一包硬件应答使用
 * @param status  CONTROL_ACK_STATUS_LINK 或 0
 * @param fault   0 为无故障
 */
void ControlSlave_BuildAck(uint8_t seq, uint8_t status, uint8_t fault, uint8_t out[CONTROL_ACK_LEN]);

/**
 * @brief 覆盖最新命令并记下到达时刻
 * @note  任务上下文；短暂锁调度
 */
void ControlSlave_Publish(const ControlCommand *cmd);

/**
 * @brief 读取最新命令
 * @return 1 仍在 200 ms 内；0 超时或还没收到过
 */
uint8_t ControlSlave_Get(ControlCommand *out);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SLAVE_H */
