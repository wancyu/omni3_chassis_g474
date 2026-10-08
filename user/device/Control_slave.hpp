/**
 * @file    Control_slave.hpp
 * @brief   车上这一份 NRF 控制协议
 *
 * 作用：解开遥控器的 16 字节，组 8 字节应答，并记住最新一包。
 * 字节含义与遥控器 Control_master 相同。不操作 SPI。
 *
 * 用法：
 *   if (g_control_slave.Parse(pkt, &cmd))
 *   {
 *     g_control_slave.Publish(cmd);
 *   }
 *
 * RadioTask 是 C，所以仍调用 ControlSlave_Parse 这些函数，它们转到本对象。
 */
#ifndef CONTROL_SLAVE_HPP
#define CONTROL_SLAVE_HPP

#include "Control_slave.h"

class Class_ControlSlave
{
public:
  Class_ControlSlave();

  uint8_t Parse(const uint8_t pkt[CONTROL_PKT_LEN], ControlCommand *out);
  void BuildAck(uint8_t seq, uint8_t status, uint8_t fault, uint8_t out[CONTROL_ACK_LEN]);
  void Publish(const ControlCommand *cmd);
  uint8_t Get(ControlCommand *out);

private:
  static uint8_t Xor(const uint8_t *data, uint8_t len);
  static uint16_t GetU16(const uint8_t *p);

  ControlCommand cmd_;
  uint32_t tick_;
  uint8_t has_;
};

extern Class_ControlSlave g_control_slave;

#endif /* CONTROL_SLAVE_HPP */
