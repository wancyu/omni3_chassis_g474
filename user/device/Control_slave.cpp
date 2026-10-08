/**
 * @file    Control_slave.cpp
 * @brief   Class_ControlSlave 的实现
 *
 * 多字节是小端。校验是前 15 字节异或，校验字节自己不参与。
 * Publish / Get 锁调度，避免读到半包。不要在中断里调用。
 */
#include "Control_slave.hpp"

#include "cmsis_os.h"

Class_ControlSlave g_control_slave;

Class_ControlSlave::Class_ControlSlave()
  : tick_(0U),
    has_(0U)
{
  cmd_.l_x = 0U;
  cmd_.l_y = 0U;
  cmd_.r_y = 0U;
  cmd_.r_x = 0U;
  cmd_.keys = 0U;
  cmd_.battery = 0U;
  cmd_.seq = 0U;
}

uint8_t Class_ControlSlave::Xor(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;
  uint8_t i;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
  }
  return crc;
}

uint16_t Class_ControlSlave::GetU16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint8_t Class_ControlSlave::Parse(const uint8_t pkt[CONTROL_PKT_LEN], ControlCommand *out)
{
  if ((pkt == NULL) || (out == NULL))
  {
    return 0U;
  }
  if ((pkt[0] != CONTROL_HEAD) || (pkt[1] != CONTROL_TYPE_CTRL))
  {
    return 0U;
  }
  if (pkt[15] != Xor(pkt, 15U))
  {
    return 0U;
  }

  out->l_x = GetU16(&pkt[3]);
  out->l_y = GetU16(&pkt[5]);
  out->r_y = GetU16(&pkt[7]);
  out->r_x = GetU16(&pkt[9]);
  out->keys = GetU16(&pkt[11]);
  out->battery = GetU16(&pkt[13]);
  out->seq = pkt[2];
  return 1U;
}

void Class_ControlSlave::BuildAck(uint8_t seq, uint8_t status, uint8_t fault, uint8_t out[CONTROL_ACK_LEN])
{
  out[0] = CONTROL_HEAD;
  out[1] = CONTROL_TYPE_ACK;
  out[2] = seq;
  out[3] = status;
  out[4] = fault;
  out[5] = 0U;
  out[6] = 0U;
  out[7] = Xor(out, 7U);
}

void Class_ControlSlave::Publish(const ControlCommand *cmd)
{
  if (cmd == NULL)
  {
    return;
  }

  (void)osKernelLock();
  cmd_ = *cmd;
  tick_ = osKernelGetTickCount();
  has_ = 1U;
  (void)osKernelUnlock();
}

uint8_t Class_ControlSlave::Get(ControlCommand *out)
{
  uint32_t age;
  uint8_t ok = 0U;

  if (out == NULL)
  {
    return 0U;
  }

  (void)osKernelLock();
  if (has_ != 0U)
  {
    age = osKernelGetTickCount() - tick_;
    if (age <= CONTROL_LINK_TIMEOUT_MS)
    {
      *out = cmd_;
      ok = 1U;
    }
  }
  (void)osKernelUnlock();
  return ok;
}

extern "C" uint8_t ControlSlave_Parse(const uint8_t pkt[CONTROL_PKT_LEN], ControlCommand *out)
{
  return g_control_slave.Parse(pkt, out);
}

extern "C" void ControlSlave_BuildAck(uint8_t seq, uint8_t status, uint8_t fault, uint8_t out[CONTROL_ACK_LEN])
{
  g_control_slave.BuildAck(seq, status, fault, out);
}

extern "C" void ControlSlave_Publish(const ControlCommand *cmd)
{
  g_control_slave.Publish(cmd);
}

extern "C" uint8_t ControlSlave_Get(ControlCommand *out)
{
  return g_control_slave.Get(out);
}
