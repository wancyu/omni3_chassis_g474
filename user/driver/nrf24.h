/**
 * @file    nrf24.h
 * @brief   车上 nRF24L01+ 接收（SPI4，PE3 CE，PE4 CSN，PC13 IRQ）
 *
 * 空中参数与遥控器一致：地址 F0 F0 F0 F0 02，频道 100，1 Mbps，0 dBm，
 * 1 字节 CRC，通道 0 自动应答。接收宽度 16 字节，应答载荷 8 字节。
 */
#ifndef NRF24_SLAVE_DRV_H
#define NRF24_SLAVE_DRV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 进入接收，并预装一包序号为 0 的应答
 * @return 1 SPI 读回频道正确；0 模块没应答，寄存器没写进去
 * @note  任务上下文；osDelay 约 2 ms。不要在中断里调用
 */
uint8_t Nrf24_InitRx(void);

/**
 * @brief 查 STATUS，有数据就读 16 字节。不依赖中断下降沿
 * @return 1 读到一包；0 当前没有
 * @note  任务上下文。中断线一直为低时，只等下降沿会把 RX FIFO 塞满，芯片就不再应答
 */
uint8_t Nrf24_Poll(uint8_t pkt[16]);

/**
 * @brief 把 8 字节应答放进通道 0，随下一包硬件应答发出
 * @note  任务上下文。来不及装进“正在接收的这一包”
 */
void Nrf24_QueueAck(const uint8_t ack[8]);

#ifdef __cplusplus
}
#endif

#endif /* NRF24_SLAVE_DRV_H */
