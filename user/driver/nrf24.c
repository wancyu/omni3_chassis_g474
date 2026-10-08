/**
 * @file    nrf24.c
 * @brief   从机 SPI4 收包。CSN 由 PE4 控制，整条命令期间保持低。
 */
#include "nrf24.h"

#include "Control_slave.h"
#include "cmsis_os.h"
#include "main.h"
#include "spi.h"

#define NRF_CMD_W_REG      0x20U
#define NRF_CMD_R_RX       0x61U
#define NRF_CMD_W_ACK      0xA8U
#define NRF_CMD_FLUSH_TX   0xE1U
#define NRF_CMD_FLUSH_RX   0xE2U
#define NRF_CMD_NOP        0xFFU

#define NRF_REG_CONFIG     0x00U
#define NRF_REG_EN_AA      0x01U
#define NRF_REG_EN_RXADDR  0x02U
#define NRF_REG_SETUP_AW   0x03U
#define NRF_REG_SETUP_RETR 0x04U
#define NRF_REG_RF_CH      0x05U
#define NRF_REG_RF_SETUP   0x06U
#define NRF_REG_STATUS     0x07U
#define NRF_REG_RX_ADDR_P0 0x0AU
#define NRF_REG_TX_ADDR    0x10U
#define NRF_REG_RX_PW_P0   0x11U
#define NRF_REG_FEATURE    0x1DU

#define NRF_STATUS_TX_DS   0x20U
#define NRF_STATUS_RX_DR   0x40U
#define NRF_FEATURE_ACKPAY 0x02U
#define NRF_IRQ_PIN        GPIO_PIN_13

extern osSemaphoreId_t NrfIrqSemHandle;

static const uint8_t s_addr[5] = {0xF0U, 0xF0U, 0xF0U, 0xF0U, 0x02U};

static void Nrf_Csn(uint8_t level)
{
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin,
                    (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Nrf_Ce(uint8_t level)
{
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin,
                    (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Nrf_Xfer(uint8_t tx)
{
  uint8_t rx = 0xFFU;

  if (HAL_SPI_TransmitReceive(&hspi4, &tx, &rx, 1U, 10U) != HAL_OK)
  {
    return 0xFFU;
  }
  return rx;
}

static void Nrf_WriteReg(uint8_t reg, uint8_t value)
{
  Nrf_Csn(0U);
  (void)Nrf_Xfer((uint8_t)(NRF_CMD_W_REG | reg));
  (void)Nrf_Xfer(value);
  Nrf_Csn(1U);
}

static void Nrf_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
  uint8_t i;

  Nrf_Csn(0U);
  (void)Nrf_Xfer((uint8_t)(NRF_CMD_W_REG | reg));
  for (i = 0U; i < len; i++)
  {
    (void)Nrf_Xfer(buf[i]);
  }
  Nrf_Csn(1U);
}

static uint8_t Nrf_ReadReg(uint8_t reg)
{
  uint8_t value;

  Nrf_Csn(0U);
  (void)Nrf_Xfer(reg & 0x1FU);
  value = Nrf_Xfer(NRF_CMD_NOP);
  Nrf_Csn(1U);
  return value;
}

static void Nrf_Cmd(uint8_t cmd)
{
  Nrf_Csn(0U);
  (void)Nrf_Xfer(cmd);
  Nrf_Csn(1U);
}

static void Nrf_ReadPayload(uint8_t *buf, uint8_t len)
{
  uint8_t i;

  Nrf_Csn(0U);
  (void)Nrf_Xfer(NRF_CMD_R_RX);
  for (i = 0U; i < len; i++)
  {
    buf[i] = Nrf_Xfer(NRF_CMD_NOP);
  }
  Nrf_Csn(1U);
}

uint8_t Nrf24_InitRx(void)
{
  uint8_t ch;

  Nrf_Ce(0U);
  Nrf_Csn(1U);
  osDelay(2U);

  Nrf_WriteReg(NRF_REG_CONFIG, 0x08U);
  Nrf_WriteReg(NRF_REG_EN_AA, 0x01U);
  Nrf_WriteReg(NRF_REG_EN_RXADDR, 0x01U);
  Nrf_WriteReg(NRF_REG_SETUP_AW, 0x03U);
  Nrf_WriteReg(NRF_REG_SETUP_RETR, 0x03U);
  Nrf_WriteReg(NRF_REG_RF_CH, 100U);
  Nrf_WriteReg(NRF_REG_RF_SETUP, 0x06U);
  Nrf_WriteReg(NRF_REG_RX_PW_P0, CONTROL_PKT_LEN);
  Nrf_WriteReg(NRF_REG_FEATURE, 0x00U);
  Nrf_WriteBuf(NRF_REG_RX_ADDR_P0, s_addr, 5U);
  Nrf_WriteBuf(NRF_REG_TX_ADDR, s_addr, 5U);
  Nrf_WriteReg(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | 0x10U);
  Nrf_Cmd(NRF_CMD_FLUSH_TX);
  Nrf_Cmd(NRF_CMD_FLUSH_RX);

  Nrf_WriteReg(NRF_REG_CONFIG, 0x0BU);          /* PWR_UP + PRIM_RX + CRC */
  osDelay(2U);
  ch = Nrf_ReadReg(NRF_REG_RF_CH);
  if (ch != 100U)
  {
    return 0U;                                  /* SPI 没写进模块 */
  }
  Nrf_Ce(1U);                                   /* CE 为高才接收，才会回硬件空应答 */
  return 1U;
}

uint8_t Nrf24_Poll(uint8_t pkt[16])
{
  uint8_t status;

  if (pkt == NULL)
  {
    return 0U;
  }

  status = Nrf_ReadReg(NRF_REG_STATUS);
  if ((status == 0x00U) || (status == 0xFFU))
  {
    return 0U;
  }

  /* 应答载荷发失败时从机也会置 MAX_RT。不清除的话，后面的包不再接收、也不再应答 */
  if ((status & (NRF_STATUS_TX_DS | 0x10U)) != 0U)
  {
    Nrf_WriteReg(NRF_REG_STATUS, 0x70U);
    Nrf_Cmd(NRF_CMD_FLUSH_TX);
  }
  if ((status & NRF_STATUS_RX_DR) == 0U)
  {
    return 0U;
  }

  Nrf_ReadPayload(pkt, CONTROL_PKT_LEN);
  Nrf_WriteReg(NRF_REG_STATUS, 0x70U);
  Nrf_Cmd(NRF_CMD_FLUSH_RX);
  return 1U;
}

void Nrf24_QueueAck(const uint8_t ack[8])
{
  uint8_t i;

  if (ack == NULL)
  {
    return;
  }

  Nrf_Cmd(NRF_CMD_FLUSH_TX);
  Nrf_Csn(0U);
  (void)Nrf_Xfer(NRF_CMD_W_ACK);
  for (i = 0U; i < CONTROL_ACK_LEN; i++)
  {
    (void)Nrf_Xfer(ack[i]);
  }
  Nrf_Csn(1U);
}

/**
 * @brief PC13 中断：只释放信号量
 * @note  ISR。不读 SPI
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == NRF_IRQ_PIN) && (NrfIrqSemHandle != NULL))
  {
    (void)osSemaphoreRelease(NrfIrqSemHandle);
  }
}
