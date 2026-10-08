#include "radio_task.h"
#include "cmsis_os2.h"
#include "../driver/nrf24.h"
#include "Control_slave.h"
#include <stdio.h>


extern osSemaphoreId_t NrfIrqSemHandle;

void radio_task(void *argument)
{
    uint8_t rx_pkt[CONTROL_PKT_LEN];
    uint8_t ack_pkt[CONTROL_ACK_LEN];
    ControlCommand cmd;

    /* 1. 初始化模块：确保上电平稳后再初始化 */
    osDelay(50);
    while (Nrf24_InitRx() == 0)
    {
        /* 读回 RF_CH 不是 100，说明 SPI 接线松了或模块没插好 */
        // printf("nRF24 Init Failed! Check SPI4 and wiring...\r\n");
        osDelay(500);
    }

    /* 预填第一包空 ACK 应答 */
    ControlSlave_BuildAck(0, CONTROL_ACK_STATUS_LINK, 0, ack_pkt);
    Nrf24_QueueAck(ack_pkt);

    for (;;)
    {
        /* 2. 等待 PC13 中断给出的信号量 (超时设为 10ms，兼顾 Polling 保底) */
        osSemaphoreAcquire(NrfIrqSemHandle, 10);
        // osStatus_t status = osSemaphoreAcquire(NrfIrqSemHandle, 10);

        // /* 3. 无论是因为中断唤醒，还是每 10ms 超时保底，都去 Poll 一下 */
        while (Nrf24_Poll(rx_pkt))
        {
            /* 4. 解析 16 字节数据帧 */
            if (ControlSlave_Parse(rx_pkt, &cmd))
            {
                /* 5. 校验通过，发布到全局命令池 (带 200ms 超时时间戳) */
                ControlSlave_Publish(&cmd);

                /* 6. 装填下一包的 ACK 硬件应答回传遥控器 */
                ControlSlave_BuildAck(cmd.seq, CONTROL_ACK_STATUS_LINK, 0, ack_pkt);
                Nrf24_QueueAck(ack_pkt);



#if RADIO_VOFA_LINK_TEST
                /* 调试打印：可在串口/VOFA 中观察摇杆值与丢包情况 */
                // 如需打印，格式可按注释里的 FireWater
#endif
            }
        }
    }
}