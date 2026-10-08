#ifndef DRV_UART_H
#define DRV_UART_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

// 缓冲区字节长度
#define UART_BUFFER_SIZE 256

/**
 * @brief UART通信接收回调函数数据类型
 */
typedef void (*uart_callback)(uint8_t *buffer, uint16_t length);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
    UART_HandleTypeDef *uart_handler;
    uint8_t tx_buffer[UART_BUFFER_SIZE];
    uint8_t rx_buffer[UART_BUFFER_SIZE];
    uint16_t rx_buffer_length;
    uart_callback callback_function;
} uart_manage_object_struct;

/* 暴露给外部的对象声明 */
extern uart_manage_object_struct uart1_manage_object;
extern uart_manage_object_struct uart2_manage_object;
extern uart_manage_object_struct uart3_manage_object;
extern uart_manage_object_struct uart4_manage_object;
extern uart_manage_object_struct uart5_manage_object;

/* 驱动接口函数 */
void    uart_manage_init(uart_manage_object_struct *obj, UART_HandleTypeDef *huart, uart_callback callback, uint16_t rx_len);
uint8_t uart_send_data(uart_manage_object_struct *obj, const uint8_t *data, uint16_t length);


#endif //DRV_UART_H