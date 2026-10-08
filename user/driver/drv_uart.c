/**
 * @file drv_uart.c
 * @author wancyu 3388409784@qq.com
 * @brief 串口驱动
 * @version 0.1
 * @date 2026_10_2
 */
#include "drv_uart.h"

#include <string.h>

/* 实例化管理对象 */
uart_manage_object_struct uart1_manage_object = {0};
uart_manage_object_struct uart2_manage_object = {0};
uart_manage_object_struct uart3_manage_object = {0};
uart_manage_object_struct uart4_manage_object = {0};
uart_manage_object_struct uart5_manage_object = {0};

/**
 * @brief 内部辅助：根据底层 huart 句柄指针反查管理对象
 */
static inline uart_manage_object_struct* get_manage_obj(const UART_HandleTypeDef *huart) {
    if (huart == NULL) return NULL;

    // 直接对比传入的 huart 指针地址，完全不需要知道它具体叫 USART 还是 UART
    if (uart1_manage_object.uart_handler == huart) return &uart1_manage_object;
    if (uart2_manage_object.uart_handler == huart) return &uart2_manage_object;
    if (uart3_manage_object.uart_handler == huart) return &uart3_manage_object;
    if (uart4_manage_object.uart_handler == huart) return &uart4_manage_object;
    if (uart5_manage_object.uart_handler == huart) return &uart5_manage_object;

    return NULL;
}

/**
 * @brief 启动单次 DMA 接收并关闭半满中断
 */
static void uart_start_rx_dma(uart_manage_object_struct *obj) {
    HAL_UARTEx_ReceiveToIdle_DMA(obj->uart_handler, obj->rx_buffer, obj->rx_buffer_length);
    if (obj->uart_handler->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(obj->uart_handler->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief 初始化 UART 管理对象
 */
void uart_manage_init(uart_manage_object_struct *obj, UART_HandleTypeDef *huart, const uart_callback callback, uint16_t rx_len) {
    if (obj == NULL || huart == NULL) return;

    if (rx_len == 0 || rx_len > UART_BUFFER_SIZE) {
        rx_len = UART_BUFFER_SIZE;
    }

    obj->uart_handler = huart;
    obj->callback_function = callback;
    obj->rx_buffer_length = rx_len;
    uart_start_rx_dma(obj);
}

/**
 * @brief 发送数据帧 (使用内部 tx_buffer 防局部变量失效)
 */
uint8_t uart_send_data(uart_manage_object_struct *obj, const uint8_t *data, const uint16_t length) {
    if (obj == NULL || obj->uart_handler == NULL || data == NULL || length == 0) {
        return HAL_ERROR;
    }

    /* 限制单次发送长度，将待发数据安全拷贝到对象的 tx_buffer */
    const uint16_t send_len = (length > UART_BUFFER_SIZE) ? UART_BUFFER_SIZE : length;


    if (obj->uart_handler->gState != HAL_UART_STATE_READY) return HAL_BUSY;

    memcpy(obj->tx_buffer, data, send_len);

    return HAL_UART_Transmit_DMA(obj->uart_handler, obj->tx_buffer, send_len);
}



/**
 * @brief HAL 库 UART 接收空闲中断 / 满中断通用回调
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    uart_manage_object_struct *obj = get_manage_obj(huart);
    if (obj == NULL) return;

    /* 执行上层业务回调 */
    if (obj->callback_function != NULL) {
        // 注意不要有耗时操作！
        obj->callback_function(obj->rx_buffer, Size);
    }
    uart_start_rx_dma(obj);
}

/**
 * @brief 错误中断（溢出/噪声保护）
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    uart_manage_object_struct *obj = get_manage_obj(huart);
    if (obj == NULL) return;

    // 1. 强行终止当前的 DMA 接收，复位 HAL 库的 RxState 状态机
    HAL_UART_AbortReceive(huart);

    // 2. 清除所有常见错误标志位（PE, FE, NE, ORE）
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    // 3. 干净地重启 DMA
    uart_start_rx_dma(obj);
}