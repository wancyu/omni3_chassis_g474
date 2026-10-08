#ifndef WANCYU_DVC_VOFA_H
#define WANCYU_DVC_VOFA_H


#include "drv_uart.h" // 包含底层的 UART 句柄与收发声明

#define VOFA_MAX_CHANNELS 24                    // VOFA+ 最大支持 24 个通道
#define VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH 100 // 单条指令最大长度为99

/**
 * @brief VOFA 核心控制结构体
 */
typedef struct
{
    uart_manage_object_struct *uart_manage_object; // 绑定的 UART 管理对象
    uint32_t frame_tail;                          // 帧尾标识 (默认 0x7F800000)

    /* 2. 接收与字典解析 */
    uint8_t rx_variable_num;                      // 字典数量
    char **rx_variable_list;                      // 字典字符串数组指针
    int32_t variable_index;                       // 匹配到的变量索引 (-1 为未命中)
    float variable_value;                         // 解析出的数值

    /* 3. 发送数据指针挂载 */
    const void *data[VOFA_MAX_CHANNELS];          // 挂载的被观测变量指针
    uint8_t data_number;                          // 当前发送通道数
    uint8_t tx_buffer[UART_BUFFER_SIZE];          // 发送缓冲区
} vofa_struct;

/* 初始化与配置接口 */
void vofa_init(vofa_struct *vofa,
               uart_manage_object_struct *uart_obj,
               uint8_t rx_var_num,
               const char **rx_var_list,
               uint32_t frame_tail);
void vofa_set_data(vofa_struct *vofa, uint32_t number, ...);

/* 数据收发与周期处理 */
void vofa_uart_rxcpltcallback(vofa_struct *vofa, const uint8_t *rx_data, uint16_t length);
void vofa_update_and_send(vofa_struct *vofa);

/* Getter 接口 */
int32_t vofa_get_variable_index(const vofa_struct *vofa);
float vofa_get_variable_value(const vofa_struct *vofa);

#endif // WANCYU_DVC_VOFA_H