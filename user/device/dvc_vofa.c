/**
 * @file dvc_vofa.c
 * @author wancyu 3388409784@qq.com
 * @brief VOFA+ 上位机驱动实现
 * @version 0.1
 * @date 2026_09_26
 */

#include "dvc_vofa.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

/* 内部私有解析函数声明 */
static uint8_t vofa_judge_variable_name(vofa_struct *vofa, const uint8_t *rx_data, uint16_t length);
static void vofa_judge_variable_value(vofa_struct *vofa, const uint8_t *rx_data, uint16_t length, uint8_t val_pos);
static void vofa_output(vofa_struct *vofa);


/**
 * @brief VOFA 初始化
 *
 * @param vofa         VOFA 实例指针
 * @param uart_obj     绑定的 UART 管理对象指针
 * @param rx_var_num   接收指令字典的数量
 * @param rx_var_list  接收指令字典列表
 * @param frame_tail   帧尾标识符 (填 0 时默认设为 JustFloat 标准尾 0x7F800000)
 */
void vofa_init(vofa_struct *vofa,
               uart_manage_object_struct *uart_obj,
               uint8_t rx_var_num,
               const char **rx_var_list,
               uint32_t frame_tail)
{
    if (vofa == NULL || uart_obj == NULL) return;

    memset(vofa, 0, sizeof(vofa_struct));

    // 直接依赖抽象对象，不关心底层具体是 USART1 还是 UART5
    vofa->uart_manage_object = uart_obj;
    vofa->rx_variable_num    = rx_var_num;
    vofa->rx_variable_list = (char **)rx_var_list;
    vofa->frame_tail         = (frame_tail != 0) ? frame_tail : 0x7F800000;
    vofa->variable_index     = -1;
}

/**
 * @brief 挂载被观测变量的内存地址（可变参数）
 */
void vofa_set_data(vofa_struct *vofa, uint32_t number, ...)
{
    if (vofa == NULL) return;
    if (number > VOFA_MAX_CHANNELS) number = VOFA_MAX_CHANNELS;

    va_list data_ptr;
    va_start(data_ptr, number);
    for (uint8_t i = 0; i < number; i++)
    {
        vofa->data[i] = va_arg(data_ptr, const void *);
    }
    va_end(data_ptr);

    vofa->data_number = number;
}

/**
 * @brief 内部函数：打包数据与 JustFloat 帧尾
 */
static void vofa_output(vofa_struct *vofa)
{
    uint8_t *tmp_buffer = vofa->tx_buffer;
    memset(tmp_buffer, 0, UART_BUFFER_SIZE);

    for (uint8_t i = 0; i < vofa->data_number; i++)
    {
        if (vofa->data[i] != NULL)
        {
            memcpy(tmp_buffer + i * sizeof(float), vofa->data[i], sizeof(float));
        }
    }

    // 在末尾追加 4 字节帧尾
    memcpy(tmp_buffer + vofa->data_number * sizeof(float), &vofa->frame_tail, sizeof(uint32_t));
}

/**
 * @brief 数据更新和发送
 */
void vofa_update_and_send(vofa_struct *vofa)
{
    if (vofa == NULL || vofa->uart_manage_object == NULL) return;

    vofa_output(vofa);

    uint16_t send_len = vofa->data_number * sizeof(float) + sizeof(uint32_t);
    uart_send_data(vofa->uart_manage_object, vofa->tx_buffer, send_len);
}


/**
 * @brief vofa串口接收回调 (纯 C/嵌入式兼容，不使用 bool，逻辑等效于老代码)
 */
void vofa_uart_rxcpltcallback(vofa_struct *vofa, const uint8_t *rx_data, uint16_t length)
{
    if (vofa == NULL || rx_data == NULL || length == 0) return;

    // 默认未匹配到有效指令
    vofa->variable_index = -1;

    // 1. 确保末尾截断为合法 C 字符串，防止 sscanf 越界踩
    ((char *)rx_data)[length] = '\0';

    char var_name[VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {0};
    float var_val = 0.0f;
    char end_char = 0;

    int match_count = sscanf((const char *)rx_data, "%99[^=]=%f%c", var_name, &var_val, &end_char);

    // 3. 严格校验：必须成功提取 3 项，且末尾紧跟的字符必须是 '#'
    if (match_count != 3 || end_char != '#')
    {
        return;
    }

    // 4. 遍历字典匹配变量名并赋值
    for (uint8_t i = 0; i < vofa->rx_variable_num; i++)
    {
        if (strcmp(var_name, vofa->rx_variable_list[i]) == 0)
        {
            vofa->variable_index = i;
            vofa->variable_value = var_val;
            break;
        }
    }
}


/* ================= Getter 接口 ================= */

int32_t vofa_get_variable_index(const vofa_struct *vofa)
{
    return (vofa != NULL) ? vofa->variable_index : -1;
}

float vofa_get_variable_value(const vofa_struct *vofa)
{
    return (vofa != NULL) ? vofa->variable_value : 0.0f;
}