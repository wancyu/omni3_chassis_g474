#ifndef DRV_FDCAN_H
#define DRV_FDCAN_H

#include "stm32g4xx_hal.h"

#define FDCAN_DATA_MAX_LEN 64
/**
 * @brief FDCAN 接收缓存数据包 (安全容量 64 字节)
 */
typedef struct {
    FDCAN_RxHeaderTypeDef header;
    uint8_t               data[FDCAN_DATA_MAX_LEN];
} fdcan_rx_buffer_struct;


/**
 * @brief 接收回调函数签名
 */
typedef void (*fdcan_callback_function)(fdcan_rx_buffer_struct *rx_buffer);

/**
 * @brief FDCAN 管理对象
 */
typedef struct {
    FDCAN_HandleTypeDef   *can_handler;
    fdcan_rx_buffer_struct rx_buffer;
    fdcan_callback_function        fdcan_callback;
} fdcan_manage_object_struct;

/* ================== 外部全局变量声明 ================== */
extern fdcan_manage_object_struct fdcan1_manage_object;
extern fdcan_manage_object_struct fdcan2_manage_object;
extern fdcan_manage_object_struct fdcan3_manage_object;


extern uint8_t CAN_Supercap_Tx_Data[];

HAL_StatusTypeDef fdcan_filter_config(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t filter_index,
                                      uint32_t id_type,
                                      uint32_t filter_type,
                                      uint32_t filter_config,
                                      uint32_t id1,
                                      uint32_t id2);
void fdcan_manage_init(fdcan_manage_object_struct *obj, FDCAN_HandleTypeDef *hfdcan, fdcan_callback_function callback);
uint8_t fdcan_send_classic_data(fdcan_manage_object_struct *obj, uint16_t id, const uint8_t *data, uint16_t length);



#endif //DRV_FDCAN_H