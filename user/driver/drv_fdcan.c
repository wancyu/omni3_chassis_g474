/**
 * @file drv_fdcan.c
 * @author wancyu
 * @brief FDCAN 驱动
 * @version 0.1
 * @date 2026_10_2
 */

#include "drv_fdcan.h"

fdcan_manage_object_struct fdcan1_manage_object = {0};
fdcan_manage_object_struct fdcan2_manage_object = {0};
fdcan_manage_object_struct fdcan3_manage_object = {0};
uint8_t CAN_Supercap_Tx_Data[8];

/**
 * @brief 根据句柄反查对象
 */
static inline fdcan_manage_object_struct* get_fdcan_manage_obj(FDCAN_HandleTypeDef *hfdcan) {
 if (hfdcan == NULL) return NULL;
 if (fdcan1_manage_object.can_handler == hfdcan) return &fdcan1_manage_object;
 if (fdcan2_manage_object.can_handler == hfdcan) return &fdcan2_manage_object;
 if (fdcan3_manage_object.can_handler == hfdcan) return &fdcan3_manage_object;
 return NULL;
}

/**
 * @brief 长度字节转换为 FDCAN DLC 宏定义(最大八字节)
 */
static inline uint32_t fdcan_bytes_to_dlc(uint16_t length)
{
 switch (length)
 {
 case 0: return FDCAN_DLC_BYTES_0;
 case 1: return FDCAN_DLC_BYTES_1;
 case 2: return FDCAN_DLC_BYTES_2;
 case 3: return FDCAN_DLC_BYTES_3;
 case 4: return FDCAN_DLC_BYTES_4;
 case 5: return FDCAN_DLC_BYTES_5;
 case 6: return FDCAN_DLC_BYTES_6;
 case 7: return FDCAN_DLC_BYTES_7;
 default: return FDCAN_DLC_BYTES_8; //
 }
}


/**
 * @brief 通用 FDCAN 硬件滤波器配置函数
 *
 * @param hfdcan        FDCAN 句柄
 * @param filter_index  滤波器编号 (0~27 视 CubeMX 分配而定)
 * @param id_type       FDCAN_STANDARD_ID (标准帧) 或 FDCAN_EXTENDED_ID (扩展帧)
 * @param filter_type   FDCAN_FILTER_MASK / FDCAN_FILTER_RANGE / FDCAN_FILTER_DUAL
 * @param filter_config FDCAN_FILTER_TO_RXFIFO0 或 FDCAN_FILTER_TO_RXFIFO1
 * @param id1           目标 ID (标准帧 11 位 / 扩展帧 29 位)
 * @param id2           Mask 掩码或范围结束 ID
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef fdcan_filter_config(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t filter_index,
                                      uint32_t id_type,
                                      uint32_t filter_type,
                                      uint32_t filter_config,
                                      uint32_t id1,
                                      uint32_t id2)
{
 if (hfdcan == NULL) return HAL_ERROR;

 FDCAN_FilterTypeDef filter_init = {0};

 filter_init.IdType       = id_type;
 filter_init.FilterIndex  = filter_index;
 filter_init.FilterType   = filter_type;
 filter_init.FilterConfig = filter_config;

 if (id_type == FDCAN_STANDARD_ID) {
  filter_init.FilterID1 = id1 & 0x7FF;
  filter_init.FilterID2 = id2 & 0x7FF;
 } else {
  filter_init.FilterID1 = id1 & 0x1FFFFFFF;
  filter_init.FilterID2 = id2 & 0x1FFFFFFF;
 }

 return HAL_FDCAN_ConfigFilter(hfdcan, &filter_init);
}

/**
 * @brief 初始化 FDCAN 管理对象并启动外设
 */
void fdcan_manage_init(fdcan_manage_object_struct *obj, FDCAN_HandleTypeDef *hfdcan, fdcan_callback_function callback) {
 if (obj == NULL || hfdcan == NULL) return;

 obj->can_handler = hfdcan;
 obj->fdcan_callback = callback;

 // 配置默认通配滤波器
 fdcan_filter_config(hfdcan,0,FDCAN_STANDARD_ID,FDCAN_FILTER_MASK,FDCAN_FILTER_TO_RXFIFO0,0,0);

 // 配置全局未知帧策略 (拒绝未知帧与遥控帧)
 HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

 // 激活中断通知 (RX FIFO0 新消息到达中断)
 HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

 // Start 外设
 HAL_FDCAN_Start(hfdcan);
}

/**
 * @brief FDCAN 发送标准经典 CAN 数据帧
 *
 * @param obj    FDCAN 管理对象指针
 * @param id     目标标准 ID (0x000 ~ 0x7FF)
 * @param data   待发送数据缓冲区指针
 * @param length 数据长度 (1 ~ 8)
 * @return uint8_t HAL 状态码
 */
uint8_t fdcan_send_classic_data(fdcan_manage_object_struct *obj, const uint16_t id, const uint8_t *data, const uint16_t length)
{
 if (obj == NULL || obj->can_handler == NULL || data == NULL || length == 0 || length > 8)
 {
  return HAL_ERROR;
 }

 // 检查硬件发送 FIFO 是否有空闲位
 if (HAL_FDCAN_GetTxFifoFreeLevel(obj->can_handler) == 0)
 {
  return HAL_BUSY;
 }

 FDCAN_TxHeaderTypeDef tx_header = {0};

 tx_header.Identifier          = id & 0x7FF;
 tx_header.IdType              = FDCAN_STANDARD_ID;
 tx_header.TxFrameType         = FDCAN_DATA_FRAME;
 tx_header.DataLength          = fdcan_bytes_to_dlc(length);
 tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
 tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
 tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
 tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
 tx_header.MessageMarker       = 0x00;

 return HAL_FDCAN_AddMessageToTxFifoQ(obj->can_handler, &tx_header, (uint8_t *)data);
}

/**
 * @brief 内部接收统一处理函数
 */
static inline void fdcan_rx_dispatcher(fdcan_manage_object_struct *obj, uint32_t rx_fifo)
{
 if (HAL_FDCAN_GetRxMessage(obj->can_handler, rx_fifo, &obj->rx_buffer.header, obj->rx_buffer.data) == HAL_OK)
 {
  if (obj->fdcan_callback != NULL)
  {
   obj->fdcan_callback(&obj->rx_buffer);
  }
 }
}

/**
 * @brief FDCAN 接收 FIFO0 新消息中断回调
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, const uint32_t RxFifo0ITs)
{
 if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) return;

 // 在中断边界完成硬件句柄到管理对象的转换
 fdcan_manage_object_struct *obj = get_fdcan_manage_obj(hfdcan);
 if (obj != NULL)
 {
  fdcan_rx_dispatcher(obj, FDCAN_RX_FIFO0);
 }
}

/**
 * @brief FDCAN 接收 FIFO1 新消息中断回调
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, const uint32_t RxFifo1ITs)
{
 if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET) return;

 fdcan_manage_object_struct *obj = get_fdcan_manage_obj(hfdcan);
 if (obj != NULL)
 {
  fdcan_rx_dispatcher(obj, FDCAN_RX_FIFO1);
 }
}
