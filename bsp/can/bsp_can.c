/**
 * @file bsp_can.c
 * @brief FDCAN 收发封装实现
 */
#include "bsp_can.h"
#include <string.h>

static CANInstance can_pool[CAN_MX_REGISTER_CNT];
static uint8_t can_count = 0;

volatile uint8_t  can_busoff_flag[DEVICE_CAN_CNT] = {0};
volatile uint32_t can_busoff_recovery_cnt[DEVICE_CAN_CNT] = {0};

static void config_broad_filter(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter;
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x7FF;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void BspCanInit(void)
{
    memset(can_pool, 0, sizeof(can_pool));
    can_count = 0;
    /* 注意: hfdcanX 是变量不是宏, 不能写 #ifdef hfdcanX(那样恒为假, FDCAN 永远不 Start)。
     * 三个 FDCAN 无条件启动(配置全收滤波器 + Start + 使能 RX 中断)。 */
    config_broad_filter(&hfdcan1);
    config_broad_filter(&hfdcan2);
    config_broad_filter(&hfdcan3);
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (can_count >= CAN_MX_REGISTER_CNT)
        return NULL;

    CANInstance *ins = &can_pool[can_count++];
    memset(ins, 0, sizeof(CANInstance));

    ins->can_handle = config->can_handle;
    ins->tx_id      = config->tx_id;
    ins->rx_id      = config->rx_id;
    ins->can_module_callback = config->can_module_callback;
    ins->id         = config->id;
    ins->rx_len     = 8;

    ins->txconf.Identifier          = config->tx_id;
    ins->txconf.IdType              = FDCAN_STANDARD_ID;
    ins->txconf.TxFrameType         = FDCAN_DATA_FRAME;
    ins->txconf.DataLength          = FDCAN_DLC_BYTES_8;
    ins->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    ins->txconf.BitRateSwitch       = FDCAN_BRS_OFF;
    ins->txconf.FDFormat            = FDCAN_CLASSIC_CAN;
    ins->txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    ins->txconf.MessageMarker       = 0;

    return ins;
}

void CANSetDLC(CANInstance *instance, uint8_t length)
{
    if (length > 8)
        length = 8;
    instance->txconf.DataLength = (uint32_t)length << 16;
}

uint8_t CANTransmit(CANInstance *instance, float timeout)
{
    (void)timeout;
    if (HAL_FDCAN_AddMessageToTxFifoQ(instance->can_handle,
                                      &instance->txconf,
                                      instance->tx_buff) != HAL_OK)
        return 0;
    return 1;
}

void BspCanOnRx(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    /* 一次中断可能积压多帧, 循环排空 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK)
            break;

        for (uint8_t i = 0; i < can_count; i++)
        {
            if (can_pool[i].can_handle == hfdcan &&
                can_pool[i].rx_id == rx.Identifier)
            {
                memcpy(can_pool[i].rx_buff, data, 8);
                can_pool[i].rx_len = 8;
                if (can_pool[i].can_module_callback)
                    can_pool[i].can_module_callback(&can_pool[i]);
            }
        }
    }
}

void CANBusOffRecovery(void)
{
    /* 预留: 检测各 FDCAN 的 bus-off 状态并 Stop+Start 恢复 */
}

/* HAL 弱回调重写: RX FIFO0 新消息 → 排空并分发 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        BspCanOnRx(hfdcan);
}
