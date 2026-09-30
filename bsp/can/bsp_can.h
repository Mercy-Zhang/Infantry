/**
 * @file bsp_can.h
 * @brief FDCAN 收发封装(标准帧, 8 字节)
 */
#ifndef INF_BSP_CAN_H
#define INF_BSP_CAN_H

#define FDCAN

#include <stdint.h>

#ifdef FDCAN
#include "fdcan.h"
#define hcan1 hfdcan1
#define hcan2 hfdcan2
#define hcan3 hfdcan3
#define DEVICE_CAN_CNT 3
#endif

#define CAN_MX_REGISTER_CNT 16

#pragma pack(1)
typedef struct _CANInstance
{
    FDCAN_HandleTypeDef  *can_handle;
    FDCAN_TxHeaderTypeDef txconf;
    uint32_t tx_id;
    uint32_t tx_mailbox;
    uint8_t  tx_buff[8];
    uint8_t  rx_buff[8];
    uint32_t rx_id;
    uint8_t  rx_len;
    void (*can_module_callback)(struct _CANInstance *);
    void *id;   /* 拥有此 CAN 实例的模块指针 */
} CANInstance;
#pragma pack()

typedef struct
{
    FDCAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;

/* 注册一个 CAN 实例, 返回实例指针 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/* 修改发送帧数据长度(最大 8) */
void CANSetDLC(CANInstance *instance, uint8_t length);

/* 发送 tx_buff 中的一帧, 返回 1=成功 0=失败 */
uint8_t CANTransmit(CANInstance *instance, float timeout);

/* FDCAN bus-off 检测与恢复(周期调用) */
void CANBusOffRecovery(void);

extern volatile uint8_t  can_busoff_flag[DEVICE_CAN_CNT];
extern volatile uint32_t can_busoff_recovery_cnt[DEVICE_CAN_CNT];

/* 由 HAL_FDCAN_RxFifo0Callback 调用, 排空 FIFO 并分发 */
void BspCanOnRx(FDCAN_HandleTypeDef *hfdcan);

/* 配置接收全部标准 ID(在 FDCAN 初始化后调用一次) */
void BspCanInit(void);

#endif /* INF_BSP_CAN_H */
