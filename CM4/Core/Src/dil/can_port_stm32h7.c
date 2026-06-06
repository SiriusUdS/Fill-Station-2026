/**
  ******************************************************************************
  * @file    dil/can_port_stm32h7.c
  * @brief   STM32H7 FDCAN adapter implementing the stm-2026-common CAN port
  *          contract (dil/can_port.h). This is the only place that touches the
  *          HAL; the shared CAN module stays HAL-independent.
  ******************************************************************************
  */

#include "dil/can.h"
#include "dil/can_port.h"
#include "main.h"        // HAL, FDCAN types

/* Target id occupies bits [7:4] of the extended identifier */
#define TARGET_ID_SHIFT 4
#define TARGET_ID_MASK  (0xF << TARGET_ID_SHIFT)

bool can_port_init(can_handle_t handle, uint8_t node_id)
{
  FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)handle;

  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = ((uint32_t)node_id << TARGET_ID_SHIFT);
  sFilterConfig.FilterID2 = TARGET_ID_MASK;

  if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(
        hfdcan,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    return false;
  }

  return true;
}

bool can_port_send(can_handle_t handle, uint32_t ext_id, const uint8_t *data8)
{
  FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)handle;

  FDCAN_TxHeaderTypeDef txHeader = {0};
  txHeader.Identifier = ext_id;
  txHeader.IdType = FDCAN_EXTENDED_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0) {
    return false;   // TX FIFO full
  }

  return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, (uint8_t *)data8) == HAL_OK;
}

/* FDCAN RX FIFO0 "new message" interrupt: read the frame and hand it to the
 * HAL-independent module. */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
    return;
  }

  FDCAN_RxHeaderTypeDef rxHeader;
  can_frame_t frame;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, frame.data) == HAL_OK) {
    frame.id = rxHeader.Identifier;
    CAN_OnRxFrame(&frame);
  }
}
