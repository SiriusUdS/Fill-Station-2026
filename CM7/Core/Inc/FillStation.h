#ifndef __STATEMACH
#define __STATEMACH
#include "sirius-headers-common/FillingStation/FillingStationState.h"
#include "sirius-headers-common/FillingStation/FillingStationErrorStatus.h"
#include "Ethernet.h"
#include "SDCard.h"

#define RX_WATCHDOG 500

typedef struct {
    uint8_t fillState;
    uint32_t currentTick;
    uint32_t lastEthernetMessageTx_MS;
    uint32_t lastEthernetMessageRx_MS;
    FillingStationErrorStatus errors;
    NetAddr gsAddr;
    SDCard sd;
    /* Latest CAN frame received on FDCAN1. */
    uint32_t canRxCount;
    uint32_t canLastRxId;
    uint8_t  canLastRxData[8];
} FillStation;



extern void FILL_init(SD_HandleTypeDef* sdHandler, FDCAN_HandleTypeDef* hfdcan);

extern void FILL_tick(uint32_t tick);

/* Queue a valve command for transmission over FDCAN1. */
extern void FILL_sendValveCmd(uint8_t valve, uint8_t cmd);

#endif