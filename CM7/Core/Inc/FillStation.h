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
    /* Latest CAN frame received from the M4 (via the shared RX ring). */
    uint32_t canRxCount;
    uint32_t canLastRxId;
    uint8_t  canLastRxData[8];
} FillStation;



extern void FILL_init(SD_HandleTypeDef* sdHandler);

extern void FILL_tick(uint32_t tick);

/* Queue a valve command for the M4 to transmit over CAN. */
extern void FILL_sendValveCmd(uint8_t valve, uint8_t cmd);

#endif