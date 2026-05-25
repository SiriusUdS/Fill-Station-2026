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
} FillStation;



extern void FILL_init(SD_HandleTypeDef* sdHandler);

extern void FILL_tick(uint32_t tick);

#endif