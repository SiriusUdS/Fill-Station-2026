#pragma once
/* HAL-free: the protocol layer is transport-agnostic (it talks over CanBus_*,
   which resolves to FDCAN on the M4 and the IPC bridge on the M7). The M7's HAL
   has the FDCAN module disabled, so this header must not pull in dil/can.h. */
#include "dil/can_types.h"

typedef void (*CANMsgHandler)(void *ctx,
                              const CANHeader *header,
                              const uint8_t *rxData);

typedef struct {
    uint32_t messageID;
    CANMsgHandler handler;
    void *ctx;
} CANHandlerEntry;

typedef struct {
    uint8_t nodeID;   /* board id (PacketHeaderVariable.h) */
    const CANHandlerEntry *handlers;
    uint8_t handlerCount;
} CANControllerConfig;