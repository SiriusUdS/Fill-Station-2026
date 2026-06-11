#pragma once
#include "can/CANControllerConfig.h"
#include "valve/ValveController.h"

typedef struct {
    Valve *valves;
    uint8_t valveCount;
} ValveHandlerCtx;

void handler_valve(void *ctx, const CANHeader *header, const uint8_t *rxData);