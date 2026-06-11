#pragma once
#include <stdbool.h>
#include "can/CANControllerConfig.h"

typedef struct {
    const CANControllerConfig *cfg;
} CANController;

void CANController_Init (CANController *ctrl, const CANControllerConfig *cfg);

/* Process at most one received frame. Returns true if a frame was dequeued
   (whether or not it was dispatched), false when the RX ring is empty -- so
   callers can drain with `while (CANController_Process(&ctrl));`. */
bool CANController_Process(CANController *ctrl);