#pragma once
#include "can/CANControllerConfig.h"
void handler_ping(void *ctx, const CANHeader *header, const uint8_t *rxData);