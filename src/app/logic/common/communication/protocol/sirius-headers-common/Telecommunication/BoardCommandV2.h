#pragma once

#include <stdint.h>

#define SYNC_PACKET_CODE     (uint8_t)0x81
#define DIAGNOSE_PACKET_CODE (uint8_t)0x02
#define REQUEST_STATE        (uint8_t)0x83
#define GET_SYSTEM           (uint8_t)0x03
#define SET_SERVO            (uint8_t)0x84
#define GET_SERVO            (uint8_t)0x04