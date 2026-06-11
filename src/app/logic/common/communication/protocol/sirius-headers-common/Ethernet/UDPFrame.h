#pragma once
#include <stdint.h>
#include "UDPDeviceCtrlFlags.h"
/**
 * 
 * UDP PACKET :
 * UDPPacketHeader (12 bytes) 
 * Payload (lenght is a multiple of 4 bytes)
 * CRC (4 bytes)
 */

/// @brief This fix-lenght frame is the first bytes read to interprets the payload. It gives RT data of the device. The payload lenght MUST be a multiple of 4 bytes (CRC not included)
typedef struct {
    uint32_t deviceID: 8;
    uint32_t payloadID: 8;
    uint32_t payloadLenght: 16;
    UDPDeviceCtrlFlags deviceCtrlFlags;
    uint8_t deviceState;
    uint16_t reserved;
    uint32_t deviceTS_MS;
}FrameUDPPacketHeader;

typedef union {
    FrameUDPPacketHeader frame;
    uint8_t bytes[sizeof(FrameUDPPacketHeader)];
} UDPPacketHeader;

