#pragma once
#include <stdint.h>
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
    uint8_t  deviceState;
    uint8_t  reserved[3];   /**< Explicit padding bytes — keeps the layout padding-free. */
    uint32_t deviceTS_MS;
} EthernetHeader;

// Guard the wire layout: the struct must be exactly 12 bytes with no implicit
// padding. If a future field change reintroduces padding, the sum-of-members
// assert fails — add an explicit reservedN byte to close the gap.
static_assert(sizeof(EthernetHeader) == 12,
              "EthernetHeader must be exactly 12 bytes (UDP packet header wire format)");
static_assert(sizeof(EthernetHeader) == sizeof(uint32_t)   // deviceID + payloadID + payloadLenght bit-fields
                                      + sizeof(uint8_t)     // deviceState
                                      + sizeof(uint8_t[3])  // reserved
                                      + sizeof(uint32_t),   // deviceTS_MS
              "EthernetHeader has implicit padding — add explicit reserved bytes to close the gap");

