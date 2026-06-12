#pragma once

#include <cstdint>

#include "framing/udp_device_ctrl_flags.hpp"

/* UDP packet: UDPPacketHeader (12 bytes) + payload (a multiple of 4 bytes) + CRC (4 bytes).
   The header is the first bytes read to interpret the payload. */
struct FrameUDPPacketHeader {
    uint32_t           deviceID: 8;
    uint32_t           payloadID: 8;
    uint32_t           payloadLenght: 16;
    UDPDeviceCtrlFlags deviceCtrlFlags;
    uint8_t            deviceState;
    uint16_t           reserved;
    uint32_t           deviceTS_MS;
};

union UDPPacketHeader {
    FrameUDPPacketHeader frame;
    uint8_t              bytes[sizeof(FrameUDPPacketHeader)];
};
static_assert(sizeof(UDPPacketHeader) == 12, "UDPPacketHeader must be exactly 12 bytes (UDP packet header wire format)");
