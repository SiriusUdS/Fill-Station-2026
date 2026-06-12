#pragma once

#include <cstdint>

struct FrameUDPDeviceCtrlFlags {
    uint8_t deviceRdy   : 1;
    uint8_t onCAN       : 1;
    uint8_t errDetected : 1;
    uint8_t remoteReq   : 1;
    uint8_t reserved    : 4;
};

union UDPDeviceCtrlFlags {
    FrameUDPDeviceCtrlFlags frame;
    uint8_t flags;
};
static_assert(sizeof(UDPDeviceCtrlFlags) == 1, "UDPDeviceCtrlFlags must be exactly 1 byte (on the wire)");
