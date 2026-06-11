#pragma once

#include <stdint.h>

typedef struct {
    uint8_t deviceRdy :1;
    uint8_t onCAN: 1;
    uint8_t errDetected: 1;
    uint8_t remoteReq: 1;
    uint8_t reserved : 4;
} FrameUDPDeviceCtrlFlags;

typedef union {
    FrameUDPDeviceCtrlFlags frame;
    uint8_t flags;
} UDPDeviceCtrlFlags;