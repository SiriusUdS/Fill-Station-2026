#pragma once

#include <cstdint>

union InterfaceFieldFlags {
    struct {
        uint8_t initialized: 1;
        uint8_t readingError: 1;
        uint8_t writingError: 1;
        uint8_t deviceNotFound: 1;
        uint8_t invalidState: 1;
        uint8_t crcError: 1;
        uint8_t openState: 1;
        uint8_t closeState: 1;
    } bits;
    uint8_t value;
};
static_assert(sizeof(InterfaceFieldFlags) == 1, "InterfaceFieldFlags must be exactly 1 byte (on the wire)");

struct InterfaceFrame {
    InterfaceFieldFlags canFlags;
    InterfaceFieldFlags ethernetFlags;
    InterfaceFieldFlags sdCardFlags;
    InterfaceFieldFlags valve1Flags;
    InterfaceFieldFlags valve2Flags;
    InterfaceFieldFlags igniterFlags;
    uint16_t            reserved;
    uint32_t            erno;
};

union InterfaceField {
    InterfaceFrame frame;
    uint8_t        byteMap[sizeof(InterfaceFrame)];
};
static_assert(sizeof(InterfaceField) == 12, "InterfaceField has implicit padding — expected 6 flag bytes + 2 reserved + 4 erno");
