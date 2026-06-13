#pragma once

#include <cstdint>

#include "peripherals/storage/storage_error.hpp"   // StorageError (the 2-bit error field)

/** @brief Backing-store status register (8-bit), the Status half of StorageInfo. */
struct StorageStatus {
    uint8_t      initialized   : 1;  /**< true once the volume is mounted and the log file is open. */
    uint8_t      plugged_in    : 1;  /**< true if the card is detected as present. */
    uint8_t      write_enabled : 1;  /**< true if writing is armed; false (default) makes write() a no-op. */
    StorageError error         : 2;  /**< last failure cause; None while healthy (state then Active). */
    uint8_t      write_overrun : 1;  /**< log double-buffer overran: a half filled before it could be flushed, so records were dropped. */
    uint8_t      reserved      : 2;  /**< Padding; room for future status bits. */
};
static_assert(sizeof(StorageStatus) == 1, "StorageStatus must be exactly 1 byte (on the wire)");
