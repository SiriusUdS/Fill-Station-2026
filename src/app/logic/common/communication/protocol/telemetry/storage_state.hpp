#pragma once

#include <cstdint>

/* Telemetry representation of a backing store's (SD card's) lifecycle state. */

/** @brief One store's reported state. */
enum class StorageState : uint8_t {
    Init   = 0x00,  /**< constructed but not yet mounted (or init() not run) */
    Active = 0x01,  /**< mounted, log file open, ready to write */
    Error  = 0x02,  /**< mount / open / write failed (see the driver's Error for the cause) */
};

// Wire layout guard: must be packed with no implicit padding.
static_assert(sizeof(StorageState) == 1, "StorageState must be exactly 1 byte");
