#pragma once

#include <cstdint>

/* Telemetry representation of the CAN interface's state. */

/** @brief The CAN interface's reported state. */
enum class CanState : uint8_t {
    Unknown = 0x00,  /**< not yet initialized */
    Active  = 0x01,  /**< initialized, peripheral started */
    Error   = 0x02,  /**< init failed / bus-off */
};

// Wire layout guard: must be packed with no implicit padding.
static_assert(sizeof(CanState) == 1, "CanState must be exactly 1 byte");
