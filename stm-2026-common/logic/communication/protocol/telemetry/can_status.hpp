#pragma once

#include <cstdint>

/** @brief CAN status register (8-bit), the Status half of CanInfo. */
struct CanStatus {
    uint8_t initialized : 1;  /**< true once the FDCAN peripheral is started (init() succeeded). */
    uint8_t tx_error    : 1;  /**< true if the last send() failed (TX FIFO full / HAL error). */
    uint8_t reserved    : 6;  /**< Padding; room for future status bits (bus-off, error-passive, …). */
};
static_assert(sizeof(CanStatus) == 1, "CanStatus must be exactly 1 byte (on the wire)");
