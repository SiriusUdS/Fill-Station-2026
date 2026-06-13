#pragma once

#include <cstdint>

#include "peripherals/can/can_state.hpp"
#include "peripherals/can/can_status.hpp"

/* Telemetry unit for the CAN interface: its state, status bits and a dropped-
 * frame counter, owned and kept current by the CAN driver. */

/** @brief The CAN interface's reported state + status + dropped-frame count. */
struct CanInfo {
    CanState  state;
    CanStatus status;
    uint16_t  rx_dropped;  /**< inbound frames dropped on RX-ring overflow (saturating, ISR-counted). */
};

// Wire layout guard: must be packed with no implicit padding.
static_assert(sizeof(CanInfo) == 4, "CanInfo must be exactly 4 bytes");
static_assert(sizeof(CanInfo) == 2 * sizeof(uint8_t) + sizeof(uint16_t),
              "CanInfo has implicit padding — keep state/status then the uint16 counter");
