#pragma once

#include <cstdint>

#include "devices/valve/valve_state.hpp"
#include "devices/valve/valve_status.hpp"

/* Telemetry unit for one valve: its state, status bits and commanded position,
 * packed into 24 bits. One per valve in SystemStateBase::valve_info; replaces the
 * older 4-byte ValveState (this is the complete per-valve record). */

/** @brief One valve's reported state (24 bits). */
struct ValveInfo {
    ValveState state; /**< Current state of the valve */
    ValveStatus status; /**< Status bits: limit switch readings, transition flag, etc. */
    uint8_t current_set_value; /**< Commanded open position, 0..100 %. */
};

// Wire layout guard: must be exactly 24 bits with no implicit padding.
static_assert(sizeof(ValveInfo) == 3, "ValveInfo must be exactly 3 bytes (24 bits)");
static_assert(sizeof(ValveInfo) == 3 * sizeof(uint8_t),
              "ValveInfo has implicit padding — keep it three packed bytes");
