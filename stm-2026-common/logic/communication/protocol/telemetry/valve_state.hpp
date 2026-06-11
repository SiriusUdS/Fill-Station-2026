#pragma once

#include <cstdint>

/* Telemetry representation of a single valve's state. */

/** @brief One valve's reported state. */
enum class ValveState : uint8_t {
    Unknown  = 0x00,  /**< not yet read or in an error state */
    Opened   = 0x01,  /**< fully open, on the open limit switch */
    Closed   = 0x02,  /**< fully closed, on the closed limit switch */
    Opening  = 0x03,  /**< moving towards open, not on either switch */
    Closing  = 0x04,  /**< moving towards closed, not on either switch */
    Faulted  = 0x05,  /**< in an error state (e.g. limit switch disagreement) */
    Floating = 0x06,  /**< somewhere between open and closed, not on either switch */
};

// Wire layout guard: must be packed with no implicit padding.
static_assert(sizeof(ValveState) == 1, "ValveState must be exactly 1 bytes");
