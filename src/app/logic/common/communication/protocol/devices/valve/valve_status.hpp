#pragma once

#include <cstdint>

struct ValveStatus {
    uint8_t initialized : 1;  /**< true if the valve has been initialized and is ready to operate. */
    uint8_t open_limit_high : 1;  /**< true if the open limit switch is currently asserted. */
    uint8_t closed_limit_high : 1;  /**< true if the closed limit switch is currently asserted. */
    uint8_t in_transition : 1;  /**< true if the valve is currently moving towards a target position. */
    uint8_t fault_both_switches : 1;  /**< true if both limit switches are simultaneously asserted (an error state). */
    uint8_t opened_switch_ignored : 1;  /**< true if the valve has no physical open switch: open() just floats at 100 %. */
    uint8_t reserved : 2;  /**< Padding; keeps the struct byte-aligned and allows for future status bits. */
};

static_assert(sizeof(ValveStatus) == 1, "ValveStatus must be exactly 1 byte (on the wire)");