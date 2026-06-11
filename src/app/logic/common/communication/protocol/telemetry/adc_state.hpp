#pragma once

#include <cstdint>

/** @brief Current state of an ADC (the State half of AdcInfo). */
enum class AdcState : uint8_t {
    Unknown   = 0x00,  /**< Not yet acquiring. */
    Streaming = 0x01,  /**< Acquiring — DRDY-paced conversions are flowing. */
    Faulted   = 0x02,  /**< No conversions (silent / error). */
};
static_assert(sizeof(AdcState) == 1, "AdcState must be exactly 1 byte (on the wire)");
