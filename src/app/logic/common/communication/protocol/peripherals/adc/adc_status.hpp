#pragma once

#include <cstdint>

/** @brief ADC status register (8-bit), the Status half of AdcInfo. */
struct AdcStatus {
    uint8_t initialized : 1;  /**< true once the ADC is bound and acquiring. */
    uint8_t data_valid  : 1;  /**< true if this record carries a fresh conversion (not a fallback). */
    uint8_t reserved    : 6;  /**< Padding; room for future status bits (crc, reset, …). */
};
static_assert(sizeof(AdcStatus) == 1, "AdcStatus must be exactly 1 byte (on the wire)");
