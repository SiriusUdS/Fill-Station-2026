#pragma once

#include <cstdint>

#include "peripherals/adc/adc_state.hpp"
#include "peripherals/adc/adc_status.hpp"

/* Telemetry unit for one ADC: its state, status and the latest simultaneous
 * conversion (one count per channel), owned and produced by the ADC driver. */

/** @brief Channels the streaming ADC (ADS131M08) converts simultaneously. */
inline constexpr unsigned ADC_CHANNEL_COUNT = 8;

/** @brief One ADC's reported state + status + latest channel counts. */
struct AdcInfo {
    AdcState  state;
    AdcStatus status;
    uint8_t   reserved[2];   /**< Aligns channels[] to 4 bytes; keeps the record packed. */
    /* Signed sign-extended 24-bit counts. On the wire / SD these are just the raw
       4 bytes each (the ground station reads them as it likes — signed or not). */
    int32_t   channels[ADC_CHANNEL_COUNT];
};

// Wire layout guard: must be packed with no implicit padding.
static_assert(sizeof(AdcInfo) == 4 + ADC_CHANNEL_COUNT * sizeof(int32_t),
              "AdcInfo has implicit padding — keep state/status/reserved at 4 bytes then channels");
