#pragma once

#include <concepts>
#include <optional>

#include "communication/protocol/peripherals/adc/adc_info.hpp"   // AdcInfo (the ADC's own info record)

/* ------------------------------------------------------------------------- *
 * Class-based ADC contract for the logic layer (C++23 concepts).
 *
 * The logic layer depends ONLY on the concepts below: any type with the right
 * member functions models them, checked at compile time. No HAL type, and no
 * bus/SPI/DMA/DRDY detail, appears here — acquisition mechanism is the driver's
 * concern, which is exactly what this seam hides.
 *
 *   - Adc           — read the ADC's latest info (state + status + channels) via
 *                     info(). Both a continuous and a polled ADC honour it.
 *   - StreamingAdc  — a continuous ADC that also buffers EVERY conversion in a
 *                     ring: pop() returns the next queued conversion (or nullopt
 *                     if the ring is empty). The controller drains the ring on its
 *                     own cadence, so no conversion is lost and the comms rate is
 *                     never tied to the ADC's DRDY rate.
 *
 * Each device OWNS its AdcInfo and keeps it current as it converts.
 * ------------------------------------------------------------------------- */

namespace logic::communication {

/**
 * @brief The baseline ADC contract: read the ADC's latest info.
 *
 * info() — the ADC's own AdcInfo (state + status + the latest per-channel counts),
 * kept up to date by the device as it converts.
 */
template <typename T>
concept Adc = requires(T adc) {
    { adc.info() } -> std::same_as<::AdcInfo>;
};

/**
 * @brief A continuous ADC that buffers every conversion in a ring.
 *
 * pop() removes and returns the oldest queued conversion (a complete AdcInfo, with
 * data_valid set), or std::nullopt when the ring is empty. The producer (the ADC's
 * own DRDY ISR) pushes; the consumer (the controller's record timer) pops — one
 * conversion is delivered exactly once, decoupling capture from the save cadence.
 */
template <typename T>
concept StreamingAdc = Adc<T> && requires(T adc) {
    { adc.pop() } -> std::same_as<std::optional<::AdcInfo>>;
};

} // namespace logic::communication
