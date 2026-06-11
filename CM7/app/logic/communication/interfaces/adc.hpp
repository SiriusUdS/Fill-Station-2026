#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

/* ------------------------------------------------------------------------- *
 * Class-based ADC contract for the logic layer (C++23 concepts).
 *
 * The logic layer depends ONLY on the concepts below: any type with the right
 * member functions models them, checked at compile time. No HAL type, and no
 * bus/SPI/DMA/DRDY detail, appears here — acquisition mechanism is the driver's
 * concern, which is exactly what this seam hides.
 *
 * Two contracts, by acquisition model rather than by transport:
 *   - Adc           — the PULL baseline: ask for the latest conversion. Both a
 *                     continuous ADC and a polled one can honour it.
 *   - StreamingAdc  — a REFINEMENT of Adc that also PUSHES each conversion through
 *                     a callback. Only a continuous/DRDY-paced device offers this
 *                     (on this board, the ADS131M08 on SPI4). A polled device
 *                     (e.g. the SPI6 ADCs, ~10 Hz) models Adc but not StreamingAdc.
 *
 * Each device is an OBJECT that owns its own latest-sample storage, so a board
 * holds one instance per ADC instead of a hidden global. Logic that needs the
 * high-rate stream is templated on StreamingAdc and registers a callback; logic
 * that just wants the latest values is templated on Adc and pulls samples().
 * ------------------------------------------------------------------------- */

namespace logic::communication {

/**
 * @brief  Per-sample callback type used by the StreamingAdc refinement.
 *
 * Invoked with the freshly-parsed channel counts (one signed int32 per channel).
 * For a continuous/DRDY device this runs in interrupt context at the conversion
 * rate — keep it short (no blocking I/O). The span views driver-owned storage,
 * valid only for the duration of the call. nullptr disables it.
 */
using SampleCallback = void (*)(std::span<const int32_t> channels);

/**
 * @brief The baseline ADC contract: pull the latest conversion.
 *
 * A conforming type exposes:
 *   - static constexpr std::size_t channel_count — channels it streams.
 *   - samples() — the most recent conversion as a view over driver-owned storage
 *                 (one signed count per channel, valid only until the next call),
 *                 or std::nullopt if no new conversion has completed since the
 *                 last call.
 */
template <typename T>
concept Adc = requires {
    { T::channel_count } -> std::convertible_to<std::size_t>;
} && requires(T adc) {
    { adc.samples() } -> std::same_as<std::optional<std::span<const int32_t>>>;
};

/**
 * @brief A continuous ADC that, on top of Adc, pushes every conversion.
 *
 * Adds set_sample_callback(): register (or clear, with nullptr) a callback
 * invoked per conversion. Only event-driven devices (continuous, DRDY-paced)
 * can satisfy this; a polled device models Adc alone.
 */
template <typename T>
concept StreamingAdc = Adc<T> && requires(T adc, SampleCallback cb) {
    { adc.set_sample_callback(cb) } -> std::same_as<void>;
};

} // namespace logic::communication
