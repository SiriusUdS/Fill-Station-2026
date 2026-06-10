#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

/* ------------------------------------------------------------------------- *
 * Statically-linked ADC interface for the logic layer.
 *
 * The logic layer depends ONLY on the declarations below; a platform ADC driver
 * provides the definitions at link time (on this board, the ADS131M08 over SPI4
 * - but the transport is the driver's concern, not the logic layer's, and not
 * every ADC uses SPI). No HAL type, and no bus/SPI detail, appears here.
 *
 * Acquisition is started by platform bring-up; from then on the driver keeps the
 * latest conversion available through samples(), and (if registered) invokes a
 * per-sample callback from the ADC ISR for event-driven consumers.
 * ------------------------------------------------------------------------- */

namespace logic::communication {

namespace adc {

/** @brief Number of channels the ADC streams. */
inline constexpr std::size_t CHANNEL_COUNT = 8;

/**
 * @brief  The most recent conversion: one raw signed count per channel
 *         (24-bit two's complement, sign-extended into int32).
 *
 * @return A view over driver-owned storage (one value per channel, valid only
 *         until the next samples() call), or std::nullopt if no new conversion
 *         has completed since the last call.
 */
[[nodiscard]] std::optional<std::span<const int32_t>> samples();

/**
 * @brief  Per-sample callback, invoked from the ADC ISR with the freshly-parsed
 *         channel counts (one int32 per channel).
 *
 * Runs in interrupt context at the conversion rate — keep it short (no blocking
 * I/O). The span views driver-owned storage, valid only for the call. nullptr
 * to disable.
 */
using SampleCallback = void (*)(std::span<const int32_t> channels);

/** @brief Register (or clear, with nullptr) the per-sample callback. */
void set_sample_callback(SampleCallback cb);

} // namespace adc

} // namespace logic::communication
