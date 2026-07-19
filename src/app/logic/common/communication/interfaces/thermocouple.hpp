#pragma once

#include <array>
#include <concepts>
#include <cstdint>

#include "communication/protocol/peripherals/thermocouple/thermocouple_info.hpp"   // ThermocoupleInfo + THERMOCOUPLE_COUNT

/* ------------------------------------------------------------------------- *
 * Class-based thermocouple-bank contract for the logic layer (C++23 concept).
 *
 * The polled sibling of the ADC seam (adc.hpp): where the streaming ADC pushes
 * every conversion into a ring (StreamingAdc::pop), a thermocouple bank is polled.
 * One object owns all THERMOCOUPLE_COUNT channels (the 2 MAX31856 on SPI6 in
 * firmware). Acquisition is NON-BLOCKING: service() advances an interrupt-driven
 * round-robin one step at a time so the caller (the foreground loop) never stalls
 * on the bus, and info() returns the latest snapshot the logic folds into
 * telemetry. No HAL / SPI / CS detail appears here — acquisition mechanism is the
 * driver's concern, which is exactly what this seam hides.
 * ------------------------------------------------------------------------- */

namespace logic::communication {

/**
 * @brief A bank of polled thermocouple channels, acquired without blocking.
 *
 * service(now_ms) — advance acquisition by one non-blocking step (kick the next
 *                   channel's interrupt-driven read, or harvest/abandon the one in
 *                   flight). Call every foreground iteration; it self-paces and
 *                   times a stuck transfer out, so it never blocks the loop.
 * info()          — the latest per-channel ThermocoupleInfo snapshot (state +
 *                   status + temperatures); cheap, owned and kept current by the bank.
 */
template <typename T>
concept ThermocoupleBank = requires(T tc, uint32_t now_ms) {
    { tc.service(now_ms) } -> std::same_as<void>;
    { tc.info() }          -> std::same_as<std::array<::ThermocoupleInfo, THERMOCOUPLE_COUNT>>;
};

} // namespace logic::communication
