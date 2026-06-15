#pragma once

#include <concepts>
#include <cstdint>

#include "communication/protocol/devices/ematch/ematch_info.hpp"   // EmatchInfo (the e-match's telemetry unit)

/* ------------------------------------------------------------------------- *
 * Class-based e-match (igniter) contract for the logic layer (C++23 concept).
 *
 * The control + telemetry layers depend ONLY on the @ref Ematch concept below: any
 * type with the right members models it, checked at compile time. There is no vtable
 * and no HAL type here — logic energises the igniter and reads its presence without
 * knowing about GPIO lines or pin polarities. The firmware models it with a
 * GpioEmatch composed over the digital-in/out seams; host tests inject a FakeEmatch.
 *
 * Like the Valve seam, hardware/bring-up init() is NOT part of the contract (the
 * board configures the pins); the logic safes the firing line through deenergise().
 * ------------------------------------------------------------------------- */

namespace logic::actuation {

/**
 * @brief The contract an e-match must satisfy to be driven by the logic layer.
 *
 * A conforming type exposes:
 *   - energise(now_ms)    — drive the firing line active (held only during Ignite) and
 *                           record now_ms as the last-energised time.
 *   - deenergise(now_ms)  — drive the firing line inactive (also the boot-safe state) and
 *                           record now_ms as the last-deenergised time.
 *   - poll()              — sample the e-match-present input, mirror it onto the continuity
 *                           indicator, and return whether an e-match is detected.
 *   - info()              — the e-match's own EmatchInfo record (presence + firing-line
 *                           state + the last energise/deenergise ticks), kept current by
 *                           the e-match; consumers (telemetry) just read it.
 */
template <typename T>
concept Ematch = requires(T ematch, uint32_t now_ms) {
    { ematch.energise(now_ms) }   -> std::same_as<void>;
    { ematch.deenergise(now_ms) } -> std::same_as<void>;
    { ematch.poll() }             -> std::same_as<bool>;
    { ematch.info() }             -> std::same_as<::EmatchInfo>;
};

} // namespace logic::actuation
