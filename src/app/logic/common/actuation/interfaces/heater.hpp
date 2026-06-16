#pragma once

#include <concepts>
#include <cstdint>

#include "communication/protocol/devices/heater/heater_info.hpp"   // HeaterInfo (the telemetry unit)

/* ------------------------------------------------------------------------- *
 * Class-based heater contract for the logic layer (C++23 concept).
 *
 * The control + telemetry layers depend ONLY on the @ref Heater concept below: any type
 * with the right members models it, checked at compile time. There is no vtable and no
 * HAL type here — logic turns the heater on/off without knowing about the GPIO line or its
 * polarity. The firmware models it with a GpioHeater over a single digital-out seam; host
 * tests inject a FakeHeater.
 *
 * Like the Solenoid seam but simpler: a heater is a bare on/off output, so it has no
 * detect input and no continuity LED — and therefore no poll(). Hardware/bring-up init()
 * is NOT part of the contract (the board configures the pin); the logic safes the heater
 * through off(). on()/off() are idempotent and carry now_ms, so they can be driven every
 * tick (the FCU drives the heater straight from its control flag); the edge ticks in
 * info() update only on an actual on/off transition.
 * ------------------------------------------------------------------------- */

namespace logic::actuation {

/**
 * @brief The contract a heater must satisfy to be driven by the logic layer.
 *
 * A conforming type exposes:
 *   - on(now_ms)   — drive the heater on (output energised); stamp now_ms on the
 *                    off->on edge. Idempotent (no-op stamp if already on).
 *   - off(now_ms)  — drive the heater off (also the boot-safe state); stamp now_ms on the
 *                    on->off edge. Idempotent.
 *   - info()       — the heater's own HeaterInfo record (on/off state + last on/off ticks),
 *                    kept current; telemetry reads it.
 */
template <typename T>
concept Heater = requires(T heater, uint32_t now_ms) {
    { heater.on(now_ms) }  -> std::same_as<void>;
    { heater.off(now_ms) } -> std::same_as<void>;
    { heater.info() }      -> std::same_as<::HeaterInfo>;
};

} // namespace logic::actuation
