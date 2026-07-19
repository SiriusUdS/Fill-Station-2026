#pragma once

#include <concepts>
#include <cstdint>

#include "communication/protocol/devices/solenoid/solenoid_info.hpp"   // SolenoidInfo (the telemetry unit)

/* ------------------------------------------------------------------------- *
 * Class-based solenoid-valve contract for the logic layer (C++23 concept).
 *
 * The control + telemetry layers depend ONLY on the @ref Solenoid concept below: any
 * type with the right members models it, checked at compile time. There is no vtable
 * and no HAL type here — logic opens/closes the solenoid without knowing about the GPIO
 * line or its polarity. The firmware models it with a GpioSolenoid composed over a single
 * digital-out seam; host tests inject a FakeSolenoid.
 *
 * Like the Valve / Ematch seams, hardware/bring-up init() is NOT part of the contract
 * (the board configures the pins); the logic safes the solenoid through close().
 * open()/close() are idempotent and carry now_ms, so they can be driven every tick
 * (the FCU enforces "open only while the SolenoidValve flag is set AND in the Unsafe
 * state"); the edge ticks in info() update only on an actual open/close transition.
 * ------------------------------------------------------------------------- */

namespace logic::actuation {

/**
 * @brief The contract a solenoid valve must satisfy to be driven by the logic layer.
 *
 * A conforming type exposes:
 *   - open(now_ms)   — drive the solenoid open (coil energised); stamp now_ms on the
 *                      closed->open edge. Idempotent (no-op stamp if already open).
 *   - close(now_ms)  — drive the solenoid closed (also the boot-safe state); stamp now_ms
 *                      on the open->closed edge. Idempotent.
 *   - info()         — the solenoid's own SolenoidInfo record (open/closed state + last
 *                      open/close ticks), kept current; telemetry reads it.
 */
template <typename T>
concept Solenoid = requires(T solenoid, uint32_t now_ms) {
    { solenoid.open(now_ms) }  -> std::same_as<void>;
    { solenoid.close(now_ms) } -> std::same_as<void>;
    { solenoid.info() }        -> std::same_as<::SolenoidInfo>;
};

} // namespace logic::actuation
