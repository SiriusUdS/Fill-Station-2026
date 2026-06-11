#pragma once

#include <concepts>
#include <cstdint>
#include <optional>

#include "communication/protocol/telemetry/valve_info.hpp"   // ValveInfo (the valve's own info record)

/* ------------------------------------------------------------------------- *
 * Class-based valve contract for the logic layer (C++23 concept).
 *
 * The logic layer depends ONLY on the @ref Valve concept below: any type with
 * the right member functions models it, checked at compile time. There is no
 * vtable and no HAL type here, so logic commands valves without knowing about
 * servos, limit switches or GPIO.
 *
 * This replaces the previous free-function seam (open(valve_id)/close(valve_id)/
 * ...): each valve is now an OBJECT that owns its own state machine, so a board
 * holds one Valve instance per physical valve instead of indexing a hidden
 * platform-side table by id. Logic components are templated on a Valve and hold
 * a reference (or a span of references) to the instances they command. Host
 * tests inject a FakeValve that models the same concept — no separate link.
 * ------------------------------------------------------------------------- */

namespace logic::actuation {

/**
 * @brief Errors the valve interface can report.
 */
enum class ValveError {
    InternalError,  /**< Unspecified failure in the underlying actuator. */
};

/**
 * @brief The contract a valve must satisfy to be commanded by the logic layer.
 *
 * A conforming type exposes:
 *   - open()             — move to fully open;           returns nullopt or a ValveError.
 *   - close()            — move to fully closed;         returns nullopt or a ValveError.
 *   - setOpenPercent(p)  — move to a proportional hold;  returns nullopt or a ValveError.
 *                          p is 0 (closed) .. 100 (open); out-of-range is clamped.
 *   - info()             — the valve's own ValveInfo record (state + status + set
 *                          value). The valve owns it and keeps it up to date as it
 *                          operates; consumers just read it (no separate state()
 *                          getter — the state lives in info()).
 *
 * Conformance is structural (no inheritance) and checked at the point of use,
 * or eagerly via a static_assert next to the concrete type.
 */
template <typename T>
concept Valve = requires(T valve, float percent) {
    { valve.open() }                   -> std::same_as<std::optional<ValveError>>;
    { valve.close() }                  -> std::same_as<std::optional<ValveError>>;
    { valve.setOpenPercent(percent) }  -> std::same_as<std::optional<ValveError>>;
    { valve.info() }                   -> std::same_as<::ValveInfo>;
};

} // namespace logic::actuation
