#pragma once

#include "system/state.hpp"  // logic::control::State

/* ------------------------------------------------------------------------- *
 * Last refused state transition — runtime diagnostic state.
 *
 * Control::transitionTo() records here every state change the shared transition
 * table rejected (from -> to); the telemetry pipeline surfaces it in the
 * ExtendedSystemState so the ground station can see which commanded change was
 * denied and why a state did not move. Process-wide and not battery-backed (like
 * control_flags): it resets to {Init, Init} — "none refused since boot" — on
 * every boot.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/** @brief A state transition pair (the state we were in, and the one requested). */
struct RefusedTransition {
    State from;
    State to;
};

/** @brief The last transition Control::transitionTo() refused. {Init, Init} = none yet. */
inline RefusedTransition last_refused_transition{State::Init, State::Init};

} // namespace logic::control
