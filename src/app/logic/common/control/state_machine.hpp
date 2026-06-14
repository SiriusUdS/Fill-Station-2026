#pragma once

#include <cstdint>
#include <optional>

#include "system/state.hpp"  // logic::control::State

/* ------------------------------------------------------------------------- *
 * Shared global state-machine policy: the on-wire State id decode and the
 * single transition table.
 *
 * This is the SAME on every board — the filling-station state machine has one
 * set of legal edges, so both the FCU and the ECU validate a commanded SetState
 * against this table. What differs per board is the ACTION run on a transition
 * (Control::onTransition): e.g. on Ignite -> Launch the ECU drives both
 * propellant valves fully open while the FCU does nothing. Keep the edges here;
 * keep the side effects in each board's Control.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/**
 * @brief Map a raw on-wire state id to the typed State, or nullopt if unknown.
 *
 * State's underlying values ARE the wire encoding, so a command can only name a
 * defined state; anything else is rejected here.
 */
[[nodiscard]] inline std::optional<State> toState(uint8_t id)
{
    switch (static_cast<State>(id)) {
        case State::Init:
        case State::Safe:
        case State::Unsafe:
        case State::Abort:
        case State::Error:
        case State::Ignite:
        case State::Launch:
        case State::Test:
            return static_cast<State>(id);
    }
    return std::nullopt;
}

/**
 * @brief The filling-station transition table: is @p requested a legal
 *        operator-commanded transition from @p current?
 *
 * Self-transitions and any pair not listed are rejected. Init -> Safe is NOT a
 * commanded edge — each board makes that move automatically once init() is done.
 *
 * Safety model: Safe means people are near the system, so ANY action is hazardous
 * — its only exit is Unsafe (declare the area clear). Abort is reachable from
 * every armed state (Unsafe / Ignite / Launch), and Safe is the way back down.
 * Error clears only to Safe.
 */
[[nodiscard]] inline bool isTransitionAllowed(State current, State requested)
{
    switch (current) {
        case State::Safe:
            // People are near: the only permitted move is to clear the area (Unsafe). No
            // abort/test/etc. while people are present — every other action is hazardous.
            return requested == State::Unsafe;
        case State::Unsafe:
            // Area clear; the system may be operated. Abort is available here and from every
            // armed state below.
            return requested == State::Safe || requested == State::Ignite ||
                   requested == State::Abort;
        case State::Ignite:
            return requested == State::Safe || requested == State::Launch ||
                   requested == State::Abort;
        case State::Launch:
            return requested == State::Safe || requested == State::Abort;
        case State::Abort:
            return requested == State::Safe;
        case State::Error:
            // A latched fault clears only down to Safe.
            return requested == State::Safe;
        case State::Test:
            // Not currently reachable (Safe no longer commands into Test); allow the way back
            // to Safe should it ever be entered.
            return requested == State::Safe;
        case State::Init:
            // Init -> Safe only: the post-boot transition. It is issued automatically once
            // init() completes (the board is never in Init when a command arrives), but it is a
            // real table edge so it routes through Control::transitionTo() like every other.
            return requested == State::Safe;
    }
    return false;
}

} // namespace logic::control
