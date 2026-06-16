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
            // Launch -> Abort is available at all times; Launch -> Safe is structurally
            // legal but additionally locked out for the early burn (see
            // isTransitionLockedOut / LAUNCH_TO_SAFE_LOCKOUT_MS).
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

/**
 * @brief How long Launch must be held before it may be safed. An abort
 *        (Launch -> Abort) is NOT subject to this — it is available at all times.
 */
inline constexpr uint32_t LAUNCH_TO_SAFE_LOCKOUT_MS = 20000;

/**
 * @brief Time-gated leg of the transition policy: is a structurally-legal edge
 *        currently locked out by a minimum-dwell requirement?
 *
 * Layered on top of isTransitionAllowed: an edge must be BOTH allowed (legal in the
 * table) AND not locked out here to commit. Today the only gated edge is
 * Launch -> Safe, blocked for the first LAUNCH_TO_SAFE_LOCKOUT_MS after entering
 * Launch so the vehicle cannot be safed mid-burn. Launch -> Abort is deliberately
 * not gated, so an abort stays available at every instant of launch.
 *
 * @param current      The state being left.
 * @param requested    The state being requested.
 * @param ms_in_state  Elapsed time in `current` (now_ms - state_entered_ms).
 * @return true if the edge is legal-but-locked-out right now (caller must refuse it).
 */
[[nodiscard]] inline bool isTransitionLockedOut(State current, State requested,
                                                uint32_t ms_in_state)
{
    if (current == State::Launch && requested == State::Safe) {
        return ms_in_state < LAUNCH_TO_SAFE_LOCKOUT_MS;
    }
    return false;
}

} // namespace logic::control
