#pragma once

#include <cstdint>

#include "system/state.hpp"  // logic::control::State

/* ------------------------------------------------------------------------- *
 * Last refused SetValvePosition command — runtime diagnostic state.
 *
 * The control layer records here every SetValvePosition it rejected (an operator
 * per-valve command in a state that does not permit it — outside Unsafe — or one
 * addressed to Broadcast, which is invalid because valve commands are single-board
 * only, or one naming an unknown valve / invalid action): the valve id, the action,
 * the value requested, and the state the board was in when it refused. The telemetry
 * pipeline surfaces it in the ExtendedSystemState so the ground station can see which
 * valve command was denied and from which state. The sibling of last_refused_transition
 * (SetState) and last_refused_control_flag (SetControlFlag).
 *
 * A malformed frame (too short) or a command not addressed to us is NOT a refusal — it
 * is silently dropped, never recorded here. Process-wide and not battery-backed: resets
 * to "none refused since boot" on every boot.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/** @brief Sentinel valve id meaning "no SetValvePosition refused since boot". Real valve ids
 *         are small (FcuValves / EcuValves), so 0xFF is unused. */
inline constexpr uint8_t REFUSED_VALVE_NONE = 0xFF;

/** @brief A refused SetValvePosition: the raw valve id + action + value requested, and the
 *         state the board was in when the command was refused. */
struct RefusedValve {
    uint8_t valve;   /**< Raw valve id that was refused (REFUSED_VALVE_NONE = none). */
    uint8_t action;  /**< Raw ValveCommand requested (open / close / set-%). */
    uint8_t value;   /**< Opened-% value requested (used only by SetOpenedPct). */
    State   state;   /**< State the board was in when the command was refused. */
};

/** @brief The last SetValvePosition the control layer refused. valve == REFUSED_VALVE_NONE = none yet. */
inline RefusedValve last_refused_valve{REFUSED_VALVE_NONE, 0, 0, State::Init};

/** @brief Count of refused SetValvePosition commands since boot (wraps at 16 bits; a diagnostic).
 *         Incremented alongside last_refused_valve; surfaced in the ExtendedSystemState. */
inline uint16_t refused_valve_count = 0;

} // namespace logic::control
