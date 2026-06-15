#pragma once

#include <cstdint>

#include "system/state.hpp"  // logic::control::State

/* ------------------------------------------------------------------------- *
 * Last refused SetControlFlag command — runtime diagnostic state.
 *
 * The control layer records here every SetControlFlag it rejected (an unknown flag
 * id, or a flag commanded in a state that does not permit it — e.g. the solenoid
 * valve outside Unsafe): the flag, the value requested, and the state the board was
 * in when it refused. The telemetry pipeline surfaces it in the ExtendedSystemState
 * so the ground station can see which flag command was denied and from which state.
 * The sibling of last_refused_transition (which covers refused SetState commands).
 * Process-wide and not battery-backed: it resets to "none refused since boot" on
 * every boot.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/** @brief Sentinel flag id meaning "no SetControlFlag refused since boot". A real flag id
 *         is in 0..15 (the 16-bit base/per-board id space), so 0xFFFF is unused. */
inline constexpr uint16_t REFUSED_CONTROL_FLAG_NONE = 0xFFFF;

/** @brief A refused SetControlFlag: the 16-bit on-wire flag id, the value requested, and the
 *         state the board was in when the command was refused. */
struct RefusedControlFlag {
    uint16_t flag;   /**< 16-bit ControlFlag id that was refused (REFUSED_CONTROL_FLAG_NONE = none). */
    uint8_t  value;  /**< The on/off value requested (0 = clear, non-zero = set). */
    State    state;  /**< State the board was in when the command was refused. */
};

/** @brief The last SetControlFlag the control layer refused. flag == REFUSED_CONTROL_FLAG_NONE = none yet. */
inline RefusedControlFlag last_refused_control_flag{REFUSED_CONTROL_FLAG_NONE, 0, State::Init};

/** @brief Count of refused SetControlFlag commands since boot (wraps at 16 bits; a diagnostic).
 *         Incremented alongside last_refused_control_flag; surfaced in the ExtendedSystemState. */
inline uint16_t refused_control_flag_count = 0;

} // namespace logic::control
