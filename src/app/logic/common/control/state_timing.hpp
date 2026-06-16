#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Time the board entered its current state — runtime timing state.
 *
 * Control::transitionTo() stamps this with now_ms on every committed transition;
 * the time-gated leg of the transition policy (logic::control::isTransitionLockedOut)
 * reads `now_ms - state_entered_ms` to enforce dwell-based lockouts, e.g. Launch
 * may not be safed for the first LAUNCH_TO_SAFE_LOCKOUT_MS after entering Launch.
 *
 * Process-wide and NOT battery-backed (like last_refused_transition): it resets to
 * 0 on every boot. A warm reboot therefore restarts the dwell clock, which keeps the
 * Launch->Safe lockout conservatively re-armed rather than letting it lapse silently.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/** @brief Millisecond tick at which the board entered its current state. 0 at boot;
 *         stamped by Control::transitionTo on each committed transition. */
inline uint32_t state_entered_ms = 0;

} // namespace logic::control
