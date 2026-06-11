#pragma once

#include "command/command.hpp"   // logic::communication::command::Command

/* ------------------------------------------------------------------------- *
 * SetState command handler — the most complex command. It does not just decide
 * a transition: it effects it end to end. From the controller's point of view a
 * SetState simply "changes the state", and that is all.
 *
 * handleSetState reads the current state, validates the requested transition,
 * runs the per-transition action for that exact from -> to pair, and THEN
 * commits the new state. Action-before-commit is deliberate: some transitions
 * also adjust flags (e.g. switching data logging to fast mode — not implemented
 * yet), so the action and the commit are tightly coupled and live together.
 *
 * The per-transition actions are NOT bundled into this file — there is one file
 * per transition that has an action (e.g. set_state_transitions/activate_igniter
 * for UNSAFE -> IGNITE); handleSetState dispatches directly to the matching one.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

using logic::communication::command::Command;

/**
 * @brief  Effect a SetState command: validate, run the transition action, and
 *         commit the new state to logic::control::persistent_state.
 * @param  cmd  The parsed command; its payload is a SetStateFrame.
 * @return true if the command produced a state transition (committed), false if
 *         it was rejected — not a SetState, unknown requested state id, or an
 *         illegal transition from the current state.
 *
 * The current state is read from persistent_state, so the caller needs only to
 * pass the command. Encodes the filling-station transition table (the single
 * source of truth for which operator-commanded transitions are legal).
 * SetStateFrame::flags are not acted on yet (TODO: reset / start-logging /
 * get-system-after).
 */
bool handleSetState(const Command& cmd);

} // namespace logic::control::command_handlers
