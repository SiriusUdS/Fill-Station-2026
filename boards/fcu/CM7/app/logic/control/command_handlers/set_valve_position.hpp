#pragma once

#include "command/command.hpp"   // logic::communication::command::Command

/* ------------------------------------------------------------------------- *
 * SetValvePosition command handler.
 *
 * Validates the requested valve action and runs the valve action itself
 * (execute_set_valve_position); from the controller's point of view the command
 * is handled. Unlike SetState, this command does not touch the state machine.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

using logic::communication::command::Command;

/**
 * @brief  Handle a SetValvePosition: validate it, then run the valve action.
 * @return true if @p cmd was a SetValvePosition with a valid action and was
 *         handled, false otherwise.
 *
 * State admissibility (whether valves may be actuated in the current state) is
 * NOT checked here — the main command handler gates that upstream, so this
 * handler runs only when the command is already admissible. It validates only
 * the action bitmask.
 */
bool handleSetValvePosition(const Command& cmd);

} // namespace logic::control::command_handlers
