#pragma once

#include "command/command.hpp"   // Command + CommandType
#include "control/states.hpp"     // logic::control::State

/* ------------------------------------------------------------------------- *
 * Main command handler — the single entry point for an inbound command.
 *
 * It gates on the current state first ("can I execute this command now?") and
 * only then dispatches to the command's own handler. If the command is not
 * admissible in the current state, its handler is never called.
 *
 * Skeleton: the admissibility matrix (canExecute) is intentionally permissive
 * for now and expected to be reworked heavily — it is the place to add the
 * per-command, per-state rules as they are pinned down.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

using logic::communication::command::Command;
using logic::communication::command::CommandType;

/**
 * @brief  May a command of @p type be executed in state @p current?
 * @return true if admissible (the gate the main handler checks before dispatch).
 *
 * Skeleton: currently returns true for every known command in every state. Add
 * per-command, per-state gating here (e.g. SetValvePosition only in TEST/UNSAFE).
 */
[[nodiscard]] bool canExecute(CommandType type, State current);

/**
 * @brief  Handle an inbound command: gate on the current state, then dispatch.
 * @param  cmd  The parsed command.
 * @return true if the command was admissible and handled, false if it was
 *         dropped (inadmissible in the current state, or an unknown type).
 *
 * The current state is read from logic::control::persistent_state.
 */
bool handleCommand(const Command& cmd);

} // namespace logic::control::command_handlers
