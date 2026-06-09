#pragma once

/* ------------------------------------------------------------------------- *
 * Umbrella header: the handlers for the FCU's current commands. Include this to
 * pull in every command handler at once; include the individual headers when
 * only one is needed.
 *
 * Each handler validates one CommandType (command.hpp) and runs its action.
 * handleCommand (main_handler) is the entry point: it gates on the current
 * state, then dispatches to the matching handler below.
 * ------------------------------------------------------------------------- */

#include "control/command_handlers/main_handler.hpp"
#include "control/command_handlers/ping.hpp"
#include "control/command_handlers/set_state.hpp"
#include "control/command_handlers/set_valve_position.hpp"
#include "control/command_handlers/synchronise.hpp"
