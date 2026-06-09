#pragma once

#include "command/set_valve_position.hpp"   // SetValvePositionFrame

/* ------------------------------------------------------------------------- *
 * SetValvePosition action — run by handleSetValvePosition once the command is
 * validated. Kept in its own file so the handler stays a thin
 * validate-then-execute step.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

/**
 * @brief Drive a valve per a validated SetValvePosition action.
 * @param action  The validated valve action (OPEN / CLOSE / MANUAL + value).
 *
 * Stub: currently inert. TODO: frame and queue the valve command over CAN to the
 * engine board.
 */
void execute_set_valve_position(const SetValvePositionFrame& action);

} // namespace logic::control::command_handlers
