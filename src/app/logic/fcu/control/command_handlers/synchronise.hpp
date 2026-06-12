#pragma once

#include "communication/command/command.hpp"   // logic::communication::command::Command

/* ------------------------------------------------------------------------- *
 * Synchronise command handler.
 *
 * Synchronise carries no payload; its purpose is to align the board's clock to
 * the network time the command was stamped with. The handler validates the
 * command and runs the sync action itself (execute_synchronise). Synchronising
 * device state across the network is TBD.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

using logic::communication::command::Command;

/**
 * @brief  Handle a Synchronise: validate it, then run the sync action.
 * @return true if @p cmd was a Synchronise and was handled, false otherwise.
 */
bool handleSynchronise(const Command& cmd);

} // namespace logic::control::command_handlers
