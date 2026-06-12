#pragma once

#include "communication/command/command.hpp"   // logic::communication::command::Command

/* ------------------------------------------------------------------------- *
 * Ping command handler.
 *
 * A Ping is a link test that carries no payload. The handler validates the
 * command and runs the ping action itself (execute_ping); from the controller's
 * point of view a ping is received and handled, and that is all.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

using logic::communication::command::Command;

/**
 * @brief  Handle a Ping: validate it, then run the ping action (execute_ping).
 * @return true if @p cmd was a Ping and was handled, false otherwise.
 */
bool handlePing(const Command& cmd);

} // namespace logic::control::command_handlers
