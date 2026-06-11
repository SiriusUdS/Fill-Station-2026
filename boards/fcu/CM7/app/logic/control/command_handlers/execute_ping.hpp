#pragma once

/* ------------------------------------------------------------------------- *
 * Ping action — run by handlePing once the command is validated. Kept in its
 * own file so the handler stays a thin validate-then-execute step.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

/**
 * @brief Answer a Ping link test.
 *
 * Stub: currently inert. TODO: emit an immediate system/heartbeat reply (today
 * the board already heartbeats every tick, so a ping is implicitly answered).
 */
void execute_ping();

} // namespace logic::control::command_handlers
