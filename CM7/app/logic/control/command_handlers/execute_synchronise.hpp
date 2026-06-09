#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Synchronise action — run by handleSynchronise once the command is validated.
 * Kept in its own file so the handler stays a thin validate-then-execute step.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

/**
 * @brief Adopt the network time carried by a Synchronise command.
 * @param network_time_ms  The network time (ms) to align the local clock to.
 *
 * Stub: currently inert. TODO: set the clock offset from @p network_time_ms (and
 * later synchronise device state across the network).
 */
void execute_synchronise(uint32_t network_time_ms);

} // namespace logic::control::command_handlers
