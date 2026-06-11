#pragma once

/* ------------------------------------------------------------------------- *
 * On-transition action for UNSAFE -> IGNITE.
 *
 * One file per state transition that needs an action; handleSetState() calls
 * this one when it commits the UNSAFE -> IGNITE transition. Most transitions
 * have no such file.
 * ------------------------------------------------------------------------- */

namespace logic::control::command_handlers {

/**
 * @brief Begin the ignition sequence — run on the UNSAFE -> IGNITE transition.
 *
 * Stub: currently inert, so the transition is wired but has no effect yet.
 * TODO: energise the igniter (HAL-free — command it over the appropriate
 * interface, e.g. a CAN command to the engine board).
 */
void activate_igniter();

} // namespace logic::control::command_handlers
