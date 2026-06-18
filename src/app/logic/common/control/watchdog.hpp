#pragma once

/* ------------------------------------------------------------------------- *
 * Independent watchdog (IWDG) feed seam.
 *
 * The board feeds its hardware watchdog from exactly ONE place: a serviced Ping
 * (the ~1 Hz GS heartbeat — the FCU directly from the GS, the ECU from the FCU's
 * bridged ping). A board that cannot service a Ping for the IWDG timeout (~30 s)
 * is treated as dead and is reset by the IWDG. See Control::handlePing.
 *
 * This is a DECLARATION only: the definition is supplied by the platform DIL.
 *   - Firmware: refreshes the STM32 IWDG peripheral (platform::system::watchdog),
 *     which must be bound (watchdog::init) after MX_IWDG_Init before the first kick.
 *   - Host tests: a no-op definition in the test support (no peripheral).
 * ------------------------------------------------------------------------- */

namespace logic::control::watchdog {

/** @brief Refresh the independent watchdog, restarting its timeout. Called from
 *         Control::handlePing on every serviced Ping (the ~1 Hz GS heartbeat). */
void kick();

} // namespace logic::control::watchdog
