#pragma once

/* ------------------------------------------------------------------------- *
 * Independent watchdog (IWDG) feed seam.
 *
 * The board feeds its hardware watchdog from two places, and which ones are live
 * depends on the state:
 *   - a serviced Ping (the ~1 Hz GS heartbeat — the FCU directly from the GS, the ECU
 *     from the FCU's bridged ping), in EVERY state. See Control::handlePing.
 *   - the main loop, every tick, ONLY while in Safe. See Control::serviceWatchdog.
 *
 * So outside Safe a board that cannot service a Ping for the IWDG timeout (~30 s) is
 * treated as dead and is reset. Inside Safe — the state in which people are allowed
 * near the system, and the one held through assembly when the link is expected to be
 * gone for minutes — the main-loop feed keeps the board alive through comm loss.
 *
 * This is a DECLARATION only: the definition is supplied by the platform DIL.
 *   - Firmware: refreshes the STM32 IWDG peripheral (platform::system::watchdog),
 *     which must be bound (watchdog::init) after MX_IWDG_Init before the first kick.
 *   - Host tests: a no-op definition in the test support (no peripheral).
 * ------------------------------------------------------------------------- */

namespace logic::control::watchdog {

/** @brief Refresh the independent watchdog, restarting its timeout. Called from
 *         Control::handlePing on every serviced Ping (the ~1 Hz GS heartbeat), and
 *         from Control::serviceWatchdog every tick while the board is in Safe. */
void kick();

} // namespace logic::control::watchdog
