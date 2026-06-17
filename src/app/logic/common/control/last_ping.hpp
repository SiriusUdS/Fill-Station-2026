#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Time the board last received a Ping (the GS heartbeat) — runtime comms-liveness state.
 *
 * Control::handlePing() stamps this with now_ms every time a Ping reaches the board (the FCU
 * directly from the GS; the ECU from the FCU's bridged ping). Telemetry reads it when building the
 * ExtendedSystemStateBase to publish `seconds_since_last_ping`, so the GS can see, per board, how
 * stale the heartbeat is from that board's own clock.
 *
 * Process-wide and NOT battery-backed (like state_entered_ms): it resets to 0 on every boot, so
 * "seconds since last ping" counts from boot until the first Ping arrives.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/** @brief Millisecond tick at which the board last received a Ping. 0 at boot; stamped by
 *         Control::handlePing on every received Ping. */
inline uint32_t last_ping_ms = 0;

} // namespace logic::control
