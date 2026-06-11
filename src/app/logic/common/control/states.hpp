#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Global filling-station state machine states.
 *
 * The typed C++ handle the logic layer passes around (state machine,
 * persistent state, commands). The underlying values ARE the on-the-wire
 * encoding shared with the ground station, so they must stay in sync with the
 * filling-station protocol — keep these numbers matching the agreed wire
 * values when the protocol changes.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/**
 * @brief The board's global state-machine state.
 *
 * Underlying type is uint8_t to match the wire/telemetry encoding exactly.
 */
enum class State : uint8_t {
    Init   = 0x00,  /**< Power-on / boot init. */
    Safe   = 0x01,  /**< Safe, valves in known-safe positions. */
    Unsafe = 0x02,  /**< Armed / propellant operations permitted. */
    Abort  = 0x03,  /**< Abort sequence engaged. */
    Error  = 0x04,  /**< Fault latched. */
    Ignite = 0x05,  /**< Ignition sequence. */
    Test   = 0x06,  /**< Bench/diagnostics. */
};

} // namespace logic::control
