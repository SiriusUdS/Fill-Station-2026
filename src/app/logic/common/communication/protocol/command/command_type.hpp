#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "command/set_state.hpp"          // SetStateFrame         — SetState payload
#include "command/set_valve_position.hpp" // SetValvePositionFrame — SetValvePosition payload

/* ------------------------------------------------------------------------- *
 * Canonical command id (the on-wire SSOT) + per-command payload sizing.
 *
 * CommandType's numeric value is what travels on the wire on *every* transport —
 * the CAN `payload_id` and the Ethernet `payload_id` carry the identical value.
 * Flight-critical: never define a command id anywhere else, so CAN and Ethernet
 * can never disagree about what a command means.
 *
 * This header is pure wire format. The in-memory `Command` *intent* that the
 * parsers produce is logic, and lives in communication/command/command.hpp.
 * ------------------------------------------------------------------------- */

namespace logic::communication::command {

/**
 * @brief Canonical command id — the single source of truth shared by every
 *        transport. The enumerator value IS the on-wire id (CAN payload_id /
 *        Ethernet payload_id).
 */
enum class CommandType : uint8_t {
    Ping             = 0x01,  /**< Link test. Carries no payload. */
    SetState         = 0x02,  /**< Request a device state change. Payload: SetStateFrame (State TBD). */
    SetValvePosition = 0x03,  /**< Drive a valve to a position. Payload: SetValvePositionFrame. */
    Synchronise      = 0x04,  /**< Synchronise device state across the network (always includes time). No payload. */
};
// Replies to commands live in ResponseType (response_type.hpp), tagged
// PayloadType::Response — e.g. Pong answering Ping. CommandType is requests only.

/**
 * @brief Wire payload size, in bytes, for a command. One definition for all
 *        transports so a parser can never size a payload differently.
 */
[[nodiscard]] constexpr std::size_t payloadSize(CommandType type)
{
    switch (type) {
        case CommandType::Ping:             return 0;
        case CommandType::SetState:         return sizeof(SetStateFrame);
        case CommandType::SetValvePosition: return sizeof(SetValvePositionFrame);
        case CommandType::Synchronise:      return 0;
    }
    return 0;
}

/**
 * @brief Validate a raw on-wire id and recover the command it denotes.
 * @return The CommandType, or std::nullopt if @p id is not a known command.
 */
[[nodiscard]] constexpr std::optional<CommandType> toCommandType(uint8_t id)
{
    switch (static_cast<CommandType>(id)) {
        case CommandType::Ping:
        case CommandType::SetState:
        case CommandType::SetValvePosition:
        case CommandType::Synchronise:
            return static_cast<CommandType>(id);
    }
    return std::nullopt;
}

} // namespace logic::communication::command
