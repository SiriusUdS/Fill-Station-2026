#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "command/set_state.hpp"          // SetStateFrame         — SetState payload (State TBD)
#include "command/set_valve_position.hpp" // SetValvePositionFrame — SetValvePosition payload
// Synchronise carries no payload (yet), so it has no header to include.

/* ------------------------------------------------------------------------- *
 * Internal, transport-agnostic command format for the FCU/ECU system.
 *
 * Commands reach a board over different wires — UDP/Ethernet from the ground
 * station, CAN between boards — but the logic reasons about *intent*, not wire
 * bytes. `Command` is that intent; the per-transport parsers under parser/
 * convert to/from it.
 *
 * SINGLE SOURCE OF TRUTH: @ref CommandType is the canonical command id. Its
 * numeric value is what travels on the wire on *every* transport — the CAN
 * `messageID` field and the Ethernet `payloadID` field carry the identical
 * value. This is flight-critical: never define a command id anywhere else, so
 * CAN and Ethernet can never disagree about what a command means.
 *
 * This header is transport-free (no CAN/Ethernet includes): a board that speaks
 * only one transport includes just that parser. The per-command payload layouts
 * (RequestState.h, ServoPacket.h, SyncPacket.h) are command definitions, not
 * transport detail, so they live here. Callers reinterpret_cast the payload to
 * the matching frame struct, following the ValveCmdPacket.h idiom.
 * ------------------------------------------------------------------------- */

namespace logic::communication::command {

/**
 * @brief Canonical command id — the single source of truth shared by every
 *        transport. The enumerator value IS the on-wire id (CAN messageID /
 *        Ethernet payloadID).
 */
enum class CommandType : uint8_t {
    Ping             = 0x01,  /**< Link test. Carries no payload. */
    SetState         = 0x02,  /**< Request a device state change. Payload: SetStateFrame (State TBD). */
    SetValvePosition = 0x03,  /**< Drive a valve to a position. Payload: SetValvePositionFrame. */
    Synchronise      = 0x04,  /**< Synchronise device state across the network (always includes time). No payload. */
};

/**
 * @brief A transport-agnostic command.
 *
 * @ref payload holds the raw command bytes (reinterpret as the frame struct that
 * matches @ref type); @ref CommandType::Ping leaves it unused. @ref source /
 * @ref target are node ids in whichever id space the producing transport uses;
 * a field the source transport does not carry is left zero.
 */
struct Command {
    CommandType type{};
    uint8_t  source{};                 /**< Origin node/device id. */
    uint8_t  target{};                 /**< Destination node/device id (or broadcast). */
    uint32_t timestamp_ms{};           /**< Transport timestamp / fill tick. */
    std::array<uint8_t, 8> payload{};  /**< Raw payload bytes; reinterpret per @ref type. */
};

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
