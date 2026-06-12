#pragma once

#include <array>
#include <cstdint>

#include "command/command_type.hpp"   // CommandType — the on-wire command id SSOT

/* ------------------------------------------------------------------------- *
 * Internal, transport-agnostic command intent.
 *
 * Commands reach a board over different wires — UDP/Ethernet from the ground
 * station, CAN between boards — but the logic reasons about *intent*, not wire
 * bytes. `Command` is that intent; the per-transport parsers under parser/
 * convert wire <-> Command.
 *
 * This is logic, not wire format: the on-wire pieces (CommandType + the payload
 * frame structs SetStateFrame / SetValvePositionFrame) are the protocol SSOT and
 * live under protocol/command/. A Command is never sent verbatim; it is rebuilt
 * on each side from those wire pieces.
 * ------------------------------------------------------------------------- */

namespace logic::communication::command {

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

} // namespace logic::communication::command
