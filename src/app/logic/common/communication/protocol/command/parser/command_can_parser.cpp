/* ------------------------------------------------------------------------- *
 * CAN -> Command parser. Pure byte transform, no HAL dependency.
 *
 * The CAN messageID carries the canonical CommandType id directly (single
 * source of truth in command.hpp), so there is no per-transport id table here.
 *
 * Command -> CAN is intentionally absent: it arrives with the upcoming CAN
 * fragmentation layer (shared with telemetry), which owns splitting a payload
 * across multiple frames.
 * ------------------------------------------------------------------------- */

#include "command/parser/command_can_parser.hpp"

#include "can/can_header.hpp"

#include <bit>
#include <cstddef>
#include <cstring>

namespace logic::communication::command {

std::optional<Command> fromCan(const CanFrame& frame)
{
    const CanHeader header = std::bit_cast<CanHeader>(frame.id);

    const std::optional<CommandType> type =
        toCommandType(static_cast<uint8_t>(header.messageID));
    if (!type) {
        return std::nullopt;   // not a command (status frames, unknown ids)
    }

    Command cmd{};
    cmd.type   = *type;
    cmd.source = static_cast<uint8_t>(header.senderID);
    cmd.target = static_cast<uint8_t>(header.targetID);

    std::size_t n = payloadSize(*type);
    if (n > frame.data.size()) {
        n = frame.data.size();
    }
    std::memcpy(cmd.payload.data(), frame.data.data(), n);
    return cmd;
}

} // namespace logic::communication::command
