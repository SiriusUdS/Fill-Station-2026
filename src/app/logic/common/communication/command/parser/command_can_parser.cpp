/* ------------------------------------------------------------------------- *
 * CAN -> Command parser. Pure byte transform, no HAL dependency.
 *
 * The CAN payload_id carries the canonical CommandType id directly (single
 * source of truth in command.hpp), so there is no per-transport id table here.
 *
 * Command -> CAN is intentionally absent: it arrives with the upcoming CAN
 * fragmentation layer (shared with telemetry), which owns splitting a payload
 * across multiple frames.
 * ------------------------------------------------------------------------- */

#include "communication/command/parser/command_can_parser.hpp"

#include "framing/can_header.hpp"
#include "framing/payload_type.hpp"

#include <cstddef>
#include <cstring>

namespace logic::communication::command {

std::optional<Command> fromCan(const CanFrame& frame)
{
    CanHeader header{};
    header.code = frame.id;

    if (static_cast<PayloadType>(header.frame.payload_type) != PayloadType::Command) {
        return std::nullopt;   // telemetry/response or unset — not a command
    }

    const std::optional<CommandType> type =
        toCommandType(static_cast<uint8_t>(header.frame.payload_id));
    if (!type) {
        return std::nullopt;   // not a command (status frames, unknown ids)
    }

    Command cmd{};
    cmd.type   = *type;
    cmd.source = static_cast<uint8_t>(header.frame.sender_id);
    cmd.target = static_cast<uint8_t>(header.frame.target_id);
    cmd.seq    = static_cast<uint8_t>(header.frame.seq);

    std::size_t n = payloadSize(*type);
    if (n > frame.data.size()) {
        n = frame.data.size();
    }
    std::memcpy(cmd.payload.data(), frame.data.data(), n);
    return cmd;
}

} // namespace logic::communication::command
