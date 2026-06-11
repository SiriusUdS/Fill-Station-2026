/* ------------------------------------------------------------------------- *
 * Ethernet (UDP) -> Command parser. Pure byte transform, no HAL dependency.
 *
 * The EthernetHeader payloadID carries the canonical CommandType id directly
 * (single source of truth in command.hpp), so there is no per-transport id
 * table here.
 * ------------------------------------------------------------------------- */

#include "command/parser/command_ethernet_parser.hpp"

#include "ethernet/ethernet_header.hpp"

#include <cstddef>
#include <cstring>

namespace logic::communication::command {

std::optional<Command> fromEthernet(std::span<const uint8_t> frame)
{
    constexpr std::size_t ETH_HEADER_BYTES = sizeof(EthernetHeader);
    if (frame.size() < ETH_HEADER_BYTES) {
        return std::nullopt;
    }

    EthernetHeader header{};
    std::memcpy(&header, frame.data(), ETH_HEADER_BYTES);

    const std::optional<CommandType> type =
        toCommandType(static_cast<uint8_t>(header.payloadID));
    if (!type) {
        return std::nullopt;   // unknown command id
    }

    const std::size_t need = payloadSize(*type);
    const std::span<const uint8_t> body = frame.subspan(ETH_HEADER_BYTES);
    if (body.size() < need) {
        return std::nullopt;   // truncated payload
    }

    Command cmd{};
    cmd.type         = *type;
    cmd.source       = static_cast<uint8_t>(header.deviceID);  // from the header — not assumed
    cmd.timestamp_ms = header.deviceTS_MS;
    std::memcpy(cmd.payload.data(), body.data(), need);
    return cmd;
}

} // namespace logic::communication::command
