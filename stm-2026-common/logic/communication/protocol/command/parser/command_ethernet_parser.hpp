#pragma once

#include <cstdint>
#include <optional>
#include <span>

#include "command/command.hpp"

namespace logic::communication::command {

/**
 * @brief  Parse a UDP/Ethernet datagram into a Command.
 * @param  frame  Full datagram: a 12-byte EthernetHeader followed by the command
 *                payload bytes.
 * @return The command, or std::nullopt if the frame is too short or its
 *         payloadID is not a known command.
 */
[[nodiscard]] std::optional<Command> fromEthernet(std::span<const uint8_t> frame);

} // namespace logic::communication::command
