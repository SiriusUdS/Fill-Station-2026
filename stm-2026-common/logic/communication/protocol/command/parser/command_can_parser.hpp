#pragma once

#include <optional>

#include "command/command.hpp"
#include "communication/interfaces/can.hpp"   // logic::communication::CanFrame

namespace logic::communication::command {

/**
 * @brief  Parse a received CAN frame into a Command.
 * @param  frame  The CAN frame; its @c id overlays a CanHeader.
 * @return The command, or std::nullopt if the messageID is not a known command.
 */
[[nodiscard]] std::optional<Command> fromCan(const CanFrame& frame);

} // namespace logic::communication::command
