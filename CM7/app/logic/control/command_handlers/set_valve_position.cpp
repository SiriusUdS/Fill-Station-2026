#include "control/command_handlers/set_valve_position.hpp"

#include "control/command_handlers/execute_set_valve_position.hpp"

namespace logic::control::command_handlers {

using logic::communication::command::CommandType;

namespace {

/* Exactly one action bit must be set — OPEN, CLOSE and MANUAL are mutually
   exclusive intents. Rejects an empty mask or a combination. */
[[nodiscard]] bool isValidAction(uint8_t action)
{
    return action == VALVE_OPEN || action == VALVE_CLOSE || action == VALVE_MANUAL;
}

} // namespace

bool handleSetValvePosition(const Command& cmd)
{
    if (cmd.type != CommandType::SetValvePosition) {
        return false;
    }

    const auto* frame = reinterpret_cast<const SetValvePositionFrame*>(cmd.payload.data());
    if (!isValidAction(frame->action)) {
        return false;
    }

    execute_set_valve_position(*frame);
    return true;
}

} // namespace logic::control::command_handlers
