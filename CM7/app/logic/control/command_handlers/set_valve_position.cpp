#include "control/command_handlers/set_valve_position.hpp"

#include "control/command_handlers/execute_set_valve_position.hpp"

namespace logic::control::command_handlers {

using logic::communication::command::CommandType;

namespace {

/* Accept only the defined valve commands; reject any out-of-range value. */
[[nodiscard]] bool isValidAction(ValveCommand action)
{
    return action == ValveCommand::Open ||
           action == ValveCommand::Close ||
           action == ValveCommand::SetOpenedPct;
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
