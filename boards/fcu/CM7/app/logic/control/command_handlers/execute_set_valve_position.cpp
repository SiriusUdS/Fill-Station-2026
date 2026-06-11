#include "control/command_handlers/execute_set_valve_position.hpp"

namespace logic::control::command_handlers {

void execute_set_valve_position(const SetValvePositionFrame& action)
{
    (void)action;
    // TODO: frame and queue the valve command over CAN to the engine board.
    // Stub for now — handleSetValvePosition is wired to call this, but it has no
    // effect yet.
}

} // namespace logic::control::command_handlers
