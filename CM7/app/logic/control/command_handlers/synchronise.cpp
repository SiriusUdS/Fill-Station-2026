#include "control/command_handlers/synchronise.hpp"

#include "control/command_handlers/execute_synchronise.hpp"

namespace logic::control::command_handlers {

using logic::communication::command::CommandType;

bool handleSynchronise(const Command& cmd)
{
    if (cmd.type != CommandType::Synchronise) {
        return false;
    }
    execute_synchronise(cmd.timestamp_ms);
    return true;
}

} // namespace logic::control::command_handlers
