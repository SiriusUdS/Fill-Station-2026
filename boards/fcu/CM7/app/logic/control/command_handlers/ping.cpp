#include "control/command_handlers/ping.hpp"

#include "control/command_handlers/execute_ping.hpp"

namespace logic::control::command_handlers {

using logic::communication::command::CommandType;

bool handlePing(const Command& cmd)
{
    if (cmd.type != CommandType::Ping) {
        return false;
    }
    execute_ping();
    return true;
}

} // namespace logic::control::command_handlers
