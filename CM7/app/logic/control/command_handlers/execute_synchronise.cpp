#include "control/command_handlers/execute_synchronise.hpp"

namespace logic::control::command_handlers {

void execute_synchronise(uint32_t network_time_ms)
{
    (void)network_time_ms;
    // TODO: set the clock offset from the network time. Stub for now —
    // handleSynchronise is wired to call this, but it has no effect yet.
}

} // namespace logic::control::command_handlers
