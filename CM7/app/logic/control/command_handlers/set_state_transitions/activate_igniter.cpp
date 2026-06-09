#include "control/command_handlers/set_state_transitions/activate_igniter.hpp"

namespace logic::control::command_handlers {

void activate_igniter()
{
    // TODO: drive the igniter. Stub for now — the UNSAFE -> IGNITE transition is
    // wired up (handleSetState dispatches here) but has no effect yet.
}

} // namespace logic::control::command_handlers
