#include "control/command_handlers/main_handler.hpp"

#include "control/persistent_state.hpp"   // logic::control::persistent_state

#include "control/command_handlers/ping.hpp"
#include "control/command_handlers/set_state.hpp"
#include "control/command_handlers/set_valve_position.hpp"
#include "control/command_handlers/synchronise.hpp"

namespace logic::control::command_handlers {

bool canExecute(CommandType type, State current)
{
    (void)current;
    /* Skeleton: every known command is admissible in every state for now. This
       is the single place to add per-state gating as the policy firms up (e.g.
       SetValvePosition only in TEST/UNSAFE). Expected to be reworked a lot. */
    switch (type) {
        case CommandType::Ping:
        case CommandType::SetState:
        case CommandType::SetValvePosition:
        case CommandType::Synchronise:
            return true;
    }
    return false;   // unknown command type
}

bool handleCommand(const Command& cmd)
{
    if (!canExecute(cmd.type, persistent_state.fill_state)) {
        return false;   // inadmissible in the current state — handler never runs
    }

    switch (cmd.type) {
        case CommandType::Ping:             return handlePing(cmd);
        case CommandType::SetState:         return handleSetState(cmd);
        case CommandType::SetValvePosition: return handleSetValvePosition(cmd);
        case CommandType::Synchronise:      return handleSynchronise(cmd);
    }
    return false;   // unknown command type
}

} // namespace logic::control::command_handlers
