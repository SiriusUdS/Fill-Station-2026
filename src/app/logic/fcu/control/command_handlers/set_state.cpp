#include "control/command_handlers/set_state.hpp"

#include <cstdint>
#include <optional>

#include "system/state.hpp"             // logic::control::State
#include "control/persistent_state.hpp"   // logic::control::persistent_state
#include "control/command_handlers/set_state_transitions/activate_igniter.hpp"

namespace logic::control::command_handlers {

using logic::communication::command::CommandType;

namespace {

/* Map a raw on-wire state id to the typed State, or nullopt if it is not a
   known state. State's underlying values ARE the wire encoding (see
   states.hpp), so a command can only ever name a state the protocol defines. */
[[nodiscard]] std::optional<State> toState(uint8_t id)
{
    switch (static_cast<State>(id)) {
        case State::Init:
        case State::Safe:
        case State::Unsafe:
        case State::Abort:
        case State::Error:
        case State::Ignite:
        case State::Test:
            return static_cast<State>(id);
    }
    return std::nullopt;
}

/* The filling-station transition table: is `requested` a legal
   operator-commanded transition from `current`? Self-transitions and any pair
   not listed are rejected. This is the single source of truth for the rules
   that were previously inline in fcu_controller. */
[[nodiscard]] bool isAllowed(State current, State requested)
{
    switch (current) {
        case State::Safe:
            return requested == State::Test || requested == State::Unsafe;
        case State::Test:
            return requested == State::Safe;
        case State::Unsafe:
            return requested == State::Safe || requested == State::Ignite ||
                   requested == State::Abort;
        case State::Ignite:
            return requested == State::Safe || requested == State::Abort;
        case State::Abort:
            return requested == State::Safe;
        case State::Init:
        case State::Error:
            return false;  // no operator-commanded transitions out of these
    }
    return false;
}

/* Run the action bound to this exact from -> to transition, if any. One file per
   transition under set_state_transitions/; most transitions have no action. */
void runTransitionAction(State from, State to)
{
    if (from == State::Unsafe && to == State::Ignite) {
        activate_igniter();
    }
}

} // namespace

bool handleSetState(const Command& cmd)
{
    if (cmd.type != CommandType::SetState) {
        return false;
    }

    const auto* frame = reinterpret_cast<const SetStateFrame*>(cmd.payload.data());
    const std::optional<State> requested = toState(frame->requestedID);
    if (!requested) {
        return false;   // unknown requested state id
    }

    const State current = persistent_state.fill_state;
    if (!isAllowed(current, *requested)) {
        return false;   // transition not permitted from `current`
    }

    /* Run the per-transition action first — it may adjust flags etc. — then
       commit so the whole change lands atomically from the caller's view. */
    runTransitionAction(current, *requested);
    persistent_state.saveState(*requested);
    return true;
}

} // namespace logic::control::command_handlers
