/* ------------------------------------------------------------------------- *
 * Unit tests for the per-command handlers (logic::control::command_handlers).
 *
 * Each handler validates a Command and runs its action. SetState is the
 * stateful one — it commits the transition through control::persistent_state —
 * so those tests seed and read back the persisted state. The other handlers are
 * exercised through their accept/reject return; their actions are inert stubs.
 *
 * State admissibility ("may this command run in the current state?") is the
 * forthcoming main handler's job, not these handlers', so it is not tested here.
 * ------------------------------------------------------------------------- */

#include "control/command_handlers/command_handlers.hpp"

#include "command/command.hpp"
#include "control/persistent_state.hpp"
#include "control/states.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

namespace {

using logic::communication::command::Command;
using logic::communication::command::CommandType;
using logic::control::State;
namespace ch = logic::control::command_handlers;

Command makeCommand(CommandType type)
{
    Command cmd{};
    cmd.type = type;
    return cmd;
}

Command makeSetState(uint8_t requestedId, uint8_t flags = 0)
{
    Command cmd = makeCommand(CommandType::SetState);
    const SetStateFrame frame{flags, requestedId};
    std::memcpy(cmd.payload.data(), &frame, sizeof(frame));
    return cmd;
}

Command makeSetValve(uint8_t action, uint16_t value = 0)
{
    Command cmd = makeCommand(CommandType::SetValvePosition);
    const SetValvePositionFrame frame{action, 0, value};
    std::memcpy(cmd.payload.data(), &frame, sizeof(frame));
    return cmd;
}

constexpr uint8_t stateId(State s) { return static_cast<uint8_t>(s); }

/* ---- SetState (stateful: commits through persistent_state) --------------- */

class SetStateHandler : public ::testing::Test {
protected:
    /* Start from cold Backup SRAM each test so persisted state never leaks. */
    void SetUp() override
    {
        logic::control::persistent_state = logic::control::PersistentState{};
    }
    void setCurrent(State s) { logic::control::persistent_state.saveState(s); }
    State current() const { return logic::control::persistent_state.fill_state; }
};

TEST_F(SetStateHandler, LegalTransitionCommitsNewState)
{
    setCurrent(State::Safe);
    EXPECT_TRUE(ch::handleSetState(makeSetState(stateId(State::Unsafe))));
    EXPECT_EQ(current(), State::Unsafe);
}

TEST_F(SetStateHandler, IllegalTransitionIsRejectedAndStateUnchanged)
{
    setCurrent(State::Safe);
    EXPECT_FALSE(ch::handleSetState(makeSetState(stateId(State::Ignite))));
    EXPECT_EQ(current(), State::Safe);
}

TEST_F(SetStateHandler, UnknownRequestedStateIsRejected)
{
    setCurrent(State::Safe);
    EXPECT_FALSE(ch::handleSetState(makeSetState(0xFF)));
    EXPECT_EQ(current(), State::Safe);
}

TEST_F(SetStateHandler, SelfTransitionIsRejected)
{
    setCurrent(State::Safe);
    EXPECT_FALSE(ch::handleSetState(makeSetState(stateId(State::Safe))));
    EXPECT_EQ(current(), State::Safe);
}

TEST_F(SetStateHandler, UnsafeToIgniteIsHandled)
{
    setCurrent(State::Unsafe);
    EXPECT_TRUE(ch::handleSetState(makeSetState(stateId(State::Ignite))));
    EXPECT_EQ(current(), State::Ignite);  // activate_igniter ran (inert stub)
}

TEST_F(SetStateHandler, WrongCommandTypeIsRejected)
{
    setCurrent(State::Safe);
    EXPECT_FALSE(ch::handleSetState(makeCommand(CommandType::Ping)));
    EXPECT_EQ(current(), State::Safe);
}

/* ---- Ping ---------------------------------------------------------------- */

TEST(PingHandler, PingIsHandled)
{
    EXPECT_TRUE(ch::handlePing(makeCommand(CommandType::Ping)));
}

TEST(PingHandler, NonPingIsRejected)
{
    EXPECT_FALSE(ch::handlePing(makeCommand(CommandType::SetState)));
}

/* ---- SetValvePosition ---------------------------------------------------- */

TEST(SetValveHandler, ValidActionIsHandled)
{
    EXPECT_TRUE(ch::handleSetValvePosition(makeSetValve(VALVE_OPEN, 1234)));
}

TEST(SetValveHandler, EmptyActionIsRejected)
{
    EXPECT_FALSE(ch::handleSetValvePosition(makeSetValve(0)));
}

TEST(SetValveHandler, CombinedActionIsRejected)
{
    EXPECT_FALSE(ch::handleSetValvePosition(makeSetValve(VALVE_OPEN | VALVE_CLOSE)));
}

TEST(SetValveHandler, WrongTypeIsRejected)
{
    EXPECT_FALSE(ch::handleSetValvePosition(makeCommand(CommandType::Ping)));
}

/* ---- Synchronise --------------------------------------------------------- */

TEST(SynchroniseHandler, SynchroniseIsHandled)
{
    Command cmd = makeCommand(CommandType::Synchronise);
    cmd.timestamp_ms = 42;
    EXPECT_TRUE(ch::handleSynchronise(cmd));
}

TEST(SynchroniseHandler, WrongTypeIsRejected)
{
    EXPECT_FALSE(ch::handleSynchronise(makeCommand(CommandType::Ping)));
}

/* ---- Main handler (gate + dispatch) -------------------------------------- *
 * Skeleton: canExecute is permissive for now, so these only check the dispatch
 * wiring and the unknown-type guard — admissibility-matrix tests come later. */

class MainHandler : public ::testing::Test {
protected:
    void SetUp() override
    {
        logic::control::persistent_state = logic::control::PersistentState{};
    }
    void setCurrent(State s) { logic::control::persistent_state.saveState(s); }
    State current() const { return logic::control::persistent_state.fill_state; }
};

TEST_F(MainHandler, DispatchesSetStateAndCommits)
{
    setCurrent(State::Safe);
    EXPECT_TRUE(ch::handleCommand(makeSetState(stateId(State::Unsafe))));
    EXPECT_EQ(current(), State::Unsafe);
}

TEST_F(MainHandler, DispatchesPing)
{
    setCurrent(State::Safe);
    EXPECT_TRUE(ch::handleCommand(makeCommand(CommandType::Ping)));
}

TEST_F(MainHandler, UnknownCommandTypeIsDropped)
{
    setCurrent(State::Safe);
    EXPECT_FALSE(ch::handleCommand(makeCommand(static_cast<CommandType>(0x7F))));
}

TEST_F(MainHandler, CanExecuteIsPermissiveForKnownCommands)
{
    EXPECT_TRUE(ch::canExecute(CommandType::Ping, State::Safe));
    EXPECT_TRUE(ch::canExecute(CommandType::SetValvePosition, State::Test));
    EXPECT_FALSE(ch::canExecute(static_cast<CommandType>(0x7F), State::Safe));
}

} // namespace
