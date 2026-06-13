/* ------------------------------------------------------------------------- *
 * Unit tests for the FCU control layer (logic::fcu::Control).
 *
 * Control is the receive side: parse an inbound datagram into a Command, gate it,
 * dispatch, and run the action — committing a state change, actuating a local
 * valve, or forwarding to the ECU through the communication layer. These tests
 * drive it at its public boundary (onDatagram / onPong / watchdog) over the real
 * Communication layer wired to the FakeBus, and assert the effects: persisted
 * state, valve calls, and what was framed onto CAN / UDP.
 *
 * The full per-state transition matrix and the watchdog timing are exercised
 * end-to-end in fcu_controller_test; here we cover the command-dispatch semantics
 * the folded-in command_handlers module used to own (validation, ECU forwarding,
 * the pong relay).
 * ------------------------------------------------------------------------- */

#include "control.hpp"
#include "communication.hpp"

#include "support/fakes.hpp"
#include "support/fake_valve.hpp"

#include "communication/command/command.hpp"                 // CommandType
#include "communication/protocol/command/set_state.hpp"      // SetStateFrame
#include "communication/protocol/command/set_valve_position.hpp"  // SetValvePositionFrame, ValveCommand
#include "system/valves/fcu.hpp"                              // FcuValves
#include "control/persistent_state.hpp"
#include "system/state.hpp"
#include "framing/can_header.hpp"
#include "framing/ethernet_header.hpp"
#include "framing/payload_type.hpp"
#include "response/response_type.hpp"
#include "system/board_id.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace command = logic::communication::command;
using logic::control::State;

namespace {

/* The control watchdog window (Control::detail::RX_WATCHDOG_MS) — an internal of
   Control, mirrored here so a test can read at exactly the abort threshold. */
constexpr uint32_t RX_WATCHDOG_MS = 500;

/* Build a UDP datagram: a 12-byte EthernetHeader (a command addressed to `target`)
   followed by `body`. */
std::vector<uint8_t> makeCommand(command::CommandType type, BoardId target,
                                 const uint8_t* body, std::size_t bodyLen)
{
    EthernetHeader header{};
    header.sender_id          = static_cast<uint8_t>(BoardId::GsControl);
    header.target_id          = static_cast<uint8_t>(target);
    header.payload_type       = static_cast<uint8_t>(PayloadType::Command);
    header.payload_id         = static_cast<uint8_t>(type);
    header.payload_size_bytes = static_cast<uint16_t>(bodyLen);

    std::vector<uint8_t> buf(sizeof(EthernetHeader) + bodyLen);
    std::memcpy(buf.data(), &header, sizeof(EthernetHeader));
    if (bodyLen != 0) {
        std::memcpy(buf.data() + sizeof(EthernetHeader), body, bodyLen);
    }
    return buf;
}

std::vector<uint8_t> makeSetState(State requested, BoardId target = BoardId::FillingStation)
{
    const SetStateFrame body{0, static_cast<uint8_t>(requested)};
    return makeCommand(command::CommandType::SetState, target,
                       reinterpret_cast<const uint8_t*>(&body), sizeof(body));
}

std::vector<uint8_t> makeSetValve(FcuValves valve, ValveCommand action, uint8_t value = 0)
{
    const SetValvePositionFrame body{valve, action, value};
    return makeCommand(command::CommandType::SetValvePosition, BoardId::FillingStation,
                       reinterpret_cast<const uint8_t*>(&body), sizeof(body));
}

std::vector<uint8_t> makePing()
{
    return makeCommand(command::CommandType::Ping, BoardId::FillingStation, nullptr, 0);
}

class ControlTest : public ::testing::Test {
protected:
    FakeEthernet eth_;
    FakeCan      can_;
    logic::fcu::Communication<FakeEthernet, FakeCan> comm_{eth_, can_};
    FakeValve    fill_valve_;
    FakeValve    dump_valve_;
    logic::fcu::Control<FakeValve, logic::fcu::Communication<FakeEthernet, FakeCan>>
                 control_{fill_valve_, dump_valve_, comm_};
    uint32_t     now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        logic::control::persistent_state = logic::control::PersistentState{};
        comm_.init();
        control_.init();
        clearValveCalls();   // discard the boot-safing close() so command tests start clean
    }

    void clearValveCalls()
    {
        fill_valve_.open_calls = fill_valve_.close_calls = fill_valve_.percent_calls = 0;
        dump_valve_.open_calls = dump_valve_.close_calls = dump_valve_.percent_calls = 0;
    }

    void setCurrent(State s) { logic::control::persistent_state.saveState(s); }
    State current() const { return logic::control::persistent_state.fill_state; }

    void deliver(const std::vector<uint8_t>& datagram)
    {
        control_.onDatagram(std::span<const uint8_t>(datagram), ++now_ms_);
    }
};

/* ---- Ping (Gs->Fcu->Ecu) ------------------------------------------------- */

TEST_F(ControlTest, PingIsForwardedToEcuOverCan)
{
    setCurrent(State::Safe);
    deliver(makePing());

    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader header;
    header.code = bus().can_tx.front().id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::Engine);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Command);
    EXPECT_EQ(static_cast<command::CommandType>(header.frame.payload_id), command::CommandType::Ping);
    EXPECT_EQ(bus().can_tx.front().length, 0);
}

/* ---- Reliable command (retry until the response echoes the seq) ----------- */

TEST_F(ControlTest, ReliablePingRetransmitsUntilGivingUp)
{
    setCurrent(State::Safe);
    deliver(makePing());                          // original send
    ASSERT_EQ(bus().can_tx.size(), 1u);

    // No Pong arrives: each elapsed timeout triggers one resend, up to MAX_COMMAND_RETRIES.
    for (uint8_t i = 1; i <= logic::fcu::detail::MAX_COMMAND_RETRIES; ++i) {
        now_ms_ += logic::fcu::detail::COMMAND_TIMEOUT_MS;
        control_.servicePending(now_ms_);
        EXPECT_EQ(bus().can_tx.size(), 1u + i);    // original + i retries
    }
    // After the last retry it gives up — no further sends however long we wait.
    now_ms_ += logic::fcu::detail::COMMAND_TIMEOUT_MS;
    control_.servicePending(now_ms_);
    EXPECT_EQ(bus().can_tx.size(), 1u + logic::fcu::detail::MAX_COMMAND_RETRIES);
}

TEST_F(ControlTest, PongEchoingTheSeqStopsRetransmission)
{
    setCurrent(State::Safe);
    deliver(makePing());
    ASSERT_EQ(bus().can_tx.size(), 1u);

    CanHeader sent;
    sent.code = bus().can_tx.front().id;
    control_.onPong(static_cast<uint8_t>(sent.frame.seq), ++now_ms_);  // ack the exact ping

    now_ms_ += logic::fcu::detail::COMMAND_TIMEOUT_MS * 4;
    control_.servicePending(now_ms_);
    EXPECT_EQ(bus().can_tx.size(), 1u);   // cleared by the Pong — never resent
}

TEST_F(ControlTest, PongWithWrongSeqDoesNotStopRetransmission)
{
    setCurrent(State::Safe);
    deliver(makePing());                                   // seq 0
    control_.onPong(/*seq=*/7, ++now_ms_);                 // a stale/mismatched Pong

    now_ms_ += logic::fcu::detail::COMMAND_TIMEOUT_MS;
    control_.servicePending(now_ms_);
    EXPECT_EQ(bus().can_tx.size(), 2u);   // still pending -> resent
}

/* ---- SetState ------------------------------------------------------------ */

TEST_F(ControlTest, LegalStateTransitionCommits)
{
    setCurrent(State::Safe);
    deliver(makeSetState(State::Unsafe));
    EXPECT_EQ(current(), State::Unsafe);
}

TEST_F(ControlTest, IllegalStateTransitionIsRejected)
{
    setCurrent(State::Safe);
    deliver(makeSetState(State::Ignite));   // Safe -> Ignite not allowed
    EXPECT_EQ(current(), State::Safe);
}

TEST_F(ControlTest, UnknownRequestedStateIsRejected)
{
    setCurrent(State::Safe);
    deliver(makeSetState(static_cast<State>(0xFF)));
    EXPECT_EQ(current(), State::Safe);
}

TEST_F(ControlTest, CommandForAnotherBoardIsIgnored)
{
    setCurrent(State::Safe);
    deliver(makeSetState(State::Unsafe, BoardId::Engine));
    EXPECT_EQ(current(), State::Safe);
}

/* ---- Boot safing --------------------------------------------------------- */

TEST_F(ControlTest, InitDrivesLocalValvesClosed)
{
    clearValveCalls();   // drop SetUp's boot-safe, then re-run init and observe it
    control_.init();
    EXPECT_EQ(fill_valve_.close_calls, 1);
    EXPECT_EQ(dump_valve_.close_calls, 1);
    EXPECT_EQ(fill_valve_.open_calls, 0);
    EXPECT_EQ(dump_valve_.open_calls, 0);
}

/* ---- SetValvePosition (actuates a local valve) --------------------------- */

TEST_F(ControlTest, ValidValveActionActuatesLocalValve)
{
    deliver(makeSetValve(FcuValves::Fill, ValveCommand::Open));
    EXPECT_EQ(fill_valve_.open_calls, 1);
    EXPECT_EQ(dump_valve_.open_calls, 0);
}

TEST_F(ControlTest, UnknownValveActionIsRejected)
{
    deliver(makeSetValve(FcuValves::Fill, static_cast<ValveCommand>(0xFF)));
    EXPECT_EQ(fill_valve_.open_calls, 0);
    EXPECT_EQ(fill_valve_.close_calls, 0);
    EXPECT_EQ(fill_valve_.percent_calls, 0);
}

/* ---- Pong (Ecu->Fcu->Gs) ------------------------------------------------- */

TEST_F(ControlTest, PongIsRelayedToGsOverEthernet)
{
    control_.onPong(/*seq=*/0, ++now_ms_);

    ASSERT_EQ(bus().udp_tx.size(), 1u);
    EthernetHeader header;
    std::memcpy(&header, bus().udp_tx.front().payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(static_cast<BoardId>(header.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.target_id), BoardId::GsControl);
    EXPECT_EQ(static_cast<PayloadType>(header.payload_type), PayloadType::Response);
    EXPECT_EQ(static_cast<ResponseType>(header.payload_id), ResponseType::Pong);
}

/* ---- Watchdog (GS-link liveness lives in Control) ------------------------ */

TEST_F(ControlTest, WatchdogAbortsArmedStateAfterSilence)
{
    setCurrent(State::Unsafe);
    deliver(makeSetState(State::Unsafe));        // rejected transition, but refreshes last_rx
    control_.watchdog(now_ms_ + RX_WATCHDOG_MS);
    EXPECT_EQ(current(), State::Abort);
}

TEST_F(ControlTest, TrafficKeepsArmedStateAlive)
{
    setCurrent(State::Unsafe);
    deliver(makeSetState(State::Unsafe));
    control_.watchdog(now_ms_);                  // no silence elapsed
    EXPECT_EQ(current(), State::Unsafe);
}

} // namespace
