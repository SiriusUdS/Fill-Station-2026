/* ------------------------------------------------------------------------- *
 * Unit tests for the FCU control layer (logic::fcu::Control).
 *
 * Control is the receive side: parse an inbound datagram into a Command, gate it,
 * dispatch, and run the action — committing a state change, actuating a local
 * valve, or forwarding to the ECU through the communication layer. These tests
 * drive it at its public boundary (onDatagram / onResponse) over the real
 * Communication layer wired to the FakeBus, and assert the effects: persisted
 * state, valve calls, and what was framed onto CAN / UDP.
 *
 * The full per-state transition matrix is exercised
 * end-to-end in fcu_controller_test; here we cover the command-dispatch semantics
 * the folded-in command_handlers module used to own (validation, ECU forwarding,
 * the pong relay).
 * ------------------------------------------------------------------------- */

#include "control.hpp"
#include "communication.hpp"

#include "support/fakes.hpp"   // the standard set of host test doubles

#include "communication/command/command.hpp"                 // CommandType
#include "communication/protocol/command/set_state.hpp"      // SetStateFrame
#include "communication/protocol/command/set_valve_position.hpp"  // SetValvePositionFrame, ValveCommand
#include "communication/protocol/command/set_control_flag.hpp"    // ControlFlag, SetControlFlagFrame
#include "system/valves/fcu.hpp"                              // FcuValves
#include "control/persistent_state.hpp"
#include "control/state_timing.hpp"                           // logic::control::state_entered_ms
#include "control/refused_valve.hpp"                          // logic::control::last_refused_valve (+ count)
#include "control/control_flags.hpp"                          // logic::control::base_control_flags / fcu_control_flags
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

/* Build a UDP datagram: a 12-byte EthernetHeader (a command addressed to `target`)
   followed by `body`. */
std::vector<uint8_t> makeCommand(command::CommandType type, BoardId target,
                                 const uint8_t* body, std::size_t bodyLen, uint8_t seq = 0)
{
    EthernetHeader header{};
    header.sender_id          = static_cast<uint8_t>(BoardId::GsControl);
    header.target_id          = static_cast<uint8_t>(target);
    header.payload_type       = static_cast<uint8_t>(PayloadType::Command);
    header.payload_id         = static_cast<uint8_t>(type);
    header.payload_size_bytes = static_cast<uint16_t>(bodyLen);
    header.seq                = seq;   // the GS-stamped sequence

    std::vector<uint8_t> buf(sizeof(EthernetHeader) + bodyLen);
    std::memcpy(buf.data(), &header, sizeof(EthernetHeader));
    if (bodyLen != 0) {
        std::memcpy(buf.data() + sizeof(EthernetHeader), body, bodyLen);
    }
    return buf;
}

std::vector<uint8_t> makeSetState(State requested, BoardId target = BoardId::FillingStation,
                                  uint8_t seq = 0)
{
    const SetStateFrame body{0, static_cast<uint8_t>(requested)};
    return makeCommand(command::CommandType::SetState, target,
                       reinterpret_cast<const uint8_t*>(&body), sizeof(body), seq);
}

std::vector<uint8_t> makeSetValve(FcuValves valve, ValveCommand action, uint8_t value = 0,
                                  BoardId target = BoardId::FillingStation, uint8_t seq = 0)
{
    const SetValvePositionFrame body{valve, action, value};
    return makeCommand(command::CommandType::SetValvePosition, target,
                       reinterpret_cast<const uint8_t*>(&body), sizeof(body), seq);
}

std::vector<uint8_t> makePing(uint8_t seq = 0)
{
    return makeCommand(command::CommandType::Ping, BoardId::FillingStation, nullptr, 0, seq);
}

std::vector<uint8_t> makeSetControlFlag(ControlFlagBase flag, uint8_t value, BoardId target,
                                        uint8_t seq = 0)
{
    const SetControlFlagFrame body{static_cast<uint16_t>(flag), value, /*reserved=*/0};
    return makeCommand(command::CommandType::SetControlFlag, target,
                       reinterpret_cast<const uint8_t*>(&body), sizeof(body), seq);
}

std::vector<uint8_t> makeSynchronise(BoardId target = BoardId::FillingStation, uint8_t seq = 0)
{
    return makeCommand(command::CommandType::Synchronise, target, nullptr, 0, seq);
}

class ControlTest : public ::testing::Test {
protected:
    FakeEthernet eth_;
    FakeCan      can_;
    logic::fcu::Communication<FakeEthernet, FakeCan> comm_{eth_, can_};
    FakeValve    fill_valve_;
    FakeValve    dump_valve_;
    FakeEmatch   ematch_;
    FakeSolenoid solenoid_;
    logic::fcu::Control<FakeValve, logic::fcu::Communication<FakeEthernet, FakeCan>,
                        FakeEmatch, FakeSolenoid>
                 control_{fill_valve_, dump_valve_, comm_, ematch_, solenoid_};
    uint32_t     now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        logic::control::persistent_state = logic::control::PersistentState{};
        logic::control::base_control_flags = logic::control::ControlFlags<ControlFlagBase>{};  // base flags off
        logic::control::fcu_control_flags  = logic::control::ControlFlags<FcuControlFlag>{};   // per-board flags off
        logic::control::last_refused_transition = {State::Init, State::Init};
        logic::control::refused_transition_count = 0;
        logic::control::last_refused_control_flag =
            {logic::control::REFUSED_CONTROL_FLAG_NONE, 0, State::Init};
        logic::control::refused_control_flag_count = 0;
        logic::control::last_refused_valve =
            {logic::control::REFUSED_VALVE_NONE, 0, 0, State::Init};
        logic::control::refused_valve_count = 0;
        logic::control::state_entered_ms = 0;   // reset the dwell clock between tests
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

    // Assert exactly one datagram went out and it is an Ack: a Response (which the Communication
    // layer routes to the COMMANDER endpoint, not the telemetry sink) echoing `seq`. Also checks
    // the datagram's destination is the command endpoint, so the Ack rides the commander line.
    void expectAckToGs(uint8_t seq)
    {
        ASSERT_EQ(bus().udp_tx.size(), 1u);
        const auto& sent = bus().udp_tx.front();
        EthernetHeader ack;
        std::memcpy(&ack, sent.payload.data(), sizeof(EthernetHeader));
        EXPECT_EQ(static_cast<BoardId>(ack.sender_id), BoardId::FillingStation);
        EXPECT_EQ(static_cast<PayloadType>(ack.payload_type), PayloadType::Response);
        EXPECT_EQ(static_cast<ResponseType>(ack.payload_id), ResponseType::Ack);
        EXPECT_EQ(ack.seq, seq);              // echoes the GS seq so it matches the command
        EXPECT_EQ(sent.dest.ipv4, logic::fcu::detail::COMMANDER_IPV4);   // commander line, not telemetry
        EXPECT_EQ(sent.dest.port, logic::fcu::detail::COMMANDER_PORT);
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

TEST_F(ControlTest, PropagatesGsSeqOntoTheCanPingAndBackOnTheRelayedPong)
{
    setCurrent(State::Safe);
    deliver(makePing(/*seq=*/9));   // the GS stamped seq 9

    // Forwarded to the ECU carrying the GS's seq (4-bit on CAN).
    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader fwd;
    fwd.code = bus().can_tx.front().id;
    EXPECT_EQ(fwd.frame.seq, 9u);

    // The ECU's Pong (echoing seq 9) is relayed to the GS still carrying seq 9.
    control_.onResponse(static_cast<uint8_t>(ResponseType::Pong), /*seq=*/9, ++now_ms_);
    ASSERT_EQ(bus().udp_tx.size(), 1u);
    EthernetHeader relayed;
    std::memcpy(&relayed, bus().udp_tx.front().payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(relayed.seq, 9u);
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
    control_.onResponse(static_cast<uint8_t>(ResponseType::Pong),
                        static_cast<uint8_t>(sent.frame.seq), ++now_ms_);  // ack the exact ping

    now_ms_ += logic::fcu::detail::COMMAND_TIMEOUT_MS * 4;
    control_.servicePending(now_ms_);
    EXPECT_EQ(bus().can_tx.size(), 1u);   // cleared by the Pong — never resent
}

TEST_F(ControlTest, PongWithWrongSeqDoesNotStopRetransmission)
{
    setCurrent(State::Safe);
    deliver(makePing());                                   // seq 0
    control_.onResponse(static_cast<uint8_t>(ResponseType::Pong),
                        /*seq=*/7, ++now_ms_);             // a stale/mismatched Pong

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

TEST_F(ControlTest, LocalSetStateAppliesAndAcksGs)
{
    setCurrent(State::Safe);
    // seq > 15 on purpose: the local Ack must echo the FULL 8-bit GS seq, not the 4-bit value
    // the CAN bridge would truncate it to.
    deliver(makeSetState(State::Unsafe, BoardId::FillingStation, /*seq=*/200));

    EXPECT_EQ(current(), State::Unsafe);
    EXPECT_TRUE(bus().can_tx.empty());   // FillingStation-targeted: applied here, no ECU hop
    expectAckToGs(/*seq=*/200);          // Acked on the commander line, full seq preserved
}

TEST_F(ControlTest, IllegalStateTransitionIsRejected)
{
    setCurrent(State::Safe);
    deliver(makeSetState(State::Ignite));   // Safe -> Ignite not allowed
    EXPECT_EQ(current(), State::Safe);
    EXPECT_TRUE(bus().udp_tx.empty());      // refused -> not Acked
}

TEST_F(ControlTest, RefusedTransitionIsRecorded)
{
    setCurrent(State::Safe);
    deliver(makeSetState(State::Ignite));   // Safe's only exit is Unsafe -> refused
    EXPECT_EQ(current(), State::Safe);
    EXPECT_EQ(logic::control::last_refused_transition.from, State::Safe);
    EXPECT_EQ(logic::control::last_refused_transition.to,   State::Ignite);
}

TEST_F(ControlTest, UnknownRequestedStateIsRejected)
{
    setCurrent(State::Safe);
    deliver(makeSetState(static_cast<State>(0xFF)));
    EXPECT_EQ(current(), State::Safe);
}

TEST_F(ControlTest, EngineSetStateBridgesOverCanWithoutLocalApply)
{
    setCurrent(State::Safe);
    deliver(makeSetState(State::Unsafe, BoardId::Engine, /*seq=*/6));

    EXPECT_EQ(current(), State::Safe);   // an Engine-only target is not applied to our own state
    EXPECT_TRUE(bus().udp_tx.empty());   // the ECU Acks; the FCU does not Ack locally

    ASSERT_EQ(bus().can_tx.size(), 1u);
    const auto& sent = bus().can_tx.front();
    CanHeader header;
    header.code = sent.id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::Engine);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Command);
    EXPECT_EQ(static_cast<command::CommandType>(header.frame.payload_id), command::CommandType::SetState);
    EXPECT_EQ(header.frame.seq, 6u);     // GS seq propagated onto the CAN hop
    ASSERT_GE(sent.length, sizeof(SetStateFrame));
    EXPECT_EQ(sent.data[1], static_cast<uint8_t>(State::Unsafe));  // requested id carried verbatim
}

TEST_F(ControlTest, BroadcastSetStateAppliesLocallyAndBridges)
{
    setCurrent(State::Safe);
    deliver(makeSetState(State::Unsafe, BoardId::Broadcast, /*seq=*/2));

    EXPECT_EQ(current(), State::Unsafe);   // applied locally
    EXPECT_EQ(bus().can_tx.size(), 1u);    // and bridged to the ECU
    expectAckToGs(/*seq=*/2);              // local Ack on the commander line (the ECU Acks too)
}

TEST_F(ControlTest, TransitionToSafeClosesLocalValves)
{
    setCurrent(State::Unsafe);
    clearValveCalls();
    deliver(makeSetState(State::Safe));   // any transition into Safe closes the local valves
    EXPECT_EQ(current(), State::Safe);
    EXPECT_EQ(fill_valve_.close_calls, 1);
    EXPECT_EQ(dump_valve_.close_calls, 1);
}

TEST_F(ControlTest, FcuIgniteToLaunchDoesNotActuateValves)
{
    setCurrent(State::Ignite);
    clearValveCalls();
    deliver(makeSetState(State::Launch));   // FCU does nothing on Ignite -> Launch (the ECU acts)
    EXPECT_EQ(current(), State::Launch);
    EXPECT_EQ(fill_valve_.open_calls, 0);
    EXPECT_EQ(dump_valve_.open_calls, 0);
    EXPECT_EQ(fill_valve_.close_calls, 0);
    EXPECT_EQ(dump_valve_.close_calls, 0);
}

/* Any transition into Abort closes Fill and opens Dump (vent the line) — a per-transition
   side effect that runs even though an operator SetValvePosition would be refused outside Unsafe. */
TEST_F(ControlTest, TransitionToAbortClosesFillAndOpensDump)
{
    setCurrent(State::Unsafe);
    clearValveCalls();
    deliver(makeSetState(State::Abort));   // Unsafe -> Abort
    EXPECT_EQ(current(), State::Abort);
    EXPECT_EQ(fill_valve_.close_calls, 1);   // fill closed
    EXPECT_EQ(dump_valve_.open_calls, 1);    // dump opened (vent)
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

/* ---- SetValvePosition (gated to Unsafe; local actuation, bridge to ECU, or both) ---------- */

TEST_F(ControlTest, ValidValveActionActuatesLocalValve)
{
    setCurrent(State::Unsafe);   // operator per-valve actuation is permitted only in Unsafe
    deliver(makeSetValve(FcuValves::Fill, ValveCommand::Open, /*value=*/0,
                         BoardId::FillingStation, /*seq=*/200));   // seq > 15: full 8-bit echo
    EXPECT_EQ(fill_valve_.open_calls, 1);
    EXPECT_EQ(dump_valve_.open_calls, 0);
    EXPECT_TRUE(bus().can_tx.empty());   // FillingStation-targeted: actuated here, no ECU hop
    expectAckToGs(/*seq=*/200);          // Acked on the commander line, full seq preserved
}

TEST_F(ControlTest, UnknownValveActionIsRejected)
{
    setCurrent(State::Unsafe);   // past the state gate, so this exercises the action-validation path
    deliver(makeSetValve(FcuValves::Fill, static_cast<ValveCommand>(0xFF)));
    EXPECT_EQ(fill_valve_.open_calls, 0);
    EXPECT_EQ(fill_valve_.close_calls, 0);
    EXPECT_EQ(fill_valve_.percent_calls, 0);
    EXPECT_TRUE(bus().udp_tx.empty());   // refused -> not Acked
}

/* The operator valve command is gated to Unsafe: outside Unsafe the FCU does not actuate its
   local valve, does not bridge the command to the ECU, and does not Ack — the gate is in
   handleCommand, before any target routing. (Transition-driven valve moves are unaffected —
   they run in onTransition, tested separately.) */
TEST_F(ControlTest, ValveCommandIsRejectedOutsideUnsafe)
{
    uint16_t expected_refusals = 0;
    for (const State s : {State::Safe, State::Ignite, State::Launch, State::Abort, State::Error}) {
        setCurrent(s);
        clearValveCalls();
        bus().reset();
        // Engine target: would bridge to the ECU in Unsafe — here it must not.
        deliver(makeSetValve(FcuValves::Fill, ValveCommand::Open, /*value=*/0,
                             BoardId::Engine, /*seq=*/4));
        EXPECT_TRUE(bus().can_tx.empty())    << "bridged in state " << static_cast<int>(s);
        EXPECT_TRUE(bus().udp_tx.empty())    << "Acked in state "   << static_cast<int>(s);
        // FillingStation target: would actuate our own valve in Unsafe — here it must not.
        deliver(makeSetValve(FcuValves::Fill, ValveCommand::Open, /*value=*/0,
                             BoardId::FillingStation, /*seq=*/4));
        EXPECT_EQ(fill_valve_.open_calls, 0) << "actuated in state " << static_cast<int>(s);
        EXPECT_TRUE(bus().udp_tx.empty())    << "Acked in state "    << static_cast<int>(s);

        expected_refusals += 2;   // both targets refused-and-recorded in this state
        EXPECT_EQ(logic::control::refused_valve_count, expected_refusals);
        EXPECT_EQ(logic::control::last_refused_valve.valve, static_cast<uint8_t>(FcuValves::Fill));
        EXPECT_EQ(logic::control::last_refused_valve.action, static_cast<uint8_t>(ValveCommand::Open));
        EXPECT_EQ(logic::control::last_refused_valve.state, s);   // the state it was refused in
    }
}

TEST_F(ControlTest, EngineValveCommandBridgesOverCanWithoutLocalActuation)
{
    setCurrent(State::Unsafe);
    deliver(makeSetValve(FcuValves::Fill, ValveCommand::Open, /*value=*/0,
                         BoardId::Engine, /*seq=*/6));

    EXPECT_EQ(fill_valve_.open_calls, 0);   // not ours to drive — relayed to the ECU
    EXPECT_TRUE(bus().udp_tx.empty());      // the ECU Acks; the FCU does not Ack locally

    ASSERT_EQ(bus().can_tx.size(), 1u);
    const auto& sent = bus().can_tx.front();
    CanHeader header;
    header.code = sent.id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::Engine);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Command);
    EXPECT_EQ(static_cast<command::CommandType>(header.frame.payload_id),
              command::CommandType::SetValvePosition);
    EXPECT_EQ(header.frame.seq, 6u);        // GS seq propagated onto the CAN hop
    ASSERT_GE(sent.length, sizeof(SetValvePositionFrame));
    EXPECT_EQ(sent.data[0], static_cast<uint8_t>(FcuValves::Fill));  // valve + action carried verbatim
    EXPECT_EQ(sent.data[1], static_cast<uint8_t>(ValveCommand::Open));
}

/* Per-valve actuation is single-board only: a Broadcast-targeted valve command is REFUSED even
   in Unsafe — it neither actuates the local valve nor bridges to the ECU, and is not Acked.
   (SetState / SetControlFlag still fan out to Broadcast; only SetValvePosition is single-board.) */
TEST_F(ControlTest, BroadcastValveCommandIsRejected)
{
    setCurrent(State::Unsafe);
    deliver(makeSetValve(FcuValves::Dump, ValveCommand::Close, /*value=*/0,
                         BoardId::Broadcast, /*seq=*/2));

    EXPECT_EQ(fill_valve_.open_calls, 0);   // not actuated locally
    EXPECT_EQ(dump_valve_.close_calls, 0);
    EXPECT_TRUE(bus().can_tx.empty());      // not bridged to the ECU
    EXPECT_TRUE(bus().udp_tx.empty());      // refused -> not Acked

    EXPECT_EQ(logic::control::refused_valve_count, 1u);   // refused-and-recorded, not silently dropped
    EXPECT_EQ(logic::control::last_refused_valve.valve, static_cast<uint8_t>(FcuValves::Dump));
    EXPECT_EQ(logic::control::last_refused_valve.action, static_cast<uint8_t>(ValveCommand::Close));
    EXPECT_EQ(logic::control::last_refused_valve.state, State::Unsafe);
}

TEST_F(ControlTest, AckFromEcuClearsThePendingBridgedValveCommand)
{
    setCurrent(State::Unsafe);
    deliver(makeSetValve(FcuValves::Fill, ValveCommand::Open, /*value=*/0,
                         BoardId::Engine, /*seq=*/8));
    ASSERT_EQ(bus().can_tx.size(), 1u);   // bridged once

    control_.onResponse(static_cast<uint8_t>(ResponseType::Ack), /*seq=*/8, ++now_ms_);

    now_ms_ += logic::fcu::detail::COMMAND_TIMEOUT_MS * 4;
    control_.servicePending(now_ms_);
    EXPECT_EQ(bus().can_tx.size(), 1u);   // cleared by the Ack — never resent
}

/* ---- Pong (Ecu->Fcu->Gs) ------------------------------------------------- */

TEST_F(ControlTest, PongIsRelayedToGsOverEthernet)
{
    control_.onResponse(static_cast<uint8_t>(ResponseType::Pong), /*seq=*/0, ++now_ms_);

    ASSERT_EQ(bus().udp_tx.size(), 1u);
    EthernetHeader header;
    std::memcpy(&header, bus().udp_tx.front().payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(static_cast<BoardId>(header.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.target_id), BoardId::GsControl);
    EXPECT_EQ(static_cast<PayloadType>(header.payload_type), PayloadType::Response);
    EXPECT_EQ(static_cast<ResponseType>(header.payload_id), ResponseType::Pong);
}

/* ---- SetControlFlag (local apply, bridge to ECU, or both) ---------------- */

TEST_F(ControlTest, LocalSetControlFlagAppliesAndAcksGs)
{
    setCurrent(State::Safe);
    deliver(makeSetControlFlag(ControlFlagBase::PersistingData, /*value=*/1,
                               BoardId::FillingStation, /*seq=*/3));

    EXPECT_TRUE(logic::control::base_control_flags.get(ControlFlagBase::PersistingData));
    EXPECT_TRUE(bus().can_tx.empty());   // FillingStation-targeted: applied here, no ECU hop
    expectAckToGs(/*seq=*/3);            // Acked on the commander line
}

TEST_F(ControlTest, EngineSetControlFlagBridgesOverCanWithoutLocalApply)
{
    setCurrent(State::Safe);
    deliver(makeSetControlFlag(ControlFlagBase::PersistingData, /*value=*/1,
                               BoardId::Engine, /*seq=*/6));

    EXPECT_FALSE(logic::control::base_control_flags.get(ControlFlagBase::PersistingData));  // not ours to set
    EXPECT_TRUE(bus().udp_tx.empty());   // the ECU Acks; the FCU does not Ack locally

    ASSERT_EQ(bus().can_tx.size(), 1u);
    const auto& sent = bus().can_tx.front();
    CanHeader header;
    header.code = sent.id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::Engine);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Command);
    EXPECT_EQ(static_cast<command::CommandType>(header.frame.payload_id),
              command::CommandType::SetControlFlag);
    EXPECT_EQ(header.frame.seq, 6u);     // GS seq propagated onto the CAN hop
    ASSERT_GE(sent.length, sizeof(SetControlFlagFrame));
    SetControlFlagFrame bridged{};
    std::memcpy(&bridged, sent.data.data(), sizeof(bridged));   // the frame rides verbatim over CAN
    EXPECT_EQ(bridged.flag, static_cast<uint16_t>(ControlFlagBase::PersistingData));
    EXPECT_EQ(bridged.value, 1u);
}

TEST_F(ControlTest, BroadcastSetControlFlagAppliesLocallyAndBridges)
{
    setCurrent(State::Safe);
    deliver(makeSetControlFlag(ControlFlagBase::PersistingData, /*value=*/1,
                               BoardId::Broadcast, /*seq=*/2));

    EXPECT_TRUE(logic::control::base_control_flags.get(ControlFlagBase::PersistingData));  // applied locally
    EXPECT_EQ(bus().can_tx.size(), 1u);   // and bridged to the ECU
    EXPECT_EQ(bus().udp_tx.size(), 1u);   // local Ack to the GS
}

TEST_F(ControlTest, AckFromEcuClearsThePendingBridgedCommand)
{
    setCurrent(State::Safe);
    deliver(makeSetControlFlag(ControlFlagBase::PersistingData, /*value=*/1,
                               BoardId::Engine, /*seq=*/8));
    ASSERT_EQ(bus().can_tx.size(), 1u);   // bridged once

    control_.onResponse(static_cast<uint8_t>(ResponseType::Ack), /*seq=*/8, ++now_ms_);

    now_ms_ += logic::fcu::detail::COMMAND_TIMEOUT_MS * 4;
    control_.servicePending(now_ms_);
    EXPECT_EQ(bus().can_tx.size(), 1u);   // cleared by the Ack — never resent
}

/* ---- Synchronise --------------------------------------------------------- */

TEST_F(ControlTest, SynchroniseAcksGs)
{
    deliver(makeSynchronise(BoardId::FillingStation, /*seq=*/171));   // seq > 15: full 8-bit echo
    EXPECT_TRUE(bus().can_tx.empty());   // handled locally; not bridged to the ECU
    expectAckToGs(/*seq=*/171);          // receipt Acked on the commander line, full seq preserved
}

} // namespace
