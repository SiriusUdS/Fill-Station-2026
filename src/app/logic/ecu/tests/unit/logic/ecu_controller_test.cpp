/* ------------------------------------------------------------------------- *
 * Unit tests for the ECU engine controller (logic::ecu) and its split into
 * Communication / Telemetry / Control.
 *
 * The logic is exercised through its public surface (init / tick / produceRecord)
 * over the FakeBus: tests push inbound CAN frames (commands from the FCU), tick,
 * and inspect what the logic actuated (valve call counts) or sent back on the bus
 * (Pong replies, fragmented telemetry). No Ethernet — the ECU is CAN-only.
 * ------------------------------------------------------------------------- */

#include "ecu_controller.hpp"
#include "support/fakes.hpp"   // the standard set of host test doubles

#include "control/persistent_state.hpp"
#include "system/state.hpp"
#include "framing/can_header.hpp"
#include "framing/payload_type.hpp"
#include "command/command_type.hpp"
#include "response/response_type.hpp"
#include "command/set_valve_position.hpp"   // ValveCommand
#include "command/set_control_flag.hpp"     // ControlFlag, SetControlFlagFrame
#include "command/set_state.hpp"            // SetStateFrame
#include "control/control_flags.hpp"        // logic::control::control_flags
#include "system/valves/ecu.hpp"            // EcuValves
#include "system/board_id.hpp"
#include "telemetry/telemetry_type.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

using logic::communication::CanFrame;
namespace command = logic::communication::command;
using logic::control::State;

namespace {

/* Build a CAN command frame addressed FCU -> Engine. */
CanFrame makeCommand(command::CommandType type, uint8_t senderState,
                     BoardId target = BoardId::Engine, uint8_t seq = 0)
{
    CanHeader header        = {};
    header.frame.sender_id    = static_cast<uint8_t>(BoardId::FillingStation);
    header.frame.target_id    = static_cast<uint8_t>(target);
    header.frame.sender_state = senderState;
    header.frame.payload_type = static_cast<uint8_t>(PayloadType::Command);
    header.frame.payload_id   = static_cast<uint8_t>(type);
    header.frame.seq          = seq;

    CanFrame frame;
    frame.id     = header.code;
    frame.data   = {};
    frame.length = 8;
    return frame;
}

/* A SetValvePosition command FCU -> Engine: the 3-byte SetValvePositionFrame rides
   verbatim in the payload (data[0] = valve index, data[1] = action, data[2] = value). */
CanFrame makeValveCmd(EcuValves valve, ValveCommand action, BoardId target = BoardId::Engine,
                      uint8_t seq = 0)
{
    CanFrame frame = makeCommand(command::CommandType::SetValvePosition, /*senderState=*/0,
                                 target, seq);
    frame.data[0] = static_cast<uint8_t>(valve);
    frame.data[1] = static_cast<uint8_t>(action);
    frame.data[2] = 0;   // value, unused for Open/Close
    frame.length  = sizeof(SetValvePositionFrame);
    return frame;
}

CanFrame makePing(uint8_t seq = 0)
{
    CanFrame frame = makeCommand(command::CommandType::Ping, /*senderState=*/0,
                                 BoardId::Engine, seq);
    frame.length = 0;   // a ping carries no payload
    return frame;
}

/* A SetState command FCU -> Engine: the 2-byte SetStateFrame rides in the payload
   (data[0] = flags, data[1] = requested state id). */
CanFrame makeSetState(State requested, BoardId target = BoardId::Engine, uint8_t seq = 0)
{
    CanFrame frame = makeCommand(command::CommandType::SetState, /*senderState=*/0, target, seq);
    frame.data[0] = 0;   // flags (unused here)
    frame.data[1] = static_cast<uint8_t>(requested);
    frame.length  = sizeof(SetStateFrame);
    return frame;
}

/* A SetControlFlag command FCU -> Engine: the 2-byte SetControlFlagFrame rides in
   the payload (data[0] = flag id, data[1] = value). */
CanFrame makeSetControlFlag(ControlFlag flag, uint8_t value,
                            BoardId target = BoardId::Engine, uint8_t seq = 0)
{
    CanFrame frame = makeCommand(command::CommandType::SetControlFlag, /*senderState=*/0,
                                 target, seq);
    frame.data[0] = static_cast<uint8_t>(flag);
    frame.data[1] = value;
    frame.length  = sizeof(SetControlFlagFrame);
    return frame;
}

class EcuControllerTest : public ::testing::Test {
protected:
    FakeStorage      storage_fast_;
    FakeStorage      storage_slow_;
    FakeStorage      storage_ext_;
    FakeValve        ipa_valve_;
    FakeValve        nos_valve_;
    FakeStreamingAdc adc_;
    FakeCan          can_;
    logic::ecu::Controller<FakeStorage, FakeValve, FakeStreamingAdc, FakeCan>
                     controller_{storage_fast_, storage_slow_, storage_ext_,
                                 ipa_valve_, nos_valve_, adc_, can_};
    uint32_t         now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        logic::control::persistent_state = logic::control::PersistentState{};
        logic::control::control_flags    = logic::control::ControlFlags{};  // all flags off
        logic::control::last_refused_transition = {State::Init, State::Init};
        controller_.init();
        // init() advanced Init -> Safe, which closes both valves; discard those calls so each
        // test counts only its own actuation.
        ipa_valve_.open_calls = ipa_valve_.close_calls = ipa_valve_.percent_calls = 0;
        nos_valve_.open_calls = nos_valve_.close_calls = nos_valve_.percent_calls = 0;
    }

    void step() { controller_.tick(++now_ms_); }
    State current() const { return logic::control::persistent_state.fill_state; }

    void deliver(const CanFrame& frame)
    {
        bus().push_can(frame);
        step();
    }
};

/* ---- Startup ------------------------------------------------------------- */

TEST_F(EcuControllerTest, InitEntersSafe)
{
    // Init -> Safe as soon as init() completes (cold boot), before any tick.
    EXPECT_EQ(current(), State::Safe);
}

/* ---- Valve commands (Control actuates the propellant valves) ------------- */

TEST_F(EcuControllerTest, IpaValveOpens)
{
    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::Open));
    EXPECT_EQ(ipa_valve_.open_calls, 1);
    EXPECT_EQ(nos_valve_.open_calls, 0);
}

TEST_F(EcuControllerTest, NosValveCloses)
{
    deliver(makeValveCmd(EcuValves::NOS, ValveCommand::Close));
    EXPECT_EQ(nos_valve_.close_calls, 1);
    EXPECT_EQ(ipa_valve_.close_calls, 0);
}

TEST_F(EcuControllerTest, CommandForAnotherBoardIsIgnored)
{
    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::Open, BoardId::FillingStation));
    EXPECT_EQ(ipa_valve_.open_calls, 0);
}

TEST_F(EcuControllerTest, ValveCommandIsAckedToTheFcu)
{
    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::Open, BoardId::Engine, /*seq=*/5));

    EXPECT_EQ(ipa_valve_.open_calls, 1);

    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader header;
    header.code = bus().can_tx.front().id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Response);
    EXPECT_EQ(static_cast<ResponseType>(header.frame.payload_id), ResponseType::Ack);
    EXPECT_EQ(header.frame.seq, 5u);   // echoes the command seq so the FCU matches it
}

TEST_F(EcuControllerTest, UnknownValveIsNotActuatedOrAcked)
{
    deliver(makeValveCmd(static_cast<EcuValves>(0x7F), ValveCommand::Open));
    EXPECT_EQ(ipa_valve_.open_calls, 0);
    EXPECT_EQ(nos_valve_.open_calls, 0);
    EXPECT_TRUE(bus().can_tx.empty());   // no Ack for a valve we did not drive
}

/* ---- Ping (Control answers with Pong over CAN) --------------------------- */

TEST_F(EcuControllerTest, PingIsAnsweredWithPongToTheFcu)
{
    deliver(makePing());

    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader header;
    header.code = bus().can_tx.front().id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Response);
    EXPECT_EQ(static_cast<ResponseType>(header.frame.payload_id), ResponseType::Pong);
}

TEST_F(EcuControllerTest, PongEchoesThePingSeq)
{
    deliver(makePing(/*seq=*/7));

    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader header;
    header.code = bus().can_tx.front().id;
    EXPECT_EQ(static_cast<ResponseType>(header.frame.payload_id), ResponseType::Pong);
    EXPECT_EQ(header.frame.seq, 7u);   // so the FCU can match the reply to its ping
}

/* ---- SetControlFlag (Control sets a runtime flag + Acks the FCU) ---------- */

TEST_F(EcuControllerTest, SetControlFlagAppliesTheFlagAndAcksTheFcu)
{
    deliver(makeSetControlFlag(ControlFlag::PersistingData, /*value=*/1, BoardId::Engine, /*seq=*/4));

    EXPECT_TRUE(logic::control::control_flags.get(ControlFlag::PersistingData));

    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader header;
    header.code = bus().can_tx.front().id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Response);
    EXPECT_EQ(static_cast<ResponseType>(header.frame.payload_id), ResponseType::Ack);
    EXPECT_EQ(header.frame.seq, 4u);   // echoes the command seq so the FCU matches it
}

TEST_F(EcuControllerTest, SetControlFlagWithZeroValueClearsTheFlag)
{
    logic::control::control_flags.set(ControlFlag::PersistingData, true);
    deliver(makeSetControlFlag(ControlFlag::PersistingData, /*value=*/0));
    EXPECT_FALSE(logic::control::control_flags.get(ControlFlag::PersistingData));
}

TEST_F(EcuControllerTest, UnknownControlFlagIsNotAppliedOrAcked)
{
    deliver(makeSetControlFlag(static_cast<ControlFlag>(0xFF), /*value=*/1));
    EXPECT_FALSE(logic::control::control_flags.get(ControlFlag::PersistingData));
    EXPECT_TRUE(bus().can_tx.empty());   // no Ack for a flag we did not apply
}

/* ---- SetState (arm/disarm the ECU's global state machine) ---------------- */

TEST_F(EcuControllerTest, SetStateArmsFromSafeToUnsafeAndAcks)
{
    ASSERT_EQ(current(), State::Safe);   // init() already advanced Init -> Safe
    deliver(makeSetState(State::Unsafe, BoardId::Engine, /*seq=*/9));

    EXPECT_EQ(current(), State::Unsafe);

    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader header;
    header.code = bus().can_tx.front().id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Response);
    EXPECT_EQ(static_cast<ResponseType>(header.frame.payload_id), ResponseType::Ack);
    EXPECT_EQ(header.frame.seq, 9u);   // echoes the command seq so the FCU matches it
}

TEST_F(EcuControllerTest, SetStateDisarmsFromUnsafeBackToSafe)
{
    deliver(makeSetState(State::Unsafe));   // Safe -> Unsafe
    ASSERT_EQ(current(), State::Unsafe);
    deliver(makeSetState(State::Safe));     // Unsafe -> Safe
    EXPECT_EQ(current(), State::Safe);
}

TEST_F(EcuControllerTest, SetStateRejectsIllegalTransitionWithoutAck)
{
    ASSERT_EQ(current(), State::Safe);
    deliver(makeSetState(State::Ignite, BoardId::Engine));  // Safe -> Ignite is not permitted here
    EXPECT_EQ(current(), State::Safe);
    EXPECT_TRUE(bus().can_tx.empty());   // no Ack for a transition we did not apply

    // The refused transition is recorded for the ExtendedSystemState telemetry.
    EXPECT_EQ(logic::control::last_refused_transition.from, State::Safe);
    EXPECT_EQ(logic::control::last_refused_transition.to,   State::Ignite);
}

TEST_F(EcuControllerTest, IgniteToLaunchOpensBothValves)
{
    deliver(makeSetState(State::Unsafe));   // Safe -> Unsafe
    deliver(makeSetState(State::Ignite));   // Unsafe -> Ignite
    const auto ipa_before = ipa_valve_.open_calls;
    const auto nos_before = nos_valve_.open_calls;

    deliver(makeSetState(State::Launch));    // Ignite -> Launch: the ECU drives both valves open
    EXPECT_EQ(current(), State::Launch);
    EXPECT_GT(ipa_valve_.open_calls, ipa_before);
    EXPECT_GT(nos_valve_.open_calls, nos_before);
}

TEST_F(EcuControllerTest, TransitionToSafeClosesBothValves)
{
    deliver(makeSetState(State::Unsafe));   // Safe -> Unsafe
    const auto ipa_before = ipa_valve_.close_calls;
    const auto nos_before = nos_valve_.close_calls;

    deliver(makeSetState(State::Safe));     // Unsafe -> Safe: the ECU drives both valves closed
    EXPECT_EQ(current(), State::Safe);
    EXPECT_GT(ipa_valve_.close_calls, ipa_before);
    EXPECT_GT(nos_valve_.close_calls, nos_before);
}

/* ---- The shared 3-file recording policy (PersistingData + FastRecording) ---- */

TEST_F(EcuControllerTest, TelemetryDrainsWithoutWritingToSdWhenFlagOff)
{
    step();  // Init -> Safe; PersistingData defaults off
    const AdcInfo sample{};
    for (int i = 0; i < 2000 && bus().can_tx.empty(); ++i) {
        adc_.push(sample);
        controller_.produceRecord(++now_ms_);
        step();
    }
    ASSERT_FALSE(bus().can_tx.empty()) << "a full telemetry half never drained";
    EXPECT_TRUE(storage_fast_.writes.empty());   // nothing reached the card in any stream
    EXPECT_TRUE(storage_slow_.writes.empty());
    EXPECT_TRUE(storage_ext_.writes.empty());
}

TEST_F(EcuControllerTest, FastRecordingPersistsRawSystemStateToDataFast)
{
    step();  // Init -> Safe
    logic::control::control_flags.set(ControlFlag::PersistingData, true);
    logic::control::control_flags.set(ControlFlag::FastRecording, true);   // raw 2 kHz -> data_fast.bin
    const AdcInfo sample{};
    for (int i = 0; i < 2000 && storage_fast_.writes.empty(); ++i) {
        adc_.push(sample);
        controller_.produceRecord(++now_ms_);
        step();
    }
    EXPECT_FALSE(storage_fast_.writes.empty());  // the raw half reached data_fast.bin
    EXPECT_TRUE(storage_slow_.writes.empty());   // slow file untouched in fast mode
}

TEST_F(EcuControllerTest, SlowRecordingPersistsAveragedSystemStateToDataSlow)
{
    step();  // Init -> Safe; FastRecording stays off -> slow (125 Hz averaged) -> data_slow.bin
    logic::control::control_flags.set(ControlFlag::PersistingData, true);
    const AdcInfo sample{};
    for (int i = 0; i < 4000 && storage_slow_.writes.empty(); ++i) {
        adc_.push(sample);
        controller_.produceRecord(++now_ms_);
        step();
    }
    EXPECT_FALSE(storage_slow_.writes.empty());  // averaged records reached data_slow.bin
    EXPECT_TRUE(storage_fast_.writes.empty());    // fast file untouched in slow mode
}

/* ---- Telemetry (produce + drain + fragment onto CAN) --------------------- */

TEST_F(EcuControllerTest, TelemetryIsDownlinkedToTheFcuOverCan)
{
    step();  // Init -> Safe
    bus().can_tx.clear();

    const AdcInfo sample{};
    for (int i = 0; i < 2000 && bus().can_tx.empty(); ++i) {
        adc_.push(sample);                     // one conversion into the ADC ring
        controller_.produceRecord(++now_ms_);  // drain it into the telemetry buffer
        step();                                // drainTick flushes any full half over CAN
    }

    ASSERT_FALSE(bus().can_tx.empty()) << "a full telemetry half never downlinked over CAN";
    CanHeader header;
    header.code = bus().can_tx.front().id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Telemetry);
    EXPECT_EQ(static_cast<TelemetryType>(header.frame.payload_id), TelemetryType::SystemState);
    EXPECT_EQ(static_cast<State>(header.frame.sender_state), State::Safe);  // ECU stamps its state
}

} // namespace
