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
#include "control/state_timing.hpp"     // logic::control::state_entered_ms
#include "control/state_machine.hpp"    // logic::control::LAUNCH_TO_SAFE_LOCKOUT_MS
#include "system/state.hpp"
#include "framing/can_header.hpp"
#include "framing/payload_type.hpp"
#include "command/command_type.hpp"
#include "response/response_type.hpp"
#include "command/set_valve_position.hpp"   // ValveCommand
#include "command/set_control_flag.hpp"     // ControlFlag, SetControlFlagFrame
#include "command/set_state.hpp"            // SetStateFrame
#include "control/control_flags.hpp"        // logic::control::base_control_flags
#include "control/refused_transition.hpp"   // last_refused_transition (+ count)
#include "control/refused_control_flag.hpp" // last_refused_control_flag (+ count)
#include "control/refused_valve.hpp"        // last_refused_valve (+ count)
#include "system/valves/ecu.hpp"            // EcuValves
#include "system/board_id.hpp"
#include "telemetry/telemetry_type.hpp"
#include "telemetry/ecu_extended_system_state.hpp"   // EcuExtendedSystemState (decoded off the CAN bus)

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

/* A SetValvePosition command FCU -> Engine: the 4-byte SetValvePositionFrame rides verbatim in
   the payload (data[0] = valve index, data[1] = action, data[2] = value, data[3] = force). */
CanFrame makeValveCmd(EcuValves valve, ValveCommand action, BoardId target = BoardId::Engine,
                      uint8_t seq = 0, uint8_t value = 0, uint8_t force = 0)
{
    CanFrame frame = makeCommand(command::CommandType::SetValvePosition, /*senderState=*/0,
                                 target, seq);
    frame.data[0] = static_cast<uint8_t>(valve);
    frame.data[1] = static_cast<uint8_t>(action);
    frame.data[2] = value;   // opened-%, used by SetOpenedPct (ignored for Open/Close)
    frame.data[3] = force;   // 0 = normal, non-zero = forced (limit switches bypassed)
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

/* A SetControlFlag command FCU -> Engine: the SetControlFlagFrame {16-bit flag id, value}
   rides verbatim in the payload. */
CanFrame makeSetControlFlag(uint16_t flag, uint8_t value,
                            BoardId target = BoardId::Engine, uint8_t seq = 0)
{
    CanFrame frame = makeCommand(command::CommandType::SetControlFlag, /*senderState=*/0,
                                 target, seq);
    const SetControlFlagFrame body{flag, value, /*reserved=*/0};
    std::memcpy(frame.data.data(), &body, sizeof(body));
    frame.length = sizeof(SetControlFlagFrame);
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
    FakePowerMonitor pm_;
    logic::ecu::Controller<FakeStorage, FakeValve, FakeStreamingAdc, FakeCan, FakePowerMonitor>
                     controller_{storage_fast_, storage_slow_, storage_ext_,
                                 ipa_valve_, nos_valve_, adc_, can_, pm_};
    uint32_t         now_ms_ = 0;

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

    /* Arm the ECU into Unsafe (a bridged SetState Safe -> Unsafe), where operator per-valve
       actuation is permitted, then clear the resulting Ack so each test counts only its own
       bus traffic + valve calls. */
    void armUnsafe()
    {
        deliver(makeSetState(State::Unsafe));
        ASSERT_EQ(current(), State::Unsafe);
        bus().can_tx.clear();
        ipa_valve_.open_calls = ipa_valve_.close_calls = ipa_valve_.percent_calls = 0;
        nos_valve_.open_calls = nos_valve_.close_calls = nos_valve_.percent_calls = 0;
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
    armUnsafe();   // per-valve actuation is permitted only in Unsafe
    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::Open));
    EXPECT_EQ(ipa_valve_.open_calls, 1);
    EXPECT_EQ(nos_valve_.open_calls, 0);
}

TEST_F(EcuControllerTest, NosValveCloses)
{
    armUnsafe();
    deliver(makeValveCmd(EcuValves::NOS, ValveCommand::Close));
    EXPECT_EQ(nos_valve_.close_calls, 1);
    EXPECT_EQ(ipa_valve_.close_calls, 0);
}

/* The ECU now honours SetOpenedPct on its propellant valves (it previously ignored it): the
   command drives the valve to the requested percentage. */
TEST_F(EcuControllerTest, ValveSetOpenedPctDrivesThePercentage)
{
    armUnsafe();
    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::SetOpenedPct, BoardId::Engine,
                         /*seq=*/0, /*value=*/42));
    EXPECT_EQ(ipa_valve_.percent_calls, 1);
    EXPECT_FLOAT_EQ(ipa_valve_.last_percent, 42.0F);
    EXPECT_EQ(nos_valve_.percent_calls, 0);   // the other valve is untouched
}

TEST_F(EcuControllerTest, CommandForAnotherBoardIsIgnored)
{
    armUnsafe();
    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::Open, BoardId::FillingStation));
    EXPECT_EQ(ipa_valve_.open_calls, 0);
}

/* Per-valve actuation is single-board only: a Broadcast-targeted valve command is REFUSED (and
   recorded) even in Unsafe — the ECU only hand-drives a valve for an Engine-addressed command.
   (The FCU never bridges a Broadcast valve command; this is the ECU-side defense in depth.) */
TEST_F(EcuControllerTest, BroadcastValveCommandIsRejected)
{
    armUnsafe();
    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::Open, BoardId::Broadcast));
    EXPECT_EQ(ipa_valve_.open_calls, 0);   // not actuated off a broadcast
    EXPECT_TRUE(bus().can_tx.empty());     // and not Acked
    EXPECT_EQ(logic::control::refused_valve_count, 1u);   // refused-and-recorded, not silently dropped
    EXPECT_EQ(logic::control::last_refused_valve.valve, static_cast<uint8_t>(EcuValves::IPA));
}

/* Defense in depth: a bridged valve command is REFUSED (and recorded) unless the ECU is in
   Unsafe — it neither actuates nor Acks. (Its state mirrors the FCU's, which already gates it.) */
TEST_F(EcuControllerTest, ValveCommandIsRejectedOutsideUnsafe)
{
    ASSERT_EQ(current(), State::Safe);   // init() left us in Safe, not Unsafe
    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::Open, BoardId::Engine, /*seq=*/5));
    EXPECT_EQ(ipa_valve_.open_calls, 0);   // not actuated outside Unsafe
    EXPECT_TRUE(bus().can_tx.empty());     // and not Acked
    EXPECT_EQ(logic::control::refused_valve_count, 1u);   // refused-and-recorded
    EXPECT_EQ(logic::control::last_refused_valve.state, State::Safe);
}

TEST_F(EcuControllerTest, ValveCommandIsAckedToTheFcu)
{
    armUnsafe();
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
    armUnsafe();   // past the state gate, so this exercises the unknown-valve rejection
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
    deliver(makeSetControlFlag(static_cast<uint16_t>(ControlFlagBase::FastRecording),
                               /*value=*/1, BoardId::Engine, /*seq=*/4));

    EXPECT_TRUE(logic::control::base_control_flags.get(ControlFlagBase::FastRecording));

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
    logic::control::base_control_flags.set(ControlFlagBase::FastRecording, true);
    deliver(makeSetControlFlag(static_cast<uint16_t>(ControlFlagBase::FastRecording), /*value=*/0));
    EXPECT_FALSE(logic::control::base_control_flags.get(ControlFlagBase::FastRecording));
}

TEST_F(EcuControllerTest, UnknownControlFlagIsNotAppliedOrAcked)
{
    deliver(makeSetControlFlag(/*unknown per-board flag id=*/0xFF, /*value=*/1));
    EXPECT_FALSE(logic::control::base_control_flags.get(ControlFlagBase::FastRecording));
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

TEST_F(EcuControllerTest, IgniteToLaunchForceOpensBothValves)
{
    deliver(makeSetState(State::Unsafe));   // Safe -> Unsafe
    deliver(makeSetState(State::Ignite));   // Unsafe -> Ignite
    const auto ipa_before = ipa_valve_.open_calls;
    const auto nos_before = nos_valve_.open_calls;

    deliver(makeSetState(State::Launch));    // Ignite -> Launch: force both propellant valves open
    EXPECT_EQ(current(), State::Launch);
    EXPECT_GT(ipa_valve_.open_calls, ipa_before);
    EXPECT_GT(nos_valve_.open_calls, nos_before);
    // Forced open with the limit switches bypassed for the standard dwell.
    EXPECT_EQ(ipa_valve_.last_open_bypass_ms, logic::control::FORCED_VALVE_ACTUATION_MS);
    EXPECT_EQ(nos_valve_.last_open_bypass_ms, logic::control::FORCED_VALVE_ACTUATION_MS);
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
    // Into Safe is a NORMAL close (not forced) — no switch bypass.
    EXPECT_EQ(ipa_valve_.last_close_bypass_ms, 0u);
    EXPECT_EQ(nos_valve_.last_close_bypass_ms, 0u);
}

TEST_F(EcuControllerTest, TransitionToAbortForceClosesBothValves)
{
    deliver(makeSetState(State::Unsafe));   // Safe -> Unsafe
    const auto ipa_before = ipa_valve_.close_calls;
    const auto nos_before = nos_valve_.close_calls;

    deliver(makeSetState(State::Abort));    // Unsafe -> Abort: force both propellant valves shut
    EXPECT_EQ(current(), State::Abort);
    EXPECT_GT(ipa_valve_.close_calls, ipa_before);
    EXPECT_GT(nos_valve_.close_calls, nos_before);
    // An abort force-closes (limit switches bypassed) so the propellant shuts for certain.
    EXPECT_EQ(ipa_valve_.last_close_bypass_ms, logic::control::FORCED_VALVE_ACTUATION_MS);
    EXPECT_EQ(nos_valve_.last_close_bypass_ms, logic::control::FORCED_VALVE_ACTUATION_MS);
}

/* The operator's SetValvePosition carries the force flag: a non-zero force byte makes the ECU
   drive the valve with its limit switches bypassed (forced); a zero force byte is a normal move. */
TEST_F(EcuControllerTest, SetValvePositionForceFlagForcesTheValve)
{
    deliver(makeSetState(State::Unsafe));   // operator valve commands are Unsafe-only

    deliver(makeValveCmd(EcuValves::IPA, ValveCommand::Open, BoardId::Engine, /*seq=*/1,
                         /*value=*/0, /*force=*/1));
    EXPECT_EQ(ipa_valve_.last_open_bypass_ms, logic::control::FORCED_VALVE_ACTUATION_MS);

    deliver(makeValveCmd(EcuValves::NOS, ValveCommand::Close, BoardId::Engine, /*seq=*/2,
                         /*value=*/0, /*force=*/0));
    EXPECT_EQ(nos_valve_.last_close_bypass_ms, 0u);   // force byte clear -> a normal move
}

/* Both boards enforce the Launch -> Safe dwell lockout identically (broadcast convention
   means the ECU receives the same bridged SetState). Launch -> Safe is refused within the
   window and accepted after; Launch -> Abort is always available. */
TEST_F(EcuControllerTest, LaunchToSafeIsLockedOutOnTheEcuToo)
{
    deliver(makeSetState(State::Unsafe));   // Safe -> Unsafe
    deliver(makeSetState(State::Ignite));   // Unsafe -> Ignite
    deliver(makeSetState(State::Launch));   // Ignite -> Launch
    ASSERT_EQ(current(), State::Launch);

    deliver(makeSetState(State::Safe));     // within the lockout window -> refused
    EXPECT_EQ(current(), State::Launch);

    now_ms_ += logic::control::LAUNCH_TO_SAFE_LOCKOUT_MS;   // wait out the dwell
    deliver(makeSetState(State::Safe));     // now permitted
    EXPECT_EQ(current(), State::Safe);
}

/* ---- The shared 3-file recording policy (FastRecording picks fast vs slow) ---- */

TEST_F(EcuControllerTest, FastRecordingPersistsRawSystemStateToDataFast)
{
    step();  // Init -> Safe
    logic::control::base_control_flags.set(ControlFlagBase::FastRecording, true);   // raw 2 kHz -> data_fast.bin
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
    const AdcInfo sample{};
    // One data_slow block fills after ~(payload_cap / record) averaged records, and each averaged
    // record is SLOW_WINDOW fast samples — so scale the pump bound to the block size (a larger
    // SD_LOG_BLOCK_BYTES needs proportionally more records to flush one block). 3x margin.
    const int MAX_PUMP_RECORDS = static_cast<int>(
        (logic::telemetry::SD_BLOCK_PAYLOAD_CAP / sizeof(EcuSystemState) + 1)
        * logic::telemetry::detail::SLOW_WINDOW * 3);
    for (int i = 0; i < MAX_PUMP_RECORDS && storage_slow_.writes.empty(); ++i) {
        adc_.push(sample);
        controller_.produceRecord(++now_ms_);
        step();
    }
    EXPECT_FALSE(storage_slow_.writes.empty());  // averaged records reached data_slow.bin
    EXPECT_TRUE(storage_fast_.writes.empty());    // fast file untouched in slow mode
}

/* The recording rate is driven by the state machine on the ECU too (it runs the SAME
   transitionTo as the FCU): the burn window (Ignite/Launch/Abort) arms fast, resting states
   fall back to slow. See logic::control::isFastRecordingState. */
TEST_F(EcuControllerTest, UnsafeToIgniteArmsFastRecording)
{
    deliver(makeSetState(State::Unsafe));
    EXPECT_FALSE(logic::control::base_control_flags.get(ControlFlagBase::FastRecording));  // Unsafe is resting
    deliver(makeSetState(State::Ignite));
    EXPECT_TRUE(logic::control::base_control_flags.get(ControlFlagBase::FastRecording));   // armed
}

TEST_F(EcuControllerTest, EnteringSafeFallsBackToSlowRecording)
{
    deliver(makeSetState(State::Unsafe));
    deliver(makeSetState(State::Abort));   // armed fast
    ASSERT_TRUE(logic::control::base_control_flags.get(ControlFlagBase::FastRecording));
    deliver(makeSetState(State::Safe));
    EXPECT_FALSE(logic::control::base_control_flags.get(ControlFlagBase::FastRecording));  // back to slow
}

/* A warm reboot resuming Launch re-arms fast logging (flags are not battery-backed). */
TEST_F(EcuControllerTest, ResumeIntoLaunchReArmsFastRecording)
{
    logic::control::persistent_state.saveState(State::Launch);
    controller_.init();
    ASSERT_EQ(current(), State::Launch);
    EXPECT_TRUE(logic::control::base_control_flags.get(ControlFlagBase::FastRecording));
}

/* ---- Telemetry (produce + drain + fragment onto CAN) --------------------- */

TEST_F(EcuControllerTest, TelemetryIsDownlinkedToTheFcuOverCan)
{
    step();  // Init -> Safe
    bus().can_tx.clear();

    // Pump conversions until a full SystemState half drains onto the bus. The low-rate
    // ExtendedSystemState rides the same bus (~10 Hz, separate payload_id), so scan for the
    // SystemState frame rather than assuming the first frame on the bus is one.
    bool      found = false;
    CanHeader header;
    const AdcInfo sample{};
    for (int i = 0; i < 4000 && !found; ++i) {
        adc_.push(sample);                     // one conversion into the ADC ring
        controller_.produceRecord(++now_ms_);  // drain it into the telemetry buffer
        step();                                // drainTick flushes any full half over CAN
        for (const auto& f : bus().can_tx) {
            CanHeader h;
            h.code = f.id;
            if (static_cast<TelemetryType>(h.frame.payload_id) == TelemetryType::SystemState) {
                header = h;
                found  = true;
                break;
            }
        }
    }

    ASSERT_TRUE(found) << "a full telemetry half never downlinked a SystemState over CAN";
    EXPECT_EQ(static_cast<BoardId>(header.frame.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Telemetry);
    EXPECT_EQ(static_cast<State>(header.frame.sender_state), State::Safe);  // ECU stamps its state
}

/* The low-rate ExtendedSystemState is downlinked to the FCU over CAN too — unbatched, one
   send per ~10 Hz record (separate payload_id from the batched SystemState stream) — so the
   FCU can relay the ECU's slow state straight to the GS. Cross the extended interval, then
   find + decode the ExtendedSystemState frame off the bus and read its live control flags. */
TEST_F(EcuControllerTest, ExtendedStateIsDownlinkedToTheFcuOverCan)
{
    step();  // Init -> Safe
    bus().can_tx.clear();
    logic::control::base_control_flags.set(ControlFlagBase::FastRecording, true);  // a distinctive live flag

    for (int i = 0; i < 150; ++i) {  // cross the ~10 Hz extended interval (100 ms); each step +1 ms
        step();
    }

    bool                   found = false;
    CanHeader              header;
    EcuExtendedSystemState ext{};
    for (const auto& f : bus().can_tx) {
        CanHeader h;
        h.code = f.id;
        if (static_cast<TelemetryType>(h.frame.payload_id) != TelemetryType::ExtendedSystemState) {
            continue;
        }
        header = h;
        std::memcpy(&ext, &f.data[1], sizeof(ext));  // data[0] is the fragment index; record follows
        found  = true;
        break;
    }

    ASSERT_TRUE(found) << "no ExtendedSystemState downlinked over CAN";
    EXPECT_EQ(static_cast<BoardId>(header.frame.sender_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Telemetry);
    EXPECT_EQ(static_cast<State>(header.frame.sender_state), State::Safe);  // ECU stamps its state
    EXPECT_NE(ext.base.control_flags_base & (1u << static_cast<uint8_t>(ControlFlagBase::FastRecording)), 0u);
}

/* The ECU records refused commands too (parity with the FCU): a refused SetState (Safe's only
   exit is Unsafe) and a refused SetControlFlag (the ECU has no per-board flags, so a per-board
   id is rejected) both ride the ECU's ExtendedSystemState via the shared base. */
TEST_F(EcuControllerTest, RefusedCommandsRideTheExtendedRecord)
{
    step();  // Init -> Safe
    deliver(makeSetState(State::Ignite));                       // Safe -> Ignite: refused
    deliver(makeSetControlFlag(/*per-board id=*/CONTROL_FLAG_BOARD_OFFSET, /*value=*/1));  // ECU has none: refused
    deliver(makeValveCmd(EcuValves::NOS, ValveCommand::Close)); // valve command in Safe (not Unsafe): refused
    bus().can_tx.clear();

    EcuExtendedSystemState ext{};
    bool found = false;
    for (int i = 0; i < 150 && !found; ++i) {
        step();
        for (const auto& f : bus().can_tx) {
            CanHeader h;
            h.code = f.id;
            if (static_cast<TelemetryType>(h.frame.payload_id) != TelemetryType::ExtendedSystemState) {
                continue;
            }
            std::memcpy(&ext, &f.data[1], sizeof(ext));
            found = true;
            break;
        }
    }

    ASSERT_TRUE(found) << "no ExtendedSystemState downlinked over CAN";
    const auto& rc = ext.base.refused_command_info;
    EXPECT_EQ(rc.set_state_from, static_cast<uint8_t>(State::Safe));
    EXPECT_EQ(rc.set_state_to,   static_cast<uint8_t>(State::Ignite));
    EXPECT_EQ(rc.set_state_refused_count, 1u);
    EXPECT_EQ(rc.set_flag_id,    static_cast<uint16_t>(CONTROL_FLAG_BOARD_OFFSET));
    EXPECT_EQ(rc.set_flag_value, 1u);
    EXPECT_EQ(rc.set_flag_state, static_cast<uint8_t>(State::Safe));
    EXPECT_EQ(rc.set_flag_refused_count, 1u);
    EXPECT_EQ(rc.set_valve_id,    static_cast<uint8_t>(EcuValves::NOS));
    EXPECT_EQ(rc.set_valve_action, static_cast<uint8_t>(ValveCommand::Close));
    EXPECT_EQ(rc.set_valve_state, static_cast<uint8_t>(State::Safe));
    EXPECT_EQ(rc.set_valve_refused_count, 1u);
}

/* ---- Reload (Backup SRAM resume) ----------------------------------------- */

TEST_F(EcuControllerTest, ReloadLaunchOpensBothValves)
{
    /* A reset while LAUNCH re-executes the entry transition (Ignite -> Launch), driving both
       propellant valves OPEN to match the resumed state — they MUST be opened on a reload. */
    logic::control::persistent_state.saveState(State::Launch);
    ipa_valve_.open_calls = ipa_valve_.close_calls = 0;
    nos_valve_.open_calls = nos_valve_.close_calls = 0;

    controller_.init();

    EXPECT_EQ(current(), State::Launch);
    EXPECT_EQ(ipa_valve_.open_calls, 1);
    EXPECT_EQ(nos_valve_.open_calls, 1);
    EXPECT_EQ(ipa_valve_.close_calls, 0);
    EXPECT_EQ(nos_valve_.close_calls, 0);
}

TEST_F(EcuControllerTest, ReloadAbortClosesBothValves)
{
    /* A reset while ABORT re-executes the abort actuation (close both valves). */
    logic::control::persistent_state.saveState(State::Abort);
    ipa_valve_.open_calls = ipa_valve_.close_calls = 0;
    nos_valve_.open_calls = nos_valve_.close_calls = 0;

    controller_.init();

    EXPECT_EQ(current(), State::Abort);
    EXPECT_EQ(ipa_valve_.close_calls, 1);
    EXPECT_EQ(nos_valve_.close_calls, 1);
    EXPECT_EQ(ipa_valve_.open_calls, 0);
    EXPECT_EQ(nos_valve_.open_calls, 0);
}

TEST_F(EcuControllerTest, DoesNotResumeNonInProgressState)
{
    /* Unsafe is not one of Abort/Launch/Ignite: it is NOT resumed — the ECU boots fresh to Safe
       (closing both valves), not back into Unsafe. */
    logic::control::persistent_state.saveState(State::Unsafe);
    ipa_valve_.open_calls = ipa_valve_.close_calls = 0;
    nos_valve_.open_calls = nos_valve_.close_calls = 0;

    controller_.init();

    EXPECT_EQ(current(), State::Safe);
    EXPECT_EQ(ipa_valve_.close_calls, 1);
    EXPECT_EQ(nos_valve_.close_calls, 1);
}

} // namespace
