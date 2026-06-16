/* ------------------------------------------------------------------------- *
 * Unit tests for the FCU filling-station state machine (logic::fcu).
 *
 * The logic is exercised purely through its public surface (init/tick) and the
 * communication interfaces, which the FakeBus stands in for. The state machine
 * keeps its state in battery-backed persistent_state; telemetry is no longer a
 * per-tick heartbeat (the TelemetryType::SystemState SystemState packet is now produced on the
 * record timer, see produceRecord), so the state-machine tests read the current
 * state straight from persistent_state via currentState().
 * ------------------------------------------------------------------------- */

#include "fcu_controller.hpp"
#include "support/fakes.hpp"   // the standard set of host test doubles

#include "communication/interfaces/can.hpp"
#include "communication/interfaces/ethernet.hpp"
#include "communication/protocol/framing/ethernet_header.hpp"   // EthernetHeader (downlink header)
#include "control/persistent_state.hpp"
#include "system/state.hpp"
#include "framing/can_header.hpp"
#include "framing/payload_type.hpp"

#include "system/state.hpp"
#include "telemetry/telemetry_type.hpp"
#include "system/board_id.hpp"
#include "command/command_type.hpp"
#include "command/set_control_flag.hpp"            // ControlFlag, SetControlFlagFrame
#include "control/refused_control_flag.hpp"        // last_refused_control_flag (reset + asserted)
#include "communication/system_state_codec.hpp"   // packSystemState (ECU telemetry fragments)
#include "communication/protocol/telemetry/ecu_system_state.hpp"   // EcuSystemState
#include "communication/protocol/telemetry/fcu_system_state.hpp"   // FcuSystemState
#include "communication/protocol/telemetry/fcu_extended_system_state.hpp"   // FcuExtendedSystemState (thermocouples)
#include "communication/protocol/telemetry/ecu_extended_system_state.hpp"   // EcuExtendedSystemState (relayed from the ECU)
#include "data_integrity/crc32.hpp"   // logic::data_integrity::crc32 (validate the downlink CRC coverage)
#include "support/datagram_crc.hpp"   // appendGsCrc (inbound datagrams carry a CRC)

#include <array>
#include <span>

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

using logic::communication::CanFrame;
using logic::communication::Endpoint;
using logic::communication::command::CommandType;

namespace {

/* Build a CommandType::SetState datagram addressed to `device`, asking for `requested`:
   a 12-byte EthernetHeader followed by the 2-byte SetStateFrame (the canonical wire
   layout the Ethernet command parser reads). */
std::vector<uint8_t> makeStateRequest(BoardId device, logic::control::State requested)
{
    EthernetHeader header = {};
    header.target_id    = static_cast<uint8_t>(device);
    header.payload_type = static_cast<uint8_t>(PayloadType::Command);
    header.payload_id   = static_cast<uint8_t>(CommandType::SetState);
    header.payload_size_bytes = static_cast<uint16_t>(sizeof(SetStateFrame));

    const SetStateFrame body{0, static_cast<uint8_t>(requested)};

    std::vector<uint8_t> payload(sizeof(EthernetHeader) + sizeof(SetStateFrame));
    std::memcpy(payload.data(), &header, sizeof(EthernetHeader));
    std::memcpy(payload.data() + sizeof(EthernetHeader), &body, sizeof(SetStateFrame));
    appendGsCrc(payload);
    return payload;
}

/* The 16-bit global flag id of the FCU solenoid valve (a per-board flag: offset + its bit). */
constexpr uint16_t SOLENOID_FLAG_ID =
    CONTROL_FLAG_BOARD_OFFSET + static_cast<uint16_t>(FcuControlFlag::SolenoidValve);

/* Build a CommandType::SetControlFlag datagram addressed to `device`: a 12-byte
   EthernetHeader followed by the SetControlFlagFrame {16-bit flag id, value}. */
std::vector<uint8_t> makeControlFlagRequest(BoardId device, uint16_t flag, uint8_t value)
{
    EthernetHeader header = {};
    header.target_id    = static_cast<uint8_t>(device);
    header.payload_type = static_cast<uint8_t>(PayloadType::Command);
    header.payload_id   = static_cast<uint8_t>(CommandType::SetControlFlag);
    header.payload_size_bytes = static_cast<uint16_t>(sizeof(SetControlFlagFrame));

    const SetControlFlagFrame body{flag, value, /*reserved=*/0};

    std::vector<uint8_t> payload(sizeof(EthernetHeader) + sizeof(SetControlFlagFrame));
    std::memcpy(payload.data(), &header, sizeof(EthernetHeader));
    std::memcpy(payload.data() + sizeof(EthernetHeader), &body, sizeof(SetControlFlagFrame));
    appendGsCrc(payload);
    return payload;
}

/* Build a CAN frame with the given header fields and payload. */
CanFrame makeCanFrame(BoardId sender, BoardId target, uint8_t messageId,
                      std::array<uint8_t, 8> data = {})
{
    CanHeader header        = {};
    header.frame.sender_id    = static_cast<uint8_t>(sender);
    header.frame.target_id    = static_cast<uint8_t>(target);
    header.frame.payload_type = static_cast<uint8_t>(PayloadType::Command);
    header.frame.payload_id   = messageId;

    CanFrame frame;
    frame.id     = header.code;
    std::memcpy(frame.data.data(), data.data(), data.size());  // 8 bytes into the FD-sized frame
    frame.length = 8;
    return frame;
}

class FcuControllerTest : public ::testing::Test {
protected:
    FakeStorage      storage_fast_;
    FakeStorage      storage_slow_;
    FakeStorage      storage_ext_;
    FakeValve        fill_valve_;
    FakeValve        dump_valve_;
    FakeStreamingAdc adc_;
    FakeEthernet     eth_;
    FakeCan          can_;
    FakeThermocoupleBank tc_;
    FakePowerMonitor     pm_;
    FakeEmatch           ematch_;
    FakeSolenoid         solenoid_;
    logic::fcu::Controller<FakeStorage, FakeValve, FakeStreamingAdc, FakeEthernet, FakeCan,
                           FakeThermocoupleBank, FakePowerMonitor, FakeEmatch, FakeSolenoid>
                     controller_{storage_fast_, storage_slow_, storage_ext_,
                                 fill_valve_, dump_valve_, adc_, eth_, can_, tc_, pm_, ematch_, solenoid_};
    uint32_t         now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        /* Start every test from a cold Backup SRAM so persisted state never
           leaks between tests; init() then commits a fresh INIT. */
        logic::control::persistent_state = logic::control::PersistentState{};
        logic::control::base_control_flags = logic::control::ControlFlags<ControlFlagBase>{};  // all base flags off
        logic::control::fcu_control_flags  = logic::control::ControlFlags<FcuControlFlag>{};   // all per-board flags off
        logic::control::last_refused_transition =
            {logic::control::State::Init, logic::control::State::Init};
        logic::control::refused_transition_count = 0;
        logic::control::last_refused_control_flag =
            {logic::control::REFUSED_CONTROL_FLAG_NONE, 0, logic::control::State::Init};
        logic::control::refused_control_flag_count = 0;
        controller_.init();
    }

    /* Advance the logic one tick with a strictly increasing timestamp. */
    void step() { controller_.tick(++now_ms_); }

    /* Advance to an absolute timestamp (must be > the current one). */
    void stepTo(uint32_t now) { controller_.tick(now_ms_ = now); }

    /* The controller's current state, read straight from the persisted snapshot
       (the state machine commits every transition there). */
    logic::control::State currentState() const
    {
        return logic::control::persistent_state.fill_state;
    }

    /* Confirm SAFE (init() already moved INIT -> SAFE), tick once, and clear recorded traffic. */
    void reachSafe()
    {
        step();
        ASSERT_EQ(currentState(), logic::control::State::Safe);
        bus().udp_tx.clear();
        bus().can_tx.clear();
    }

    /* Send a CommandType::SetState command (addressed to us) and tick once. */
    void requestState(logic::control::State requested, BoardId device = BoardId::FillingStation)
    {
        const auto payload = makeStateRequest(device, requested);
        bus().push_udp(Endpoint{}, payload);
        step();
    }

    /* Send a CommandType::SetControlFlag command (addressed to us) and tick once. */
    void requestControlFlag(uint16_t flag, uint8_t value, BoardId device = BoardId::FillingStation)
    {
        bus().push_udp(Endpoint{}, makeControlFlagRequest(device, flag, value));
        step();
    }
};

/* ---- Startup ------------------------------------------------------------- */

TEST_F(FcuControllerTest, InitEntersSafe)
{
    // Init -> Safe happens as soon as init() completes (cold boot), before any tick.
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

/* Telemetry is no longer a per-tick heartbeat: SystemState records are produced
   on the record timer (produceRecord) and batched into TelemetryType::SystemState datagrams when
   a 4096-byte half fills and drains. Pump records until a SystemState frame downlinks
   (the low-rate ExtendedSystemState shares the egress, so we filter by type) and check
   it is well-formed. */
TEST_F(FcuControllerTest, FullTelemetryBufferDownlinksGetSystem)
{
    reachSafe();  // also clears udp_tx

    auto findSystemState = []() -> const SentDatagram* {
        for (const auto& d : bus().udp_tx) {
            EthernetHeader h;
            std::memcpy(&h, d.payload.data(), sizeof(h));
            if (static_cast<TelemetryType>(h.payload_id) == TelemetryType::SystemState) {
                return &d;
            }
        }
        return nullptr;
    };

    const AdcInfo sample{};
    for (int i = 0; i < 2000 && findSystemState() == nullptr; ++i) {
        adc_.push(sample);                  // one conversion into the ADC ring
        controller_.produceRecord(++now_ms_);  // drain it into the telemetry buffer
        step();                             // drainTick flushes any full half
    }

    const SentDatagram* sys = findSystemState();
    ASSERT_NE(sys, nullptr) << "a full telemetry half never downlinked a SystemState";
    ASSERT_GE(sys->payload.size(), sizeof(EthernetHeader));
    EthernetHeader header;
    std::memcpy(&header, sys->payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(header.sender_id, static_cast<uint8_t>(BoardId::FillingStation));
    EXPECT_EQ(header.payload_id, static_cast<uint8_t>(TelemetryType::SystemState));
}

/* The FCU must downlink the FULL FcuSystemState (base + eth_info), not just the shared
   SystemStateBase. Mark the Ethernet info, pump a SystemState, and read the marker back
   from the record's eth_info — which only exists if the whole FcuSystemState was sent. */
TEST_F(FcuControllerTest, DownlinkCarriesTheFullFcuSystemStateNotJustTheBase)
{
    reachSafe();  // clears udp_tx
    eth_.info_value.rx_dropped = 0xBEEF;   // a marker that lives in eth_info, past the base

    auto findSystemState = []() -> const SentDatagram* {
        for (const auto& d : bus().udp_tx) {
            EthernetHeader h;
            std::memcpy(&h, d.payload.data(), sizeof(h));
            if (static_cast<TelemetryType>(h.payload_id) == TelemetryType::SystemState) {
                return &d;
            }
        }
        return nullptr;
    };

    const AdcInfo sample{};
    for (int i = 0; i < 2000 && findSystemState() == nullptr; ++i) {
        adc_.push(sample);
        controller_.produceRecord(++now_ms_);
        step();
    }

    const SentDatagram* sys = findSystemState();
    ASSERT_NE(sys, nullptr) << "no SystemState downlinked";
    // The datagram must hold at least one WHOLE FcuSystemState after the header (a base-
    // only record would be sizeof(SystemStateBase), too short to carry eth_info).
    ASSERT_GE(sys->payload.size(), sizeof(EthernetHeader) + sizeof(FcuSystemState));

    FcuSystemState record;
    std::memcpy(&record, sys->payload.data() + sizeof(EthernetHeader), sizeof(record));
    EXPECT_EQ(record.eth_info.rx_dropped, 0xBEEF)
        << "eth_info absent — the FCU downlinked only the SystemStateBase";
}

/* The 4 thermocouples ride the low-rate ExtendedSystemState (data_ext / 10 Hz), NOT
   the 2 kHz SystemState. Set a distinctive reading, cross the extended interval, and
   read it back out of the ExtendedSystemState datagram. */
TEST_F(FcuControllerTest, ThermocouplesDownlinkInTheLowRateExtendedRecord)
{
    reachSafe();  // also clears udp_tx

    ThermocoupleInfo tc{};
    tc.state              = ThermocoupleState::Active;
    tc.status.data_valid  = 1u;
    tc.thermocouple_code  = 0x4321;
    tc.cold_junction_code = 0x0210;
    tc_.set(0, tc);

    // Cross the ~10 Hz extended interval (100 ms) so produceExtended emits one record.
    controller_.tick(now_ms_ += 150);

    bool found = false;
    for (const auto& d : bus().udp_tx) {
        ASSERT_GE(d.payload.size(), sizeof(EthernetHeader));
        EthernetHeader h;
        std::memcpy(&h, d.payload.data(), sizeof(h));
        if (static_cast<TelemetryType>(h.payload_id) != TelemetryType::ExtendedSystemState) {
            continue;
        }
        ASSERT_GE(d.payload.size(), sizeof(EthernetHeader) + sizeof(FcuExtendedSystemState));
        FcuExtendedSystemState ext;
        std::memcpy(&ext, d.payload.data() + sizeof(EthernetHeader), sizeof(ext));
        EXPECT_EQ(ext.thermocouple_info[0].thermocouple_code, 0x4321);
        EXPECT_EQ(ext.thermocouple_info[0].cold_junction_code, 0x0210);
        EXPECT_EQ(static_cast<uint8_t>(ext.thermocouple_info[0].state),
                  static_cast<uint8_t>(ThermocoupleState::Active));
        found = true;
    }
    EXPECT_TRUE(found) << "no ExtendedSystemState was downlinked";
}

/* The INA3221 power monitor also rides the low-rate ExtendedSystemState. Script a reading,
   cross the extended interval, and read the per-channel codes back out of the datagram. */
TEST_F(FcuControllerTest, PowerMonitorDownlinksInTheLowRateExtendedRecord)
{
    reachSafe();  // also clears udp_tx

    PowerMonitorInfo pm{};
    pm.state                  = PowerMonitorState::Active;
    pm.status.data_valid      = 1u;
    pm.channels[0].shunt_code = 0x0123;
    pm.channels[0].bus_code   = 0x0456;
    pm.channels[2].bus_code   = 0x0789;
    pm_.set(pm);

    // Cross the ~10 Hz extended interval (100 ms) so produceExtended emits one record.
    controller_.tick(now_ms_ += 150);

    bool found = false;
    for (const auto& d : bus().udp_tx) {
        EthernetHeader h;
        std::memcpy(&h, d.payload.data(), sizeof(h));
        if (static_cast<TelemetryType>(h.payload_id) != TelemetryType::ExtendedSystemState) {
            continue;
        }
        ASSERT_GE(d.payload.size(), sizeof(EthernetHeader) + sizeof(FcuExtendedSystemState));
        FcuExtendedSystemState ext;
        std::memcpy(&ext, d.payload.data() + sizeof(EthernetHeader), sizeof(ext));
        EXPECT_EQ(ext.power_monitor.channels[0].shunt_code, 0x0123);
        EXPECT_EQ(ext.power_monitor.channels[0].bus_code, 0x0456);
        EXPECT_EQ(ext.power_monitor.channels[2].bus_code, 0x0789);
        EXPECT_EQ(static_cast<uint8_t>(ext.power_monitor.state),
                  static_cast<uint8_t>(PowerMonitorState::Active));
        found = true;
    }
    EXPECT_TRUE(found) << "no ExtendedSystemState was downlinked";
}

/* The ExtendedSystemState carries the live control-flag bitmask, so the GS reads the
   recording config straight from telemetry (bit N = ControlFlag value N). */
TEST_F(FcuControllerTest, ExtendedRecordReportsTheLiveControlFlags)
{
    reachSafe();  // clears udp_tx
    logic::control::base_control_flags.set(ControlFlagBase::FastRecording, true);  // PersistingData stays off

    controller_.tick(now_ms_ += 150);  // cross the extended interval

    bool found = false;
    for (const auto& d : bus().udp_tx) {
        EthernetHeader h;
        std::memcpy(&h, d.payload.data(), sizeof(h));
        if (static_cast<TelemetryType>(h.payload_id) != TelemetryType::ExtendedSystemState) {
            continue;
        }
        FcuExtendedSystemState ext;
        std::memcpy(&ext, d.payload.data() + sizeof(EthernetHeader), sizeof(ext));
        EXPECT_NE(ext.base.control_flags_base & (1u << static_cast<uint8_t>(ControlFlagBase::FastRecording)), 0u);
        EXPECT_EQ(ext.base.control_flags_base & (1u << static_cast<uint8_t>(ControlFlagBase::PersistingData)), 0u);
        found = true;
    }
    EXPECT_TRUE(found) << "no ExtendedSystemState was downlinked";
}

/* The controller advances the non-blocking thermocouple bank from its foreground
   tick, so readings refresh without the record-timer ISR ever touching SPI. */
TEST_F(FcuControllerTest, EachTickServicesTheThermocoupleBank)
{
    const uint32_t before = tc_.service_calls;
    step();
    step();
    EXPECT_GT(tc_.service_calls, before);
}

/* The downlink CRC covers the EthernetHeader + payload (everything up to the CRC), and
   is appended little-endian — the contract the GS must reproduce. */
TEST_F(FcuControllerTest, DownlinkCrcCoversHeaderAndPayload)
{
    reachSafe();
    controller_.tick(now_ms_ += 150);   // emit at least the ExtendedSystemState datagram
    ASSERT_FALSE(bus().udp_tx.empty());

    const auto& dg = bus().udp_tx.front().payload;
    ASSERT_GE(dg.size(), sizeof(EthernetHeader) + sizeof(uint32_t));

    const std::size_t covered  = dg.size() - sizeof(uint32_t);   // header + payload, all but the CRC
    const uint32_t    expected = logic::data_integrity::crc32(dg.data(), covered);
    uint32_t received = 0;
    std::memcpy(&received, dg.data() + covered, sizeof(received));   // appended little-endian
    EXPECT_EQ(received, expected) << "downlink CRC must cover the EthernetHeader + payload, LE";
}

/* An inbound datagram whose CRC does not check out is dropped before it reaches the
   command handler; the same datagram with a good CRC is applied. */
TEST_F(FcuControllerTest, InboundDatagramWithBadCrcIsRejected)
{
    reachSafe();   // state SAFE; SAFE -> UNSAFE is a legal transition

    std::vector<uint8_t> good = makeStateRequest(BoardId::FillingStation, logic::control::State::Unsafe);
    std::vector<uint8_t> bad  = good;
    bad.back() ^= 0xFFu;   // corrupt the trailing CRC

    bus().push_udp(Endpoint{}, bad);
    step();
    EXPECT_EQ(currentState(), logic::control::State::Safe) << "a bad-CRC command must be ignored";

    bus().push_udp(Endpoint{}, good);
    step();
    EXPECT_EQ(currentState(), logic::control::State::Unsafe) << "the same command with a valid CRC applies";
}

TEST_F(FcuControllerTest, EveryTickServicesTheLink)
{
    step();
    step();
    step();
    EXPECT_EQ(bus().udp_tick_count, 3);
}

/* ---- Valid state transitions -------------------------------------------- */

TEST_F(FcuControllerTest, SafeToUnsafe)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    EXPECT_EQ(currentState(), logic::control::State::Unsafe);
}

TEST_F(FcuControllerTest, IgniteToLaunch)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Ignite);
    requestState(logic::control::State::Launch);
    EXPECT_EQ(currentState(), logic::control::State::Launch);
}

TEST_F(FcuControllerTest, LaunchBackToSafe)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Ignite);
    requestState(logic::control::State::Launch);
    requestState(logic::control::State::Safe);
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

TEST_F(FcuControllerTest, UnsafeToIgnite)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Ignite);
    EXPECT_EQ(currentState(), logic::control::State::Ignite);
}

TEST_F(FcuControllerTest, UnsafeToAbort)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Abort);
    EXPECT_EQ(currentState(), logic::control::State::Abort);
}

TEST_F(FcuControllerTest, IgniteToAbort)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Ignite);
    requestState(logic::control::State::Abort);
    EXPECT_EQ(currentState(), logic::control::State::Abort);
}

TEST_F(FcuControllerTest, AbortBackToSafe)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Abort);
    requestState(logic::control::State::Safe);
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

/* ---- Rejected state transitions ----------------------------------------- */

TEST_F(FcuControllerTest, SafeRejectsIgnite)
{
    reachSafe();
    requestState(logic::control::State::Ignite);
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

TEST_F(FcuControllerTest, SafeRejectsAbort)
{
    reachSafe();
    requestState(logic::control::State::Abort);   // people near: even abort is not permitted
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

TEST_F(FcuControllerTest, SafeRejectsTest)
{
    reachSafe();
    requestState(logic::control::State::Test);   // Safe's only exit is Unsafe
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

TEST_F(FcuControllerTest, UnsafeRejectsTest)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Test);
    EXPECT_EQ(currentState(), logic::control::State::Unsafe);
}

TEST_F(FcuControllerTest, CommandForAnotherBoardIsIgnored)
{
    reachSafe();
    requestState(logic::control::State::Unsafe, BoardId::Engine);
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

TEST_F(FcuControllerTest, BroadcastCommandIsAccepted)
{
    reachSafe();
    requestState(logic::control::State::Unsafe, BoardId::Broadcast);
    EXPECT_EQ(currentState(), logic::control::State::Unsafe);
}

TEST_F(FcuControllerTest, RefusedTransitionAppearsInExtendedSystemState)
{
    reachSafe();
    requestState(logic::control::State::Ignite);   // Safe's only exit is Unsafe -> refused
    ASSERT_EQ(currentState(), logic::control::State::Safe);
    bus().udp_tx.clear();

    // Pump time forward until a low-rate ExtendedSystemState downlinks, then decode it.
    FcuExtendedSystemState ext{};
    bool found = false;
    for (int i = 0; i < 200 && !found; ++i) {
        stepTo(now_ms_ + 50);
        for (const auto& d : bus().udp_tx) {
            EthernetHeader h;
            std::memcpy(&h, d.payload.data(), sizeof(h));
            if (static_cast<TelemetryType>(h.payload_id) == TelemetryType::ExtendedSystemState) {
                std::memcpy(&ext, d.payload.data() + sizeof(EthernetHeader), sizeof(ext));
                found = true;
                break;
            }
        }
    }
    ASSERT_TRUE(found) << "no ExtendedSystemState downlinked";
    EXPECT_EQ(ext.base.refused_command_info.set_state_from, static_cast<uint8_t>(logic::control::State::Safe));
    EXPECT_EQ(ext.base.refused_command_info.set_state_to,   static_cast<uint8_t>(logic::control::State::Ignite));
    EXPECT_EQ(ext.base.refused_command_info.set_state_refused_count, 1u);   // one refusal so far
}

/* ---- CAN ------------------------------------------------------------------ *
 * The FCU receives only status/telemetry from the ECU over CAN — never commands
 * (those arrive over Ethernet) — so it never answers on the bus; it just drains
 * the RX ring each tick. Consuming valve status into the state machine is a
 * later step. */

TEST_F(FcuControllerTest, IncomingCanFrameProducesNoReply)
{
    bus().push_can(makeCanFrame(BoardId::Engine, BoardId::FillingStation, static_cast<uint8_t>(TelemetryType::SystemState)));
    step();
    EXPECT_TRUE(bus().can_tx.empty());
}

TEST_F(FcuControllerTest, AllQueuedCanFramesAreDrainedInOneTick)
{
    bus().push_can(makeCanFrame(BoardId::Engine, BoardId::FillingStation, static_cast<uint8_t>(TelemetryType::SystemState)));
    bus().push_can(makeCanFrame(BoardId::Engine, BoardId::FillingStation, static_cast<uint8_t>(TelemetryType::SystemState)));
    bus().push_can(makeCanFrame(BoardId::Engine, BoardId::FillingStation, static_cast<uint8_t>(TelemetryType::SystemState)));
    step();
    EXPECT_TRUE(bus().can_rx.empty());  // all drained in one tick
    EXPECT_TRUE(bus().can_tx.empty());  // and never answered
}

/* ECU telemetry, fragmented over CAN exactly as the ECU sends it, must be reassembled and
   relayed to the GS tagged as the ECU's (sender BoardId::Engine) — and BATCHED like our own
   telemetry: records accumulate in a relay half and stream out only once it fills, NOT one tiny
   datagram per record. This drives the real codec end to end and pins the relay path the GS
   depends on. */
TEST_F(FcuControllerTest, EcuTelemetryIsBatchedAndRelayedToGs)
{
    namespace codec = logic::communication::can;
    reachSafe();  // clears udp_tx / can_tx

    // The ECU stamps its state-machine state into each fragment's header; the relay must carry
    // it onto the GS datagram (EthernetHeader.sender_state).
    constexpr auto ECU_STATE = static_cast<uint8_t>(logic::control::State::Unsafe);

    // The first relayed datagram tagged as the ECU's (own telemetry shares the egress, so
    // filter by sender_id == Engine).
    auto findEngineSystemState = []() -> const SentDatagram* {
        for (const auto& d : bus().udp_tx) {
            EthernetHeader h;
            std::memcpy(&h, d.payload.data(), sizeof(h));
            if (h.sender_id == static_cast<uint8_t>(BoardId::Engine) &&
                static_cast<TelemetryType>(h.payload_id) == TelemetryType::SystemState) {
                return &d;
            }
        }
        return nullptr;
    };

    // Stream reassembled ECU records until a relay half fills and drains. A single record no
    // longer goes out on its own — the relay batches into a half buffer like our own telemetry.
    EcuSystemState record{};
    record.base.creation_timestamp_ms = 0xABCDEF01;  // marker on every record
    for (int i = 0; i < 4000 && findEngineSystemState() == nullptr; ++i) {
        std::array<CanFrame, codec::SYSTEM_STATE_FRAGMENTS> frames;
        codec::packSystemState(record, BoardId::Engine, BoardId::FillingStation, ECU_STATE,
                               static_cast<uint8_t>(i & 0x0F),
                               std::span<CanFrame, codec::SYSTEM_STATE_FRAGMENTS>(frames));
        for (const auto& f : frames) {
            bus().push_can(f);
        }
        step();  // drains CAN, reassembles + appends; drainRelayedEcu sends any FULL half
    }

    const SentDatagram* relayed = findEngineSystemState();
    ASSERT_NE(relayed, nullptr) << "a full relay half never downlinked to the GS";

    EthernetHeader header;
    std::memcpy(&header, relayed->payload.data(), sizeof(header));
    EXPECT_EQ(header.sender_id, static_cast<uint8_t>(BoardId::Engine));  // tagged as the ECU's
    EXPECT_EQ(header.payload_id, static_cast<uint8_t>(TelemetryType::SystemState));
    EXPECT_EQ(header.sender_state, ECU_STATE);  // the ECU's state relayed through to the GS

    // Batched, not one-per-record: the datagram carries many WHOLE records (the point of the fix).
    EXPECT_EQ(header.payload_size_bytes % sizeof(EcuSystemState), 0u);
    EXPECT_GT(header.payload_size_bytes / sizeof(EcuSystemState), 1u);

    // And the bytes round-trip: the first relayed record is one we sent.
    EcuSystemState first{};
    std::memcpy(&first, relayed->payload.data() + sizeof(EthernetHeader), sizeof(EcuSystemState));
    EXPECT_EQ(first.base.creation_timestamp_ms, 0xABCDEF01u);  // bytes round-tripped intact
}

/* The ECU's low-rate ExtendedSystemState is relayed to the GS UNBATCHED — streamed on as soon
   as a record reassembles off CAN (unlike the batched SystemState relay above), tagged as the
   ECU's (sender BoardId::Engine) and carrying the ECU's state. One CAN record in -> one GS
   datagram out, same tick. */
TEST_F(FcuControllerTest, EcuExtendedStateIsRelayedToGsUnbatched)
{
    namespace codec = logic::communication::can;
    reachSafe();  // clears udp_tx / can_tx

    constexpr auto ECU_STATE = static_cast<uint8_t>(logic::control::State::Unsafe);

    EcuExtendedSystemState record{};
    record.base.creation_timestamp_ms = 0x0BADF00D;  // marker that must round-trip to the GS
    record.base.control_flags_base    = 0xA5;
    std::array<CanFrame, codec::EXTENDED_STATE_FRAGMENTS> frames;
    codec::packExtendedSystemState(record, BoardId::Engine, BoardId::FillingStation, ECU_STATE,
                                   /*seq=*/0,
                                   std::span<CanFrame, codec::EXTENDED_STATE_FRAGMENTS>(frames));
    for (const auto& f : frames) {
        bus().push_can(f);
    }
    step();  // drains CAN, reassembles the extended record, relays it straight to the GS

    const SentDatagram* relayed = nullptr;
    for (const auto& d : bus().udp_tx) {
        EthernetHeader h;
        std::memcpy(&h, d.payload.data(), sizeof(h));
        if (h.sender_id == static_cast<uint8_t>(BoardId::Engine) &&
            static_cast<TelemetryType>(h.payload_id) == TelemetryType::ExtendedSystemState) {
            relayed = &d;
            break;
        }
    }
    ASSERT_NE(relayed, nullptr) << "the ECU ExtendedSystemState was not relayed to the GS";

    EthernetHeader header;
    std::memcpy(&header, relayed->payload.data(), sizeof(header));
    EXPECT_EQ(header.sender_state, ECU_STATE);  // the ECU's state relayed through to the GS
    // Unbatched: exactly one whole record in the datagram (not a half-buffer of many).
    EXPECT_EQ(header.payload_size_bytes, sizeof(EcuExtendedSystemState));

    EcuExtendedSystemState out{};
    std::memcpy(&out, relayed->payload.data() + sizeof(EthernetHeader), sizeof(out));
    EXPECT_EQ(out.base.creation_timestamp_ms, 0x0BADF00Du);  // bytes round-tripped intact
    EXPECT_EQ(out.base.control_flags_base, 0xA5u);
}

/* ---- E-match (igniter) --------------------------------------------------- */

/* The firing line is energised on Unsafe -> Ignite and the energise tick is recorded. */
TEST_F(FcuControllerTest, EmatchEnergisesEnteringIgnite)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    ASSERT_EQ(currentState(), logic::control::State::Unsafe);
    EXPECT_FALSE(ematch_.energised);   // not armed until Ignite

    requestState(logic::control::State::Ignite);
    ASSERT_EQ(currentState(), logic::control::State::Ignite);
    EXPECT_TRUE(ematch_.energised);              // firing line high in Ignite
    EXPECT_EQ(ematch_.energise_calls, 1u);
    EXPECT_NE(ematch_.last_energised_ms, 0u);    // stamped with the transition tick
}

/* Leaving Ignite to Launch drops the firing line and records the deenergise tick. */
TEST_F(FcuControllerTest, EmatchDeenergisesOnIgniteToLaunch)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Ignite);
    ASSERT_TRUE(ematch_.energised);

    const uint32_t deenergises_before = ematch_.deenergise_calls;
    requestState(logic::control::State::Launch);
    ASSERT_EQ(currentState(), logic::control::State::Launch);
    EXPECT_FALSE(ematch_.energised);                              // dropped on leaving Ignite
    EXPECT_EQ(ematch_.deenergise_calls, deenergises_before + 1);
    EXPECT_NE(ematch_.last_deenergised_ms, 0u);
}

/* Safety: leaving Ignite by Abort (not just Launch) also drops the firing line. */
TEST_F(FcuControllerTest, EmatchDeenergisesOnIgniteToAbort)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Ignite);
    ASSERT_TRUE(ematch_.energised);

    requestState(logic::control::State::Abort);
    ASSERT_EQ(currentState(), logic::control::State::Abort);
    EXPECT_FALSE(ematch_.energised);   // never left hot once Ignite is exited
}

/* The e-match presence (detect) is sampled each tick and rides the extended record. */
TEST_F(FcuControllerTest, EmatchPresenceRidesTheExtendedRecord)
{
    reachSafe();
    ematch_.detect_present = true;       // an e-match is plugged in
    bus().udp_tx.clear();
    controller_.tick(now_ms_ += 150);    // cross the ~10 Hz extended interval -> one record

    bool found = false;
    for (const auto& d : bus().udp_tx) {
        EthernetHeader h;
        std::memcpy(&h, d.payload.data(), sizeof(h));
        if (static_cast<TelemetryType>(h.payload_id) != TelemetryType::ExtendedSystemState) {
            continue;
        }
        FcuExtendedSystemState ext;
        std::memcpy(&ext, d.payload.data() + sizeof(EthernetHeader), sizeof(ext));
        EXPECT_EQ(ext.ematch_info.status.detected, 1u);   // mirrored from the detect line
        found = true;
    }
    EXPECT_TRUE(found) << "no ExtendedSystemState was downlinked";
}

/* The firing-line state + the energise timeline ride the extended record. */
TEST_F(FcuControllerTest, EmatchFiringStateRidesTheExtendedRecord)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestState(logic::control::State::Ignite);
    const uint32_t energised_at = ematch_.last_energised_ms;

    bus().udp_tx.clear();
    controller_.tick(now_ms_ += 150);   // cross the extended interval while still in Ignite

    bool found = false;
    for (const auto& d : bus().udp_tx) {
        EthernetHeader h;
        std::memcpy(&h, d.payload.data(), sizeof(h));
        if (static_cast<TelemetryType>(h.payload_id) != TelemetryType::ExtendedSystemState) {
            continue;
        }
        FcuExtendedSystemState ext;
        std::memcpy(&ext, d.payload.data() + sizeof(EthernetHeader), sizeof(ext));
        EXPECT_EQ(ext.ematch_info.status.energised, 1u);
        EXPECT_EQ(ext.ematch_info.last_energised_ms, energised_at);
        found = true;
    }
    EXPECT_TRUE(found) << "no ExtendedSystemState was downlinked";
}

/* ---- Solenoid valve ------------------------------------------------------ */

/* The solenoid opens only while the SolenoidValve flag is set AND the board is in Unsafe:
   the flag alone (in Safe) does nothing; reaching Unsafe with it set opens the valve. */
TEST_F(FcuControllerTest, SolenoidOpensOnlyInUnsafeWhenFlagSet)
{
    reachSafe();
    logic::control::fcu_control_flags.set(FcuControlFlag::SolenoidValve, true);

    step();   // still Safe: flag set but not Unsafe -> stays closed
    EXPECT_FALSE(solenoid_.is_open);

    requestState(logic::control::State::Unsafe);   // Unsafe + flag set -> opens
    ASSERT_EQ(currentState(), logic::control::State::Unsafe);
    EXPECT_TRUE(solenoid_.is_open);
    EXPECT_NE(solenoid_.last_opened_ms, 0u);
}

/* In Unsafe but with the flag clear, the solenoid stays closed. */
TEST_F(FcuControllerTest, SolenoidStaysClosedInUnsafeWithoutFlag)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);   // flag never set
    EXPECT_FALSE(solenoid_.is_open);
}

/* Leaving Unsafe auto-closes the solenoid even though the flag is still set. */
TEST_F(FcuControllerTest, SolenoidClosesOnLeavingUnsafe)
{
    reachSafe();
    logic::control::fcu_control_flags.set(FcuControlFlag::SolenoidValve, true);
    requestState(logic::control::State::Unsafe);
    ASSERT_TRUE(solenoid_.is_open);

    requestState(logic::control::State::Ignite);   // leaving Unsafe -> closes
    ASSERT_EQ(currentState(), logic::control::State::Ignite);
    EXPECT_FALSE(solenoid_.is_open);
    EXPECT_NE(solenoid_.last_closed_ms, 0u);
}

/* The solenoid presence + open state ride the extended record. */
TEST_F(FcuControllerTest, SolenoidStateRidesTheExtendedRecord)
{
    reachSafe();
    logic::control::fcu_control_flags.set(FcuControlFlag::SolenoidValve, true);
    solenoid_.detect_present = true;               // wired up
    requestState(logic::control::State::Unsafe);   // opens
    ASSERT_TRUE(solenoid_.is_open);

    bus().udp_tx.clear();
    controller_.tick(now_ms_ += 150);   // cross the extended interval while still in Unsafe

    bool found = false;
    for (const auto& d : bus().udp_tx) {
        EthernetHeader h;
        std::memcpy(&h, d.payload.data(), sizeof(h));
        if (static_cast<TelemetryType>(h.payload_id) != TelemetryType::ExtendedSystemState) {
            continue;
        }
        FcuExtendedSystemState ext;
        std::memcpy(&ext, d.payload.data() + sizeof(EthernetHeader), sizeof(ext));
        EXPECT_EQ(ext.solenoid_info.status.open, 1u);
        EXPECT_EQ(ext.solenoid_info.status.detected, 1u);
        found = true;
    }
    EXPECT_TRUE(found) << "no ExtendedSystemState was downlinked";
}

/* A SetControlFlag(SolenoidValve) command is REJECTED outside Unsafe: the flag is not
   applied, the solenoid stays closed, and the refusal (flag + value + state) is recorded. */
TEST_F(FcuControllerTest, SolenoidFlagCommandRejectedOutsideUnsafe)
{
    reachSafe();   // Safe, not Unsafe
    requestControlFlag(SOLENOID_FLAG_ID, 1);

    EXPECT_FALSE(logic::control::fcu_control_flags.get(FcuControlFlag::SolenoidValve));  // not applied
    EXPECT_FALSE(solenoid_.is_open);
    EXPECT_EQ(logic::control::last_refused_control_flag.flag,
              SOLENOID_FLAG_ID);
    EXPECT_EQ(logic::control::last_refused_control_flag.value, 1u);
    EXPECT_EQ(logic::control::last_refused_control_flag.state, logic::control::State::Safe);
}

/* In Unsafe the command is honoured: the flag is applied and the solenoid opens. */
TEST_F(FcuControllerTest, SolenoidFlagCommandAcceptedInUnsafe)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestControlFlag(SOLENOID_FLAG_ID, 1);

    EXPECT_TRUE(logic::control::fcu_control_flags.get(FcuControlFlag::SolenoidValve));
    EXPECT_TRUE(solenoid_.is_open);
}

/* Leaving Unsafe clears the SolenoidValve flag, so a later re-entry into Unsafe does NOT
   re-open the valve without a fresh command. */
TEST_F(FcuControllerTest, LeavingUnsafeClearsSolenoidFlag)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    requestControlFlag(SOLENOID_FLAG_ID, 1);
    ASSERT_TRUE(solenoid_.is_open);

    requestState(logic::control::State::Abort);   // leaving Unsafe
    EXPECT_FALSE(logic::control::fcu_control_flags.get(FcuControlFlag::SolenoidValve));  // flag cleared
    EXPECT_FALSE(solenoid_.is_open);
}

/* The refused SetControlFlag (flag id + value + the state it was refused in) rides the
   extended record, alongside the refused SetState. */
TEST_F(FcuControllerTest, RefusedControlFlagAppearsInExtendedSystemState)
{
    reachSafe();
    requestControlFlag(SOLENOID_FLAG_ID, 1);   // refused in Safe
    bus().udp_tx.clear();
    controller_.tick(now_ms_ += 150);

    bool found = false;
    for (const auto& d : bus().udp_tx) {
        EthernetHeader h;
        std::memcpy(&h, d.payload.data(), sizeof(h));
        if (static_cast<TelemetryType>(h.payload_id) != TelemetryType::ExtendedSystemState) {
            continue;
        }
        FcuExtendedSystemState ext;
        std::memcpy(&ext, d.payload.data() + sizeof(EthernetHeader), sizeof(ext));
        EXPECT_EQ(ext.base.refused_command_info.set_flag_id,
                  SOLENOID_FLAG_ID);
        EXPECT_EQ(ext.base.refused_command_info.set_flag_value, 1u);
        EXPECT_EQ(ext.base.refused_command_info.set_flag_state,
                  static_cast<uint8_t>(logic::control::State::Safe));
        EXPECT_EQ(ext.base.refused_command_info.set_flag_refused_count, 1u);   // one refusal so far
        found = true;
    }
    EXPECT_TRUE(found) << "no ExtendedSystemState was downlinked";
}

/* Refused SetState and SetControlFlag commands are counted (independently) across the run. */
TEST_F(FcuControllerTest, RefusedCommandCountsAccumulate)
{
    reachSafe();
    requestState(logic::control::State::Ignite);   // Safe -> Ignite: refused (1)
    requestState(logic::control::State::Launch);   // Safe -> Launch: refused (2)
    EXPECT_EQ(logic::control::refused_transition_count, 2u);

    requestControlFlag(SOLENOID_FLAG_ID, 1);   // refused in Safe (1)
    requestControlFlag(SOLENOID_FLAG_ID, 0);   // refused in Safe (2)
    EXPECT_EQ(logic::control::refused_control_flag_count, 2u);
    EXPECT_EQ(logic::control::refused_transition_count, 2u);   // unaffected by flag refusals
}

/* ---- Persistent state (Backup SRAM) -------------------------------------- */

TEST_F(FcuControllerTest, StateTransitionIsPersisted)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);

    const auto loaded = logic::control::persistent_state.loadState();
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(*loaded, logic::control::State::Unsafe);
}

TEST_F(FcuControllerTest, ResumesPersistedStateOnInit)
{
    /* Simulate a reset while UNSAFE: the blob is already committed in Backup SRAM. */
    logic::control::persistent_state.saveState(logic::control::State::Unsafe);

    controller_.init();  // reboot resumes the persisted state (before any tick advances it)
    EXPECT_EQ(currentState(), logic::control::State::Unsafe);
}

TEST_F(FcuControllerTest, ColdBootWithInvalidBlobDoesNotResumeStaleState)
{
    logic::control::persistent_state.saveState(logic::control::State::Ignite);
    logic::control::persistent_state.magic = 0;  // corrupt => looks like cold garbage

    controller_.init();  // cold/corrupt boot ignores the stale blob and inits fresh: Init -> Safe
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

} // namespace
