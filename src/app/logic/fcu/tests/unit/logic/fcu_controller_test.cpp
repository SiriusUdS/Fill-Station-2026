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
#include "communication/system_state_codec.hpp"   // packSystemState (ECU telemetry fragments)
#include "communication/protocol/telemetry/ecu_system_state.hpp"   // EcuSystemState
#include "communication/protocol/telemetry/fcu_system_state.hpp"   // FcuSystemState
#include "communication/protocol/telemetry/fcu_extended_system_state.hpp"   // FcuExtendedSystemState (thermocouples)
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
    frame.data   = data;
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
    logic::fcu::Controller<FakeStorage, FakeValve, FakeStreamingAdc, FakeEthernet, FakeCan,
                           FakeThermocoupleBank>
                     controller_{storage_fast_, storage_slow_, storage_ext_,
                                 fill_valve_, dump_valve_, adc_, eth_, can_, tc_};
    uint32_t         now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        /* Start every test from a cold Backup SRAM so persisted state never
           leaks between tests; init() then commits a fresh INIT. */
        logic::control::persistent_state = logic::control::PersistentState{};
        logic::control::control_flags    = logic::control::ControlFlags{};  // all flags off
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

    /* Drive the machine into SAFE and clear recorded traffic. */
    void reachSafe()
    {
        step();  // first tick moves INIT -> SAFE
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
};

/* ---- Startup ------------------------------------------------------------- */

TEST_F(FcuControllerTest, StartsAtInitAndFirstTickEntersSafe)
{
    EXPECT_EQ(currentState(), logic::control::State::Init);  // right after init(), before any tick
    step();
    EXPECT_EQ(currentState(), logic::control::State::Safe);  // first tick moves INIT -> SAFE
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

/* The ExtendedSystemState carries the live control-flag bitmask, so the GS reads the
   recording config straight from telemetry (bit N = ControlFlag value N). */
TEST_F(FcuControllerTest, ExtendedRecordReportsTheLiveControlFlags)
{
    reachSafe();  // clears udp_tx
    logic::control::control_flags.set(ControlFlag::FastRecording, true);  // PersistingData stays off

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
        EXPECT_NE(ext.control_flags & (1u << static_cast<uint8_t>(ControlFlag::FastRecording)), 0u);
        EXPECT_EQ(ext.control_flags & (1u << static_cast<uint8_t>(ControlFlag::PersistingData)), 0u);
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

TEST_F(FcuControllerTest, SafeToTest)
{
    reachSafe();
    requestState(logic::control::State::Test);
    EXPECT_EQ(currentState(), logic::control::State::Test);
}

TEST_F(FcuControllerTest, SafeToUnsafe)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    EXPECT_EQ(currentState(), logic::control::State::Unsafe);
}

TEST_F(FcuControllerTest, TestBackToSafe)
{
    reachSafe();
    requestState(logic::control::State::Test);
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
    requestState(logic::control::State::Abort);
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

/* ---- Receive watchdog ---------------------------------------------------- */

TEST_F(FcuControllerTest, WatchdogAbortsUnsafeAfterSilence)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);  // last_rx updated at this tick
    ASSERT_EQ(currentState(), logic::control::State::Unsafe);

    stepTo(now_ms_ + 500);  // 500 ms with no inbound datagram
    EXPECT_EQ(currentState(), logic::control::State::Abort);
}

TEST_F(FcuControllerTest, WatchdogDoesNotAbortSafe)
{
    reachSafe();
    stepTo(now_ms_ + 5000);  // long silence, but SAFE has no watchdog
    EXPECT_EQ(currentState(), logic::control::State::Safe);
}

TEST_F(FcuControllerTest, TrafficKeepsUnsafeAlive)
{
    reachSafe();
    requestState(logic::control::State::Unsafe);
    /* Keep feeding (irrelevant) datagrams so last_rx stays fresh. */
    for (int i = 0; i < 600; ++i) {
        requestState(logic::control::State::Unsafe);  // self-transition keeps rx alive
    }
    EXPECT_EQ(currentState(), logic::control::State::Unsafe);
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

/* A full EcuSystemState, fragmented over CAN exactly as the ECU sends it, must be
   reassembled and relayed to the GS tagged as the ECU's (sender BoardId::Engine).
   This drives the real codec end to end, so it pins the relay path the GS depends on. */
TEST_F(FcuControllerTest, EcuTelemetryIsReassembledAndRelayedToGs)
{
    namespace codec = logic::communication::can;
    reachSafe();  // clears udp_tx / can_tx

    EcuSystemState record{};
    record.base.creation_timestamp_ms = 0xABCDEF01;  // a marker to find on the GS side

    std::array<CanFrame, codec::SYSTEM_STATE_FRAGMENTS> frames;
    codec::packSystemState(record, BoardId::Engine, BoardId::FillingStation, /*seq=*/0,
                           std::span<CanFrame, codec::SYSTEM_STATE_FRAGMENTS>(frames));
    for (const auto& f : frames) {
        bus().push_can(f);
    }

    step();  // controller drains CAN, reassembles, relays to the GS

    ASSERT_FALSE(bus().udp_tx.empty()) << "reassembled ECU record was not relayed to the GS";
    const auto& payload = bus().udp_tx.back().payload;
    ASSERT_GE(payload.size(), sizeof(EthernetHeader) + sizeof(EcuSystemState));
    EthernetHeader header;
    std::memcpy(&header, payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(header.sender_id, static_cast<uint8_t>(BoardId::Engine));  // tagged as the ECU's
    EXPECT_EQ(header.payload_id, static_cast<uint8_t>(TelemetryType::SystemState));

    EcuSystemState relayed{};
    std::memcpy(&relayed, payload.data() + sizeof(EthernetHeader), sizeof(EcuSystemState));
    EXPECT_EQ(relayed.base.creation_timestamp_ms, 0xABCDEF01u);  // bytes round-tripped intact
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

TEST_F(FcuControllerTest, ColdBootWithInvalidBlobStartsAtInit)
{
    logic::control::persistent_state.saveState(logic::control::State::Ignite);
    logic::control::persistent_state.magic = 0;  // corrupt => looks like cold garbage

    controller_.init();  // cold/corrupt boot starts at INIT (before any tick advances it)
    EXPECT_EQ(currentState(), logic::control::State::Init);
}

} // namespace
