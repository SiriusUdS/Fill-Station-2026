/* ------------------------------------------------------------------------- *
 * Unit tests for the FCU filling-station state machine (logic::fcu).
 *
 * The logic is exercised purely through its public surface (init/tick) and the
 * communication interfaces, which the FakeBus stands in for. The state machine
 * keeps its state in battery-backed persistent_state; telemetry is no longer a
 * per-tick heartbeat (the TelemetryId::SystemState SystemState packet is now produced on the
 * record timer, see produceRecord), so the state-machine tests read the current
 * state straight from persistent_state via currentState().
 * ------------------------------------------------------------------------- */

#include "fcu_controller.hpp"
#include "support/fakes.hpp"
#include "support/fake_storage.hpp"
#include "support/fake_valve.hpp"
#include "support/fake_adc.hpp"

#include "communication/interfaces/can.hpp"
#include "communication/interfaces/ethernet.hpp"
#include "communication/protocol/framing/ethernet_header.hpp"   // EthernetHeader (downlink header)
#include "control/persistent_state.hpp"
#include "system/state.hpp"
#include "framing/can_header.hpp"

#include "framing/udp_frame.hpp"
#include "system/state.hpp"
#include "telemetry/telemetry_id.hpp"
#include "system/board_id.hpp"
#include "command/command_type.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

using logic::communication::CanFrame;
using logic::communication::Endpoint;
using logic::communication::command::CommandType;

namespace {

/* Byte offset, within the UDP payload, of the requested-state field of a
   CommandType::SetState command. Mirrors REQUEST_STATE_OFFSET_BYTES in the logic. */
constexpr std::size_t REQUEST_STATE_OFFSET = 15;

/* Build a CommandType::SetState datagram addressed to `device`, asking for `requested`. */
std::vector<uint8_t> makeStateRequest(BoardId device, logic::control::State requested)
{
    std::vector<uint8_t> payload(REQUEST_STATE_OFFSET + 1, 0);
    UDPPacketHeader header = {};
    header.frame.deviceID  = static_cast<uint8_t>(device);
    header.frame.payloadID = static_cast<uint8_t>(CommandType::SetState);
    std::memcpy(payload.data(), header.bytes, sizeof(UDPPacketHeader));
    payload[REQUEST_STATE_OFFSET] = static_cast<uint8_t>(requested);
    return payload;
}

/* Build a CAN frame with the given header fields and payload. */
CanFrame makeCanFrame(BoardId sender, BoardId target, uint8_t messageId,
                      std::array<uint8_t, 8> data = {})
{
    CanHeader header        = {};
    header.frame.senderID   = static_cast<uint8_t>(sender);
    header.frame.targetID   = static_cast<uint8_t>(target);
    header.frame.messageID  = messageId;

    CanFrame frame;
    frame.id     = header.code;
    frame.data   = data;
    frame.length = 8;
    return frame;
}

class FcuControllerTest : public ::testing::Test {
protected:
    FakeStorage      storage_;
    FakeValve        fill_valve_;
    FakeValve        dump_valve_;
    FakeStreamingAdc adc_;
    FakeEthernet     eth_;
    FakeCan          can_;
    logic::fcu::Controller<FakeStorage, FakeValve, FakeStreamingAdc, FakeEthernet, FakeCan>
                     controller_{storage_, fill_valve_, dump_valve_, adc_, eth_, can_};
    uint32_t         now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        /* Start every test from a cold Backup SRAM so persisted state never
           leaks between tests; init() then commits a fresh INIT. */
        logic::control::persistent_state = logic::control::PersistentState{};
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
   on the record timer (produceRecord) and batched into TelemetryId::SystemState datagrams when
   a 4096-byte half fills and drains. Pump records until that happens and check
   the downlinked packet is a well-formed TelemetryId::SystemState frame. */
TEST_F(FcuControllerTest, FullTelemetryBufferDownlinksGetSystem)
{
    reachSafe();  // also clears udp_tx

    const AdcInfo sample{};
    for (int i = 0; i < 1000 && bus().udp_tx.empty(); ++i) {
        adc_.push(sample);                  // one conversion into the ADC ring
        controller_.produceRecord(++now_ms_);  // drain it into the telemetry buffer
        step();                             // drainTick flushes any full half
    }

    ASSERT_FALSE(bus().udp_tx.empty()) << "a full telemetry half never downlinked";
    const auto& payload = bus().udp_tx.back().payload;
    ASSERT_GE(payload.size(), sizeof(EthernetHeader));
    EthernetHeader header;
    std::memcpy(&header, payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(header.deviceID, static_cast<uint8_t>(BoardId::FillingStation));
    EXPECT_EQ(header.payloadID, static_cast<uint8_t>(TelemetryId::SystemState));
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
    bus().push_can(makeCanFrame(BoardId::Engine, BoardId::FillingStation, static_cast<uint8_t>(TelemetryId::SystemState)));
    step();
    EXPECT_TRUE(bus().can_tx.empty());
}

TEST_F(FcuControllerTest, AllQueuedCanFramesAreDrainedInOneTick)
{
    bus().push_can(makeCanFrame(BoardId::Engine, BoardId::FillingStation, static_cast<uint8_t>(TelemetryId::SystemState)));
    bus().push_can(makeCanFrame(BoardId::Engine, BoardId::FillingStation, static_cast<uint8_t>(TelemetryId::SystemState)));
    bus().push_can(makeCanFrame(BoardId::Engine, BoardId::FillingStation, static_cast<uint8_t>(TelemetryId::SystemState)));
    step();
    EXPECT_TRUE(bus().can_rx.empty());  // all drained in one tick
    EXPECT_TRUE(bus().can_tx.empty());  // and never answered
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
