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
#include "support/fakes.hpp"
#include "support/fake_storage.hpp"
#include "support/fake_valve.hpp"
#include "support/fake_adc.hpp"

#include "control/persistent_state.hpp"
#include "system/state.hpp"
#include "framing/can_header.hpp"
#include "framing/payload_type.hpp"
#include "command/command_type.hpp"
#include "response/response_type.hpp"
#include "command/set_valve_position.hpp"   // ValveCommand
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

/* data[] offset of the valve index in a SetValvePosition frame (data[0..3] = timestamp). */
constexpr std::size_t VALVE_INDEX_OFFSET = sizeof(uint32_t);

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

CanFrame makeValveCmd(EcuValves valve, ValveCommand action, BoardId target = BoardId::Engine)
{
    CanFrame frame = makeCommand(command::CommandType::SetValvePosition,
                                 static_cast<uint8_t>(action), target);
    frame.data[VALVE_INDEX_OFFSET] = static_cast<uint8_t>(valve);
    return frame;
}

CanFrame makePing(uint8_t seq = 0)
{
    CanFrame frame = makeCommand(command::CommandType::Ping, /*senderState=*/0,
                                 BoardId::Engine, seq);
    frame.length = 0;   // a ping carries no payload
    return frame;
}

class EcuControllerTest : public ::testing::Test {
protected:
    FakeStorage      storage_;
    FakeValve        ipa_valve_;
    FakeValve        nos_valve_;
    FakeStreamingAdc adc_;
    FakeCan          can_;
    logic::ecu::Controller<FakeStorage, FakeValve, FakeStreamingAdc, FakeCan>
                     controller_{storage_, ipa_valve_, nos_valve_, adc_, can_};
    uint32_t         now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        logic::control::persistent_state = logic::control::PersistentState{};
        controller_.init();
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

TEST_F(EcuControllerTest, StartsAtInitAndFirstTickEntersSafe)
{
    EXPECT_EQ(current(), State::Init);
    step();
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
}

} // namespace
