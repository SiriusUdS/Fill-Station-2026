/* ------------------------------------------------------------------------- *
 * Integration test: the broadcast Ping heartbeat. The GS always BROADCASTS the ~1 Hz Ping, so one
 * packet makes the FCU answer for itself AND bridge the ping to the ECU; the ECU's pong is relayed
 * back to the ground station.
 *
 * Drives the assembled FCU Controller end to end over the FakeBus:
 *   - Heartbeat (Gs -> {Fcu, Ecu}): a broadcast UDP Ping makes the FCU (a) Pong straight to the GS
 *     (sender FillingStation) and (b) forward a CAN Command/Ping to the Engine; and
 *   - Pong (Ecu->Fcu->Gs): a CAN Response/Pong from the ECU is relayed to the GS as a UDP
 *     Response/Pong tagged as the ECU's (sender BoardId::Engine).
 * No fakes are stubbed per-unit: raw bytes in on one transport, raw bytes out on
 * the other, inspected through the FakeBus tx queues.
 * ------------------------------------------------------------------------- */

#include "fcu_controller.hpp"
#include "support/fakes.hpp"   // the standard set of host test doubles
#include "support/datagram_crc.hpp"   // appendGsCrc (inbound datagrams carry a CRC)

#include "communication/command/command.hpp"     // CommandType
#include "control/persistent_state.hpp"

#include "framing/can_header.hpp"                 // CanHeader
#include "framing/ethernet_header.hpp"            // EthernetHeader
#include "framing/payload_type.hpp"               // PayloadType
#include "response/response_type.hpp"             // ResponseType::Pong
#include "system/board_id.hpp"                    // BoardId

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

using logic::communication::Endpoint;
namespace command = logic::communication::command;

namespace {

/* A UDP datagram carrying a no-payload Ping command, broadcast by the GS (the heartbeat is always a
   Broadcast, so both boards answer), stamped with the GS's sequence @p seq. @p target overridable for
   the unicast-routing cases. */
std::vector<uint8_t> makePingCommand(uint8_t seq = 0, BoardId target = BoardId::Broadcast)
{
    EthernetHeader header{};
    header.sender_id    = static_cast<uint8_t>(BoardId::GsControl);
    header.target_id    = static_cast<uint8_t>(target);
    header.payload_type = static_cast<uint8_t>(PayloadType::Command);
    header.payload_id   = static_cast<uint8_t>(command::CommandType::Ping);
    header.seq          = seq;

    std::vector<uint8_t> datagram(sizeof(EthernetHeader));
    std::memcpy(datagram.data(), &header, sizeof(EthernetHeader));
    appendGsCrc(datagram);
    return datagram;
}

/* A CAN frame carrying a no-payload Pong from the ECU addressed to the FCU, echoing
   the command sequence @p seq. */
logic::communication::CanFrame makePongFrame(uint8_t seq = 0)
{
    CanHeader header{};
    header.frame.sender_id    = static_cast<uint8_t>(BoardId::Engine);
    header.frame.target_id    = static_cast<uint8_t>(BoardId::FillingStation);
    header.frame.payload_type = static_cast<uint8_t>(PayloadType::Response);
    header.frame.payload_id   = static_cast<uint8_t>(ResponseType::Pong);
    header.frame.seq          = seq;

    logic::communication::CanFrame frame;
    frame.id     = header.code;
    frame.length = 0;
    return frame;
}

class PingRouting : public ::testing::Test {
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
    FakeHeater           heater_;
    logic::fcu::Controller<FakeStorage, FakeValve, FakeStreamingAdc, FakeEthernet, FakeCan,
                           FakeThermocoupleBank, FakePowerMonitor, FakeEmatch, FakeSolenoid, FakeHeater>
                     controller_{storage_fast_, storage_slow_, storage_ext_,
                                 fill_valve_, dump_valve_, adc_, eth_, can_, tc_, pm_, ematch_, solenoid_, heater_};
    uint32_t         now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        logic::control::persistent_state = logic::control::PersistentState{};
        controller_.init();
    }

    void deliverUdp(const std::vector<uint8_t>& datagram)
    {
        bus().push_udp(Endpoint{}, datagram);
        controller_.tick(++now_ms_);
    }

    void deliverCan(const logic::communication::CanFrame& frame)
    {
        bus().push_can(frame);
        controller_.tick(++now_ms_);
    }
};

/* ---- Broadcast heartbeat (Gs -> {Fcu pong, Ecu forward}) ----------------- */

TEST_F(PingRouting, GsBroadcastPingPongsFromFcuAndForwardsToEcu)
{
    deliverUdp(makePingCommand(/*seq=*/3));   // broadcast heartbeat

    // (a) the FCU answers for itself, a Pong straight to the GS.
    ASSERT_EQ(bus().udp_tx.size(), 1u);
    EthernetHeader pong;
    std::memcpy(&pong, bus().udp_tx.front().payload.data(), sizeof(pong));
    EXPECT_EQ(static_cast<BoardId>(pong.sender_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(pong.payload_type), PayloadType::Response);
    EXPECT_EQ(static_cast<ResponseType>(pong.payload_id), ResponseType::Pong);
    EXPECT_EQ(pong.seq, 3u);

    // (b) and bridges the ping to the ECU over CAN (so the ECU can pong too).
    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader header;
    header.code = bus().can_tx.front().id;
    EXPECT_EQ(static_cast<BoardId>(header.frame.target_id), BoardId::Engine);
    EXPECT_EQ(static_cast<BoardId>(header.frame.sender_id), BoardId::FillingStation);
    EXPECT_EQ(static_cast<PayloadType>(header.frame.payload_type), PayloadType::Command);
    EXPECT_EQ(static_cast<command::CommandType>(header.frame.payload_id), command::CommandType::Ping);
    EXPECT_EQ(bus().can_tx.front().length, 0);
}

/* ---- Pong (Ecu->Fcu->Gs) ------------------------------------------------- */

TEST_F(PingRouting, EcuPongIsRelayedToGsOverEthernet)
{
    deliverCan(makePongFrame());

    ASSERT_EQ(bus().udp_tx.size(), 1u);
    const auto& datagram = bus().udp_tx.front().payload;
    ASSERT_GE(datagram.size(), sizeof(EthernetHeader));
    EthernetHeader header;
    std::memcpy(&header, datagram.data(), sizeof(EthernetHeader));
    EXPECT_EQ(static_cast<BoardId>(header.sender_id), BoardId::Engine);  // the pong is the ECU's
    EXPECT_EQ(static_cast<BoardId>(header.target_id), BoardId::GsControl);
    EXPECT_EQ(static_cast<PayloadType>(header.payload_type), PayloadType::Response);
    EXPECT_EQ(static_cast<ResponseType>(header.payload_id), ResponseType::Pong);
}

/* ECU telemetry frames must still reach the reassembler, not the pong relay. */
TEST_F(PingRouting, NonPongCanFrameDoesNotRelayToGs)
{
    logic::communication::CanFrame telemetry{};  // payload_type 0 (unset) — not a Pong
    deliverCan(telemetry);
    EXPECT_TRUE(bus().udp_tx.empty());
}

/* ---- End-to-end seq propagation (Gs -> Fcu -> Ecu -> Fcu -> Gs) ----------- */

TEST_F(PingRouting, GsSeqIsPropagatedThroughTheWholeChain)
{
    deliverUdp(makePingCommand(/*seq=*/5));    // GS broadcasts the heartbeat with seq 5

    // The FCU's own Pong echoes seq 5...
    ASSERT_EQ(bus().udp_tx.size(), 1u);
    EthernetHeader self_pong;
    std::memcpy(&self_pong, bus().udp_tx.front().payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(static_cast<BoardId>(self_pong.sender_id), BoardId::FillingStation);
    EXPECT_EQ(self_pong.seq, 5u);

    // ...and the ping is forwarded to the ECU carrying seq 5.
    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader forwarded;
    forwarded.code = bus().can_tx.front().id;
    EXPECT_EQ(forwarded.frame.seq, 5u);

    bus().udp_tx.clear();   // isolate the relayed ECU pong from the FCU's own pong above
    deliverCan(makePongFrame(/*seq=*/5));      // ECU answers, echoing seq 5

    ASSERT_EQ(bus().udp_tx.size(), 1u);
    EthernetHeader relayed;
    std::memcpy(&relayed, bus().udp_tx.front().payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(static_cast<BoardId>(relayed.sender_id), BoardId::Engine);  // the ECU's pong
    EXPECT_EQ(relayed.seq, 5u);                // relayed back to the GS still carrying seq 5
}

} // namespace
