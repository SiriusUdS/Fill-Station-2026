/* ------------------------------------------------------------------------- *
 * Integration test: the FCU routes a ping through to the ECU and relays the
 * ECU's pong back to the ground station.
 *
 * Drives the assembled FCU Controller end to end over the FakeBus:
 *   - Ping (Gs->Fcu->Ecu): a UDP Ping from the GS (rxTick -> handleDatagram ->
 *     handlePing) is forwarded to the ECU as a CAN Command/Ping addressed to
 *     Engine; and
 *   - Pong (Ecu->Fcu->Gs): a CAN Response/Pong from the ECU (canTick ->
 *     relayPongToGs) is relayed to the GS as a UDP Response/Pong tagged as the
 *     ECU's (sender BoardId::Engine).
 * No fakes are stubbed per-unit: raw bytes in on one transport, raw bytes out on
 * the other, inspected through the FakeBus tx queues.
 * ------------------------------------------------------------------------- */

#include "fcu_controller.hpp"
#include "support/fakes.hpp"
#include "support/fake_storage.hpp"
#include "support/fake_valve.hpp"
#include "support/fake_adc.hpp"

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

/* A UDP datagram carrying a no-payload Ping command addressed to the FCU, stamped
   with the GS's sequence @p seq. */
std::vector<uint8_t> makePingCommand(uint8_t seq = 0)
{
    EthernetHeader header{};
    header.sender_id    = static_cast<uint8_t>(BoardId::GsControl);
    header.target_id    = static_cast<uint8_t>(BoardId::FillingStation);
    header.payload_type = static_cast<uint8_t>(PayloadType::Command);
    header.payload_id   = static_cast<uint8_t>(command::CommandType::Ping);
    header.seq          = seq;

    std::vector<uint8_t> datagram(sizeof(EthernetHeader));
    std::memcpy(datagram.data(), &header, sizeof(EthernetHeader));
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

/* ---- Ping (Gs->Fcu->Ecu) ------------------------------------------------- */

TEST_F(PingRouting, GsPingIsForwardedToEcuOverCan)
{
    deliverUdp(makePingCommand());

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
    deliverUdp(makePingCommand(/*seq=*/5));    // GS pings with seq 5

    ASSERT_EQ(bus().can_tx.size(), 1u);
    CanHeader forwarded;
    forwarded.code = bus().can_tx.front().id;
    EXPECT_EQ(forwarded.frame.seq, 5u);        // forwarded to the ECU carrying seq 5

    deliverCan(makePongFrame(/*seq=*/5));      // ECU answers, echoing seq 5

    ASSERT_EQ(bus().udp_tx.size(), 1u);
    EthernetHeader relayed;
    std::memcpy(&relayed, bus().udp_tx.front().payload.data(), sizeof(EthernetHeader));
    EXPECT_EQ(relayed.seq, 5u);                // relayed back to the GS still carrying seq 5
}

} // namespace
