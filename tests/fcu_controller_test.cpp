/* ------------------------------------------------------------------------- *
 * Unit tests for the FCU filling-station state machine (logic::fcu).
 *
 * The logic is exercised purely through its public surface (init/tick) and the
 * communication interfaces, which the FakeBus stands in for. The state machine
 * keeps its state in an anonymous namespace with no getter, but every state is
 * observable: the heartbeat emitted each tick carries the current state in its
 * UDPPacketHeader.frame.deviceState field, so we read state back off the bus.
 * ------------------------------------------------------------------------- */

#include "fcu_controller.hpp"
#include "support/fakes.hpp"

#include "communication/interfaces/can.hpp"
#include "communication/interfaces/ethernet.hpp"
#include "dil/can_types.h"

#include "sirius-headers-common/Ethernet/UDPFrame.h"
#include "sirius-headers-common/FillingStation/FillingStationState.h"
#include "sirius-headers-common/Telecommunication/BoardCommandV2.h"
#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

using logic::communication::CanFrame;
using logic::communication::Endpoint;

namespace {

/* Byte offset, within the UDP payload, of the requested-state field of a
   REQUEST_STATE command. Mirrors REQUEST_STATE_OFFSET_BYTES in the logic. */
constexpr std::size_t REQUEST_STATE_OFFSET = 15;

/* Build a REQUEST_STATE datagram addressed to `device`, asking for `requested`. */
std::vector<uint8_t> makeStateRequest(uint8_t device, uint8_t requested)
{
    std::vector<uint8_t> payload(REQUEST_STATE_OFFSET + 1, 0);
    UDPPacketHeader header = {};
    header.frame.deviceID  = device;
    header.frame.payloadID = REQUEST_STATE;
    std::memcpy(payload.data(), header.bytes, sizeof(UDPPacketHeader));
    payload[REQUEST_STATE_OFFSET] = requested;
    return payload;
}

/* Build a CAN frame with the given header fields and payload. */
CanFrame makeCanFrame(uint8_t sender, uint8_t target, uint8_t messageId,
                      std::array<uint8_t, 8> data = {})
{
    CANHeader header        = {};
    header.frame.senderID   = sender;
    header.frame.targetID   = target;
    header.frame.messageID  = messageId;

    CanFrame frame;
    frame.id     = header.code;
    frame.data   = data;
    frame.length = 8;
    return frame;
}

class FcuControllerTest : public ::testing::Test {
protected:
    uint32_t now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        logic::fcu::init();
    }

    /* Advance the logic one tick with a strictly increasing timestamp. */
    void step() { logic::fcu::tick(++now_ms_); }

    /* Advance to an absolute timestamp (must be > the current one). */
    void stepTo(uint32_t now) { logic::fcu::tick(now_ms_ = now); }

    /* The state carried by the most recent heartbeat (only UDP traffic sent). */
    uint8_t lastHeartbeatState() const
    {
        EXPECT_FALSE(bus().udp_tx.empty()) << "no heartbeat was sent";
        const auto& payload = bus().udp_tx.back().payload;
        EXPECT_GE(payload.size(), sizeof(UDPPacketHeader));
        UDPPacketHeader header;
        std::memcpy(header.bytes, payload.data(), sizeof(UDPPacketHeader));
        return header.frame.deviceState;
    }

    /* Drive the machine into SAFE and clear recorded traffic. */
    void reachSafe()
    {
        step();  // heartbeat (INIT), then INIT -> SAFE
        step();  // heartbeat now reports SAFE
        ASSERT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_SAFE);
        bus().udp_tx.clear();
        bus().can_tx.clear();
    }

    /* Send a REQUEST_STATE command (addressed to us) and tick once. */
    void requestState(uint8_t requested, uint8_t device = FILLING_STATION_BOARD_ID)
    {
        const auto payload = makeStateRequest(device, requested);
        bus().push_udp(Endpoint{}, payload);
        step();
    }
};

/* ---- Startup ------------------------------------------------------------- */

TEST_F(FcuControllerTest, FirstTickEmitsInitThenEntersSafe)
{
    step();
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_INIT);
    step();
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_SAFE);
}

TEST_F(FcuControllerTest, HeartbeatHeaderIsWellFormed)
{
    step();
    ASSERT_FALSE(bus().udp_tx.empty());
    const auto& payload = bus().udp_tx.back().payload;
    UDPPacketHeader header;
    std::memcpy(header.bytes, payload.data(), sizeof(UDPPacketHeader));
    EXPECT_EQ(header.frame.deviceID, FILLING_STATION_BOARD_ID);
    EXPECT_EQ(header.frame.payloadID, GET_SYSTEM);
    /* 12-byte header + 4-byte payload + 4-byte CRC. */
    EXPECT_EQ(payload.size(), sizeof(UDPPacketHeader) + 8);
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
    requestState(FILLING_STATION_STATE_TEST);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_TEST);
}

TEST_F(FcuControllerTest, SafeToUnsafe)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_UNSAFE);
}

TEST_F(FcuControllerTest, TestBackToSafe)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_TEST);
    requestState(FILLING_STATION_STATE_SAFE);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_SAFE);
}

TEST_F(FcuControllerTest, UnsafeToIgnite)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE);
    requestState(FILLING_STATION_STATE_IGNITE);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_IGNITE);
}

TEST_F(FcuControllerTest, UnsafeToAbort)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE);
    requestState(FILLING_STATION_STATE_ABORT);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_ABORT);
}

TEST_F(FcuControllerTest, IgniteToAbort)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE);
    requestState(FILLING_STATION_STATE_IGNITE);
    requestState(FILLING_STATION_STATE_ABORT);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_ABORT);
}

TEST_F(FcuControllerTest, AbortBackToSafe)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE);
    requestState(FILLING_STATION_STATE_ABORT);
    requestState(FILLING_STATION_STATE_SAFE);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_SAFE);
}

/* ---- Rejected state transitions ----------------------------------------- */

TEST_F(FcuControllerTest, SafeRejectsIgnite)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_IGNITE);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_SAFE);
}

TEST_F(FcuControllerTest, SafeRejectsAbort)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_ABORT);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_SAFE);
}

TEST_F(FcuControllerTest, UnsafeRejectsTest)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE);
    requestState(FILLING_STATION_STATE_TEST);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_UNSAFE);
}

TEST_F(FcuControllerTest, CommandForAnotherBoardIsIgnored)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE, ENGINE_BOARD_ID);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_SAFE);
}

TEST_F(FcuControllerTest, BroadcastCommandIsAccepted)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE, BOARD_BROADCAST_ID);
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_UNSAFE);
}

/* ---- Receive watchdog ---------------------------------------------------- */

TEST_F(FcuControllerTest, WatchdogAbortsUnsafeAfterSilence)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE);  // last_rx updated at this tick
    ASSERT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_UNSAFE);

    stepTo(now_ms_ + 500);  // 500 ms with no inbound datagram
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_ABORT);
}

TEST_F(FcuControllerTest, WatchdogDoesNotAbortSafe)
{
    reachSafe();
    stepTo(now_ms_ + 5000);  // long silence, but SAFE has no watchdog
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_SAFE);
}

TEST_F(FcuControllerTest, TrafficKeepsUnsafeAlive)
{
    reachSafe();
    requestState(FILLING_STATION_STATE_UNSAFE);
    /* Keep feeding (irrelevant) datagrams so last_rx stays fresh. */
    for (int i = 0; i < 600; ++i) {
        requestState(FILLING_STATION_STATE_UNSAFE);  // self-transition keeps rx alive
    }
    EXPECT_EQ(lastHeartbeatState(), FILLING_STATION_STATE_UNSAFE);
}

/* ---- CAN ----------------------------------------------------------------- */

TEST_F(FcuControllerTest, PingIsEchoedAsPong)
{
    const std::array<uint8_t, 8> payload = {1, 2, 3, 4, 5, 6, 7, 8};
    bus().push_can(makeCanFrame(ENGINE_BOARD_ID, FILLING_STATION_BOARD_ID, CAN_ID_COMM_PING, payload));
    step();

    ASSERT_EQ(bus().can_tx.size(), 1u);
    const CanFrame& pong = bus().can_tx.front();

    CANHeader header;
    header.code = pong.id;
    EXPECT_EQ(header.frame.messageID, CAN_ID_COMM_PONG);
    EXPECT_EQ(header.frame.senderID, FILLING_STATION_BOARD_ID);
    EXPECT_EQ(header.frame.targetID, ENGINE_BOARD_ID);
    EXPECT_EQ(pong.data, payload);  // payload echoed verbatim
}

TEST_F(FcuControllerTest, ValveStatusDoesNotProduceCanReply)
{
    bus().push_can(makeCanFrame(ENGINE_BOARD_ID, FILLING_STATION_BOARD_ID, CAN_ID_STATUS_VALVE));
    step();
    EXPECT_TRUE(bus().can_tx.empty());
}

TEST_F(FcuControllerTest, AllQueuedCanFramesAreDrainedInOneTick)
{
    bus().push_can(makeCanFrame(ENGINE_BOARD_ID, FILLING_STATION_BOARD_ID, CAN_ID_COMM_PING));
    bus().push_can(makeCanFrame(ENGINE_BOARD_ID, FILLING_STATION_BOARD_ID, CAN_ID_COMM_PING));
    bus().push_can(makeCanFrame(ENGINE_BOARD_ID, FILLING_STATION_BOARD_ID, CAN_ID_COMM_PING));
    step();
    EXPECT_EQ(bus().can_tx.size(), 3u);
    EXPECT_TRUE(bus().can_rx.empty());
}

} // namespace
