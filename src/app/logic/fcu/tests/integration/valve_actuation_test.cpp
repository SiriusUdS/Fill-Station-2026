/* ------------------------------------------------------------------------- *
 * Integration test: an incoming UDP SetValvePosition command actuates the
 * FCU's local valves.
 *
 * Drives the assembled receive path end to end — a raw UDP datagram pushed onto
 * the FakeBus, through the real Controller (rxTick -> handleDatagram ->
 * handleSetValvePosition), out to the injected valves. The valves are FakeValves
 * so we can assert exactly which one was driven and how (open / close / set %).
 * ------------------------------------------------------------------------- */

#include "fcu_controller.hpp"
#include "support/fakes.hpp"
#include "support/fake_storage.hpp"
#include "support/fake_valve.hpp"
#include "support/fake_adc.hpp"

#include "communication/command/command.hpp"             // CommandType
#include "command/set_valve_position.hpp"  // SetValvePositionFrame, ValveCommand
#include "system/valves/fcu.hpp"                   // FcuValves
#include "control/persistent_state.hpp"

#include "framing/udp_frame.hpp"                 // UDPPacketHeader
#include "system/board_id.hpp"  // BoardId::FillingStation

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

using logic::communication::Endpoint;
namespace command = logic::communication::command;

namespace {

/* Build a UDP datagram carrying a SetValvePosition command: the 12-byte header
   (payloadID = SetValvePosition) followed by the 3-byte frame. */
std::vector<uint8_t> makeValveCommand(FcuValves valve, ValveCommand action, uint8_t value)
{
    UDPPacketHeader header{};
    header.frame.deviceID  = static_cast<uint8_t>(BoardId::FillingStation);
    header.frame.payloadID = static_cast<uint8_t>(command::CommandType::SetValvePosition);

    const SetValvePositionFrame frame{valve, action, value};

    std::vector<uint8_t> datagram(sizeof(UDPPacketHeader) + sizeof(SetValvePositionFrame));
    std::memcpy(datagram.data(), header.bytes, sizeof(UDPPacketHeader));
    std::memcpy(datagram.data() + sizeof(UDPPacketHeader), &frame, sizeof(frame));
    return datagram;
}

class ValveActuation : public ::testing::Test {
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

    /* Deliver a datagram the way the link does: queue it, then tick once so the
       controller drains and handles it. */
    void deliver(const std::vector<uint8_t>& datagram)
    {
        bus().push_udp(Endpoint{}, datagram);
        controller_.tick(++now_ms_);
    }
};

/* ---- Fill valve ---------------------------------------------------------- */

TEST_F(ValveActuation, FillOpen)
{
    deliver(makeValveCommand(FcuValves::Fill, ValveCommand::Open, 0));
    EXPECT_EQ(fill_valve_.open_calls, 1);
    EXPECT_EQ(fill_valve_.close_calls, 0);
    EXPECT_EQ(dump_valve_.open_calls, 0);  // the other valve is untouched
}

TEST_F(ValveActuation, FillClose)
{
    deliver(makeValveCommand(FcuValves::Fill, ValveCommand::Close, 0));
    EXPECT_EQ(fill_valve_.close_calls, 1);
    EXPECT_EQ(fill_valve_.open_calls, 0);
    EXPECT_EQ(dump_valve_.close_calls, 0);
}

TEST_F(ValveActuation, FillSetOpenedPct)
{
    deliver(makeValveCommand(FcuValves::Fill, ValveCommand::SetOpenedPct, 42));
    EXPECT_EQ(fill_valve_.percent_calls, 1);
    EXPECT_FLOAT_EQ(fill_valve_.last_percent, 42.0F);
    EXPECT_EQ(dump_valve_.percent_calls, 0);
}

/* ---- Dump valve ---------------------------------------------------------- */

TEST_F(ValveActuation, DumpOpen)
{
    deliver(makeValveCommand(FcuValves::Dump, ValveCommand::Open, 0));
    EXPECT_EQ(dump_valve_.open_calls, 1);
    EXPECT_EQ(fill_valve_.open_calls, 0);
}

TEST_F(ValveActuation, DumpClose)
{
    deliver(makeValveCommand(FcuValves::Dump, ValveCommand::Close, 0));
    EXPECT_EQ(dump_valve_.close_calls, 1);
    EXPECT_EQ(fill_valve_.close_calls, 0);
}

TEST_F(ValveActuation, DumpSetOpenedPct)
{
    deliver(makeValveCommand(FcuValves::Dump, ValveCommand::SetOpenedPct, 73));
    EXPECT_EQ(dump_valve_.percent_calls, 1);
    EXPECT_FLOAT_EQ(dump_valve_.last_percent, 73.0F);
    EXPECT_EQ(fill_valve_.percent_calls, 0);
}

} // namespace
