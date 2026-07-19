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
#include "support/fakes.hpp"   // the standard set of host test doubles
#include "support/datagram_crc.hpp"   // appendGsCrc (inbound datagrams carry a CRC)

#include "communication/command/command.hpp"             // CommandType
#include "command/set_valve_position.hpp"  // SetValvePositionFrame, ValveCommand
#include "system/valves/fcu.hpp"                   // FcuValves
#include "control/persistent_state.hpp"

#include "framing/ethernet_header.hpp"           // EthernetHeader
#include "framing/payload_type.hpp"              // PayloadType
#include "system/board_id.hpp"  // BoardId::FillingStation

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

using logic::communication::Endpoint;
namespace command = logic::communication::command;

namespace {

/* Build a UDP datagram carrying a SetValvePosition command: the 12-byte header
   (payload_id = SetValvePosition) followed by the 3-byte frame. */
std::vector<uint8_t> makeValveCommand(FcuValves valve, ValveCommand action, uint8_t value)
{
    EthernetHeader header{};
    header.target_id    = static_cast<uint8_t>(BoardId::FillingStation);
    header.payload_type = static_cast<uint8_t>(PayloadType::Command);
    header.payload_id   = static_cast<uint8_t>(command::CommandType::SetValvePosition);

    const SetValvePositionFrame frame{valve, action, value};

    std::vector<uint8_t> datagram(sizeof(EthernetHeader) + sizeof(SetValvePositionFrame));
    std::memcpy(datagram.data(), &header, sizeof(EthernetHeader));
    std::memcpy(datagram.data() + sizeof(EthernetHeader), &frame, sizeof(frame));
    appendGsCrc(datagram);
    return datagram;
}

class ValveActuation : public ::testing::Test {
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
    FakeHeater           heater_tank_;
    logic::fcu::Controller<FakeStorage, FakeValve, FakeStreamingAdc, FakeEthernet, FakeCan,
                           FakeThermocoupleBank, FakePowerMonitor, FakeEmatch, FakeSolenoid, FakeHeater>
                     controller_{storage_fast_, storage_slow_, storage_ext_,
                                 fill_valve_, dump_valve_, adc_, eth_, can_, tc_, pm_, ematch_, solenoid_, heater_, heater_tank_};
    uint32_t         now_ms_ = 0;

    void SetUp() override
    {
        bus().reset();
        logic::control::persistent_state = logic::control::PersistentState{};
        controller_.init();

        // Operator per-valve actuation is gated to the Unsafe state; arm into Unsafe so these
        // valve-command tests exercise actuation rather than the gate (covered in control_test).
        logic::control::persistent_state.saveState(logic::control::State::Unsafe);

        // init() safe-boots the valves closed through the control layer; discard those
        // call counts so each test asserts only its own command-driven actuation.
        fill_valve_.open_calls = fill_valve_.close_calls = fill_valve_.percent_calls = 0;
        dump_valve_.open_calls = dump_valve_.close_calls = dump_valve_.percent_calls = 0;
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
