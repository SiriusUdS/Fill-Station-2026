/* ------------------------------------------------------------------------- *
 * Integration test: an incoming Ethernet frame drives a state change.
 *
 * Exercises the assembled command pipeline end to end — the real Ethernet parser
 * (fromEthernet, stm-2026-common) feeds the real main command handler
 * (handleCommand -> handleSetState, CM7), which commits to the real persistent
 * state. No fakes and no per-unit stubbing: raw frame bytes in, persisted state
 * out. This is the path fcu_controller will use once it is wired to the command
 * pipeline; here we drive it directly.
 * ------------------------------------------------------------------------- */

#include <gtest/gtest.h>

#include "command/command.hpp"
#include "command/parser/command_ethernet_parser.hpp"
#include "control/command_handlers/command_handlers.hpp"   // handleCommand
#include "control/persistent_state.hpp"
#include "control/states.hpp"
#include "ethernet/ethernet_header.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace cmd = logic::communication::command;
namespace ch  = logic::control::command_handlers;
using logic::control::State;

namespace {

/* Build a UDP/Ethernet frame: a 12-byte EthernetHeader followed by `payload`. */
std::vector<uint8_t> makeEthFrame(uint8_t deviceId, uint8_t payloadId, uint32_t ts,
                                  const uint8_t* payload, std::size_t payloadLen)
{
    EthernetHeader header{};
    header.deviceID      = deviceId;
    header.payloadID     = payloadId;
    header.payloadLenght = static_cast<uint16_t>(payloadLen);
    header.deviceTS_MS   = ts;

    std::vector<uint8_t> buf(sizeof(EthernetHeader) + payloadLen);
    std::memcpy(buf.data(), &header, sizeof(EthernetHeader));
    if (payloadLen != 0) {
        std::memcpy(buf.data() + sizeof(EthernetHeader), payload, payloadLen);
    }
    return buf;
}

/* A SetState frame (payloadID = SetState) requesting transition to `requested`. */
std::vector<uint8_t> makeSetStateFrame(State requested)
{
    SetStateFrame body{};
    body.flags       = 0;
    body.requestedID = static_cast<uint8_t>(requested);
    return makeEthFrame(/*deviceId=*/0x03,
                        static_cast<uint8_t>(cmd::CommandType::SetState),
                        /*ts=*/1234,
                        reinterpret_cast<const uint8_t*>(&body), sizeof(body));
}

class EthernetStateChange : public ::testing::Test {
protected:
    /* Cold Backup SRAM each test, then seed a known current state. */
    void SetUp() override
    {
        logic::control::persistent_state = logic::control::PersistentState{};
    }
    void setCurrent(State s) { logic::control::persistent_state.saveState(s); }
    State current() const { return logic::control::persistent_state.fill_state; }

    /* Deliver a raw frame the way the controller will: parse, then handle. */
    bool deliver(const std::vector<uint8_t>& frame)
    {
        const auto command = cmd::fromEthernet(frame);
        if (!command) {
            return false;
        }
        return ch::handleCommand(*command);
    }
};

TEST_F(EthernetStateChange, SafeToUnsafeFromEthernetFrame)
{
    setCurrent(State::Safe);
    EXPECT_TRUE(deliver(makeSetStateFrame(State::Unsafe)));
    EXPECT_EQ(current(), State::Unsafe);
}

TEST_F(EthernetStateChange, IllegalTransitionLeavesStateUnchanged)
{
    setCurrent(State::Safe);
    EXPECT_FALSE(deliver(makeSetStateFrame(State::Ignite)));  // Safe -> Ignite not allowed
    EXPECT_EQ(current(), State::Safe);
}

TEST_F(EthernetStateChange, MalformedFrameIsRejected)
{
    setCurrent(State::Safe);
    const std::vector<uint8_t> tooShort(4, 0);  // shorter than EthernetHeader
    EXPECT_FALSE(deliver(tooShort));
    EXPECT_EQ(current(), State::Safe);
}

} // namespace
