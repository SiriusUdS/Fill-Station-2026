/* Unit tests for the transport-agnostic Command parsers (fromCan / fromEthernet).
 *
 * These are pure byte transforms, so no FakeBus is needed — we build raw CAN
 * frames / UDP datagrams and assert on the parsed Command. The key invariant
 * under test is the single source of truth: the SAME CommandType value is the
 * on-wire id for both transports. */

#include <gtest/gtest.h>

#include "command/command.hpp"
#include "command/parser/command_can_parser.hpp"
#include "command/parser/command_ethernet_parser.hpp"
#include "can/can_header.hpp"
#include "ethernet/ethernet_header.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace cmd = logic::communication::command;
using cmd::Command;
using cmd::CommandType;
using logic::communication::CanFrame;

namespace {

constexpr uint8_t wireId(CommandType t) { return static_cast<uint8_t>(t); }

CanFrame makeCanFrame(uint8_t sender, uint8_t target, uint8_t messageId,
                      std::span<const uint8_t> data = {})
{
    CanHeader header{};
    header.senderID  = sender;
    header.targetID  = target;
    header.messageID = messageId;

    CanFrame frame{};
    frame.id     = std::bit_cast<uint32_t>(header);
    frame.length = static_cast<uint8_t>(data.size());
    std::memcpy(frame.data.data(), data.data(),
                data.size() < frame.data.size() ? data.size() : frame.data.size());
    return frame;
}

std::vector<uint8_t> makeEthFrame(uint8_t deviceId, uint8_t payloadId, uint32_t ts,
                                  std::span<const uint8_t> payload = {})
{
    EthernetHeader header{};
    header.deviceID      = deviceId;
    header.payloadID     = payloadId;
    header.payloadLenght = static_cast<uint16_t>(payload.size());
    header.deviceTS_MS   = ts;

    std::vector<uint8_t> buf(sizeof(EthernetHeader) + payload.size());
    std::memcpy(buf.data(), &header, sizeof(EthernetHeader));
    std::memcpy(buf.data() + sizeof(EthernetHeader), payload.data(), payload.size());
    return buf;
}

template <typename T>
std::array<uint8_t, sizeof(T)> asBytes(const T& v)
{
    std::array<uint8_t, sizeof(T)> b{};
    std::memcpy(b.data(), &v, sizeof(T));
    return b;
}

} // namespace

/* ---- fromCan ------------------------------------------------------------- */

TEST(CommandFromCan, SetValvePositionParsed)
{
    SetValvePositionFrame servo{};
    servo.valve  = FcuValves::Fill;
    servo.action = ValveCommand::Open;
    servo.value  = 42;

    const auto bytes = asBytes(servo);
    const CanFrame f = makeCanFrame(0x01, 0x02, wireId(CommandType::SetValvePosition), bytes);

    const auto c = cmd::fromCan(f);
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->type, CommandType::SetValvePosition);
    EXPECT_EQ(c->source, 0x01);
    EXPECT_EQ(c->target, 0x02);

    const auto* out = reinterpret_cast<const SetValvePositionFrame*>(c->payload.data());
    EXPECT_EQ(out->valve, FcuValves::Fill);
    EXPECT_EQ(out->action, ValveCommand::Open);
    EXPECT_EQ(out->value, 42);
}

TEST(CommandFromCan, PingParsedWithNoPayload)
{
    const auto c = cmd::fromCan(makeCanFrame(0x01, 0x02, wireId(CommandType::Ping)));
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->type, CommandType::Ping);
}

TEST(CommandFromCan, UnknownMessageIdRejected)
{
    EXPECT_FALSE(cmd::fromCan(makeCanFrame(0x01, 0x02, 0x7E)).has_value());
}

TEST(CommandFromCan, SetStateParsed)
{
    SetStateFrame rs{};
    rs.flags       = SET_STATE_FLAG_RESET;
    rs.requestedID = 5;

    const auto c = cmd::fromCan(
        makeCanFrame(0x02, 0x01, wireId(CommandType::SetState), asBytes(rs)));
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->type, CommandType::SetState);

    const auto* out = reinterpret_cast<const SetStateFrame*>(c->payload.data());
    EXPECT_EQ(out->flags, SET_STATE_FLAG_RESET);
    EXPECT_EQ(out->requestedID, 5);
}

/* ---- fromEthernet -------------------------------------------------------- */

TEST(CommandFromEthernet, SetStateParsedSourceFromHeader)
{
    SetStateFrame rs{};
    rs.flags       = SET_STATE_FLAG_SPREAD;
    rs.requestedID = 7;

    const auto bytes = asBytes(rs);
    const auto frame = makeEthFrame(0x03, wireId(CommandType::SetState), 4242, bytes);

    const auto c = cmd::fromEthernet(frame);
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->type, CommandType::SetState);
    EXPECT_EQ(c->source, 0x03);          // read from header.deviceID, not assumed
    EXPECT_EQ(c->timestamp_ms, 4242u);

    const auto* out = reinterpret_cast<const SetStateFrame*>(c->payload.data());
    EXPECT_EQ(out->requestedID, 7);
}

TEST(CommandFromEthernet, SetValvePositionParsed)
{
    SetValvePositionFrame servo{};
    servo.valve  = FcuValves::Dump;
    servo.action = ValveCommand::Close;
    servo.value  = 0;

    const auto frame = makeEthFrame(0x03, wireId(CommandType::SetValvePosition), 1, asBytes(servo));
    const auto c = cmd::fromEthernet(frame);
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->type, CommandType::SetValvePosition);

    const auto* out = reinterpret_cast<const SetValvePositionFrame*>(c->payload.data());
    EXPECT_EQ(out->valve, FcuValves::Dump);
    EXPECT_EQ(out->action, ValveCommand::Close);
}

TEST(CommandFromEthernet, PingParsedNoPayload)
{
    const auto frame = makeEthFrame(0x03, wireId(CommandType::Ping), 0);
    const auto c = cmd::fromEthernet(frame);
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->type, CommandType::Ping);
}

TEST(CommandFromEthernet, UnknownPayloadIdRejected)
{
    EXPECT_FALSE(cmd::fromEthernet(makeEthFrame(0x03, 0x99, 0)).has_value());
}

TEST(CommandFromEthernet, TruncatedPayloadRejected)
{
    // SetValvePosition needs 3 payload bytes; supply only 2.
    const std::array<uint8_t, 2> shortBody{0x01, 0x02};
    const auto frame = makeEthFrame(0x03, wireId(CommandType::SetValvePosition), 0, shortBody);
    EXPECT_FALSE(cmd::fromEthernet(frame).has_value());
}

TEST(CommandFromEthernet, ShortHeaderRejected)
{
    const std::array<uint8_t, 4> tooShort{};
    EXPECT_FALSE(cmd::fromEthernet(tooShort).has_value());
}

/* ---- single source of truth --------------------------------------------- */

TEST(CommandSsot, SameWireIdParsesIdenticallyAcrossTransports)
{
    SetValvePositionFrame servo{};
    servo.valve  = FcuValves::Fill;
    servo.action = ValveCommand::SetOpenedPct;
    servo.value  = 99;
    const auto bytes = asBytes(servo);

    // Identical wire id byte feeds both transports.
    const uint8_t id = wireId(CommandType::SetValvePosition);

    const auto fromCanCmd = cmd::fromCan(makeCanFrame(0x01, 0x02, id, bytes));
    const auto fromEthCmd = cmd::fromEthernet(makeEthFrame(0x03, id, 0, bytes));

    ASSERT_TRUE(fromCanCmd.has_value());
    ASSERT_TRUE(fromEthCmd.has_value());
    EXPECT_EQ(fromCanCmd->type, CommandType::SetValvePosition);
    EXPECT_EQ(fromEthCmd->type, fromCanCmd->type);
    EXPECT_EQ(fromCanCmd->payload, fromEthCmd->payload);
}
