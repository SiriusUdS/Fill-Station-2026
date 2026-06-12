/* ------------------------------------------------------------------------- *
 * Unit tests for the SystemState <-> CAN fragment codec (logic::communication::can).
 *
 * The codec is the agreed wire format between the ECU (packs + sends) and the FCU
 * (reassembles + relays), so the round-trip must be exact, order-independent, and
 * robust to dropped fragments / record boundaries.
 * ------------------------------------------------------------------------- */

#include "communication/protocol/can/system_state_codec.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

using logic::communication::CanFrame;
using logic::communication::can::SystemStateReassembler;
using logic::communication::can::SYSTEM_STATE_FRAGMENTS;

namespace codec = logic::communication::can;

namespace {

// A SystemState with a recognisable byte pattern (so a byte-exact round-trip is
// meaningful regardless of the struct's field layout).
SystemState makeRecord(uint8_t seed)
{
    SystemState s = {};
    auto* bytes = reinterpret_cast<uint8_t*>(&s);
    for (std::size_t i = 0; i < sizeof(SystemState); ++i) {
        bytes[i] = static_cast<uint8_t>(seed + i);
    }
    return s;
}

std::array<CanFrame, SYSTEM_STATE_FRAGMENTS> pack(const SystemState& s, uint8_t seq)
{
    std::array<CanFrame, SYSTEM_STATE_FRAGMENTS> frames{};
    codec::packSystemState(s, /*sender*/ 0x01, /*target*/ 0x02, seq,
                           std::span<CanFrame, SYSTEM_STATE_FRAGMENTS>(frames));
    return frames;
}

bool bytesEqual(const SystemState& a, const SystemState& b)
{
    return std::memcmp(&a, &b, sizeof(SystemState)) == 0;
}

}  // namespace

TEST(SystemStateCodec, RoundTripInOrder)
{
    const SystemState original = makeRecord(0x10);
    const auto frames = pack(original, /*seq*/ 3);

    SystemStateReassembler rx;
    std::optional<SystemState> out;
    for (const auto& f : frames) {
        if (auto r = rx.accept(f)) out = r;
    }

    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(bytesEqual(*out, original));
}

TEST(SystemStateCodec, RoundTripOutOfOrder)
{
    const SystemState original = makeRecord(0x55);
    auto frames = pack(original, /*seq*/ 7);

    SystemStateReassembler rx;
    std::optional<SystemState> out;
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {  // reversed
        if (auto r = rx.accept(*it)) out = r;
    }

    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(bytesEqual(*out, original));
}

TEST(SystemStateCodec, IncompleteYieldsNothing)
{
    const SystemState original = makeRecord(0x20);
    const auto frames = pack(original, /*seq*/ 1);

    SystemStateReassembler rx;
    for (std::size_t i = 0; i + 1 < frames.size(); ++i) {  // all but the last
        EXPECT_FALSE(rx.accept(frames[i]).has_value());
    }
}

TEST(SystemStateCodec, NewSequenceResetsInProgressRecord)
{
    const SystemState a = makeRecord(0x01);
    const SystemState b = makeRecord(0x99);
    const auto fa = pack(a, /*seq*/ 2);
    const auto fb = pack(b, /*seq*/ 3);

    SystemStateReassembler rx;
    for (std::size_t i = 0; i < fa.size() / 2; ++i) {  // partial 'a'
        EXPECT_FALSE(rx.accept(fa[i]).has_value());
    }
    std::optional<SystemState> out;
    for (const auto& f : fb) {                          // full 'b' (different seq)
        if (auto r = rx.accept(f)) out = r;
    }

    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(bytesEqual(*out, b));
}

TEST(SystemStateCodec, FragmentsCoverTheWholeRecord)
{
    // The fixed fragment count must be enough to carry every byte.
    EXPECT_GE(SYSTEM_STATE_FRAGMENTS * codec::FRAGMENT_PAYLOAD_BYTES, sizeof(SystemState));
    // Each frame is index byte + payload; the last carries the remainder.
    const auto frames = pack(makeRecord(0), 0);
    EXPECT_EQ(frames.front().data[0], 0u);
    EXPECT_EQ(frames.back().data[0], SYSTEM_STATE_FRAGMENTS - 1);
}
