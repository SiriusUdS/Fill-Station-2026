/* ------------------------------------------------------------------------- *
 * Unit tests for the shared SD recorder (logic::telemetry::SdRecorder).
 *
 * Proves the no-fragmentation guarantee: every stream — data_fast (raw), data_slow
 * (averaged) and data_ext — is written one footer-stamped, CRC-verifiable
 * SD_LOG_BLOCK_BYTES block at a time, never a sub-block / unaligned chunk. Driven over
 * FakeStorage, which captures each write() so we can size and verify it.
 * ------------------------------------------------------------------------- */

#include "telemetry/sd_recorder.hpp"
#include "communication/protocol/telemetry/sd_block_footer.hpp"
#include "communication/protocol/telemetry/fcu_system_state.hpp"   // FcuSystemState (the high-rate Record)

#include "support/fake_storage.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstring>

namespace {

using logic::telemetry::SdRecorder;
using logic::telemetry::SD_LOG_BLOCK_BYTES;
using logic::telemetry::SD_BLOCK_PAYLOAD_CAP;
using logic::telemetry::sdBlockVerify;

// A whole number of high-rate records that fills a slot up to the payload cap.
constexpr std::size_t kRecordsPerSlot = SD_BLOCK_PAYLOAD_CAP / sizeof(FcuSystemState);
constexpr std::size_t kSlotPayload    = kRecordsPerSlot * sizeof(FcuSystemState);

class SdRecorderTest : public ::testing::Test {
protected:
    FakeStorage fast_;
    FakeStorage slow_;
    FakeStorage ext_;
    SdRecorder<FakeStorage, FcuSystemState> rec_{fast_, slow_, ext_};

    void SetUp() override
    {
        logic::control::base_control_flags = logic::control::ControlFlags<ControlFlagBase>{};
        rec_.init();
    }

    // One slot's worth of distinguishable records in a 4096 buffer; returns it by value.
    static std::array<uint8_t, SD_LOG_BLOCK_BYTES> makeSlot(uint32_t tag)
    {
        std::array<uint8_t, SD_LOG_BLOCK_BYTES> block{};
        for (std::size_t off = 0; off + sizeof(FcuSystemState) <= kSlotPayload;
             off += sizeof(FcuSystemState)) {
            FcuSystemState r{};
            r.base.creation_timestamp_ms = tag + static_cast<uint32_t>(off);
            std::memcpy(block.data() + off, &r, sizeof(r));
        }
        return block;
    }
};

/* data_fast: each drained slot becomes exactly one 4096 footer-verified block whose payload
   is the records verbatim. */
TEST_F(SdRecorderTest, FastWritesOne4096FooterVerifiedBlockPerSlot)
{
    logic::control::base_control_flags.set(ControlFlagBase::FastRecording, true);

    auto block = makeSlot(/*tag=*/1000);
    rec_.recordSystemState(std::span<uint8_t>(block), kSlotPayload, /*now_ms=*/4242);

    ASSERT_EQ(fast_.writes.size(), 1u);
    EXPECT_EQ(fast_.writes[0].size(), SD_LOG_BLOCK_BYTES);   // sector-aligned, no fragmentation

    const auto view = sdBlockVerify(fast_.writes[0]);
    ASSERT_TRUE(view.has_value());
    EXPECT_EQ(view->trailer.saved_ms, 4242u);
    EXPECT_EQ(view->trailer.payload_bytes, kSlotPayload);
    // The recorder must not touch the records (it only stamps the tail).
    auto fresh = makeSlot(1000);
    EXPECT_EQ(std::memcmp(view->payload.data(), fresh.data(), kSlotPayload), 0);
}

/* data_slow: averaged records accumulate into 4096 footer-verified blocks (not the small
   per-drain chunks the old recorder wrote). */
TEST_F(SdRecorderTest, SlowWritesAccumulateInto4096FooterVerifiedBlocks)
{
    // FastRecording stays off -> averaged data_slow path.
    for (int i = 0; i < 1000 && slow_.writes.empty(); ++i) {
        auto block = makeSlot(static_cast<uint32_t>(i));
        rec_.recordSystemState(std::span<uint8_t>(block), kSlotPayload, static_cast<uint32_t>(i));
    }
    ASSERT_FALSE(slow_.writes.empty()) << "no averaged block ever filled";
    EXPECT_EQ(slow_.writes[0].size(), SD_LOG_BLOCK_BYTES);
    EXPECT_TRUE(sdBlockVerify(slow_.writes[0]).has_value());
}

/* data_ext: extended records accumulate into 4096 footer-verified blocks. */
TEST_F(SdRecorderTest, ExtWritesAccumulateInto4096FooterVerifiedBlocks)
{
    struct FakeExtended { uint8_t bytes[200]; };   // a stand-in low-rate record
    FakeExtended e{};

    for (int i = 0; i < 1000 && ext_.writes.empty(); ++i) {
        std::memset(e.bytes, i & 0xFF, sizeof(e.bytes));
        rec_.recordExtended(e, static_cast<uint32_t>(i));
    }
    ASSERT_FALSE(ext_.writes.empty()) << "no ext block ever filled";
    EXPECT_EQ(ext_.writes[0].size(), SD_LOG_BLOCK_BYTES);

    const auto view = sdBlockVerify(ext_.writes[0]);
    ASSERT_TRUE(view.has_value());
    EXPECT_EQ(view->trailer.payload_bytes % sizeof(FakeExtended), 0u);   // a whole number of records
}

} // namespace
