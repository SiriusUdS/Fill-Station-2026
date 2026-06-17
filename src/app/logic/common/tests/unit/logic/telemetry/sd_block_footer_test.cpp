/* ------------------------------------------------------------------------- *
 * Unit tests for the on-disk SD block footer (logic::telemetry::sd_block_footer).
 *
 * Proves the writer (stampSdBlockFooter, given the data-integrity CRC seam) and the
 * self-contained reader (sdBlockVerify / sdBlockScanForMagic, with its own software CRC)
 * agree: a stamped 4096-byte block round-trips, fills the whole block with no dead padding,
 * fails verification on any corrupted byte, and exposes a scannable MAGIC boundary.
 * ------------------------------------------------------------------------- */

#include "communication/protocol/telemetry/sd_block_footer.hpp"
#include "data_integrity/crc32.hpp"   // logic::data_integrity::crc32 (host software seam)

#include <gtest/gtest.h>

#include <array>
#include <cstring>

namespace {

using namespace logic::telemetry;

// Fill [0, n) with a recognizable, non-magic pattern.
void fillPayload(uint8_t* p, std::size_t n)
{
    for (std::size_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
}

/* The writer's CRC seam and the reader's self-contained CRC must compute identical values,
   or every stamped block would fail verification. */
TEST(SdBlockFooter, WriterSeamCrcMatchesReaderCrc)
{
    const uint8_t data[] = {0, 1, 2, 3, 250, 251, 252, 9, 9, 9, 42};
    EXPECT_EQ(sdBlockCrc32(data, sizeof(data)),
              logic::data_integrity::crc32(data, sizeof(data)));
}

/* A stamped 4096 block round-trips: the reader recovers the exact payload, timestamp and
   length, and the block is fully used (MAGIC fill absorbs the slack — no dead padding). */
TEST(SdBlockFooter, RoundTripsAFull4096Block)
{
    std::array<uint8_t, SD_LOG_BLOCK_BYTES> block{};
    const std::size_t payload = SD_BLOCK_PAYLOAD_CAP - 40;   // some records, room to spare
    fillPayload(block.data(), payload);

    stampSdBlockFooter(std::span<uint8_t>(block), payload, 0xABCD1234u, &logic::data_integrity::crc32);

    const auto view = sdBlockVerify(block);
    ASSERT_TRUE(view.has_value());
    EXPECT_EQ(view->trailer.saved_ms, 0xABCD1234u);
    EXPECT_EQ(view->trailer.payload_bytes, payload);
    ASSERT_EQ(view->payload.size(), payload);
    EXPECT_EQ(std::memcmp(view->payload.data(), block.data(), payload), 0);
    EXPECT_EQ(block.size(), SD_LOG_BLOCK_BYTES);   // exactly one sector-aligned block
}

/* The maximum payload leaves exactly the minimum footer (one MAGIC word + trailer). */
TEST(SdBlockFooter, MaxPayloadLeavesTheMinimumFooter)
{
    std::array<uint8_t, SD_LOG_BLOCK_BYTES> block{};
    fillPayload(block.data(), SD_BLOCK_PAYLOAD_CAP);

    stampSdBlockFooter(std::span<uint8_t>(block), SD_BLOCK_PAYLOAD_CAP, 1u, &logic::data_integrity::crc32);

    const auto view = sdBlockVerify(block);
    ASSERT_TRUE(view.has_value());
    EXPECT_EQ(view->payload.size(), SD_BLOCK_PAYLOAD_CAP);
}

/* Any corrupted byte in the payload, MAGIC fill, or trailer fails the CRC. */
TEST(SdBlockFooter, CorruptedByteFailsVerify)
{
    std::array<uint8_t, SD_LOG_BLOCK_BYTES> block{};
    fillPayload(block.data(), 256);
    stampSdBlockFooter(std::span<uint8_t>(block), 256, 1u, &logic::data_integrity::crc32);
    ASSERT_TRUE(sdBlockVerify(block).has_value());

    auto corrupt_at = [&](std::size_t i) {
        auto copy = block;
        copy[i] ^= 0xFFu;
        return sdBlockVerify(copy).has_value();
    };
    EXPECT_FALSE(corrupt_at(50));                          // a payload byte
    EXPECT_FALSE(corrupt_at(300));                         // a MAGIC-fill byte
    EXPECT_FALSE(corrupt_at(SD_LOG_BLOCK_BYTES - 8));      // the payload_bytes field
}

/* The MAGIC fill begins right after the payload, so a reader can resync to the boundary. */
TEST(SdBlockFooter, MagicFillIsScannableAtThePayloadEnd)
{
    std::array<uint8_t, SD_LOG_BLOCK_BYTES> block{};
    const std::size_t payload = 256;
    fillPayload(block.data(), payload);
    stampSdBlockFooter(std::span<uint8_t>(block), payload, 1u, &logic::data_integrity::crc32);

    EXPECT_EQ(sdBlockScanForMagic(block, 0), payload);
}

/* A block too small to even hold a footer is rejected, not misread. */
TEST(SdBlockFooter, RejectsAnUndersizedBlock)
{
    std::array<uint8_t, SD_BLOCK_MIN_FOOTER_BYTES - 1> tiny{};
    EXPECT_FALSE(sdBlockVerify(tiny).has_value());
}

} // namespace
