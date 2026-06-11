/* ------------------------------------------------------------------------- *
 * Unit tests for the Backup-SRAM persistent state blob (logic::control).
 *
 * These exercise the self-validation contract directly on stack instances: a
 * fresh blob is invalid, a saved blob round-trips, and any corruption (magic,
 * payload, or an interrupted commit) is rejected. The section placement and
 * battery retention are linker/hardware concerns and are not covered here.
 * ------------------------------------------------------------------------- */

#include "control/persistent_state.hpp"
#include "control/states.hpp"

#include <gtest/gtest.h>

#include <cstring>

using logic::control::PersistentState;
using logic::control::State;

namespace {

/* A committed blob holding `s`. */
PersistentState makeBlob(State s)
{
    PersistentState blob{};  // zero-initialised => invalid
    blob.saveState(s);       // stamps magic + CRC
    return blob;
}

TEST(PersistentState, ZeroInitialisedBlobIsInvalid)
{
    PersistentState blob{};
    EXPECT_FALSE(blob.validateLoadedState());
    EXPECT_FALSE(blob.loadState().has_value());
}

TEST(PersistentState, SavedBlobValidatesAndLoadsState)
{
    PersistentState blob = makeBlob(State::Unsafe);
    EXPECT_TRUE(blob.validateLoadedState());

    const auto loaded = blob.loadState();
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(*loaded, State::Unsafe);
}

TEST(PersistentState, EveryStateRoundTrips)
{
    for (const State s : {State::Init, State::Safe, State::Unsafe, State::Abort,
                          State::Error, State::Ignite, State::Test}) {
        const PersistentState blob = makeBlob(s);
        ASSERT_TRUE(blob.validateLoadedState());
        EXPECT_EQ(blob.loadState(), s);
    }
}

TEST(PersistentState, SurvivesAByteCopyReboot)
{
    /* Backup SRAM "retains" the bytes; a reboot is just the same bytes reread. */
    const PersistentState saved = makeBlob(State::Ignite);

    unsigned char raw[sizeof(PersistentState)];
    std::memcpy(raw, &saved, sizeof(saved));
    PersistentState reloaded;
    std::memcpy(&reloaded, raw, sizeof(reloaded));

    EXPECT_TRUE(reloaded.validateLoadedState());
    EXPECT_EQ(reloaded.loadState(), State::Ignite);
}

TEST(PersistentState, CorruptMagicIsRejected)
{
    PersistentState blob = makeBlob(State::Safe);
    blob.magic ^= 0x1u;
    EXPECT_FALSE(blob.validateLoadedState());
    EXPECT_FALSE(blob.loadState().has_value());
}

TEST(PersistentState, CorruptPayloadIsRejected)
{
    PersistentState blob = makeBlob(State::Safe);
    blob.fill_state = State::Ignite;  // changed without re-saving => CRC now stale
    EXPECT_FALSE(blob.validateLoadedState());
}

TEST(PersistentState, TornWriteIsRejected)
{
    /* magic written but CRC never recomputed (an interrupted saveState()). */
    PersistentState blob{};
    blob.fill_state = State::Unsafe;
    blob.magic      = logic::control::PERSISTENT_STATE_MAGIC;
    EXPECT_FALSE(blob.validateLoadedState());
}

TEST(PersistentState, SaveStateZeroesReservedPadding)
{
    PersistentState blob{};
    blob.reserved[0] = 0xAA;
    blob.reserved[1] = 0xBB;
    blob.reserved[2] = 0xCC;
    blob.saveState(State::Test);
    EXPECT_EQ(blob.reserved[0], 0);
    EXPECT_EQ(blob.reserved[1], 0);
    EXPECT_EQ(blob.reserved[2], 0);
    EXPECT_TRUE(blob.validateLoadedState());
}

} // namespace
