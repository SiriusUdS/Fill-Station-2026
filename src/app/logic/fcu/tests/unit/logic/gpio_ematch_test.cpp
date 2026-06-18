/* ------------------------------------------------------------------------- *
 * Unit tests for logic::fcu::GpioEmatch — the concrete e-match composed over the
 * digital-in/out seams. Exercised over fake GPIO lines: energise/deenergise drive
 * the firing output and stamp the edge ticks; poll() mirrors the detect input onto
 * the continuity LED; info() reports the lot for the extended telemetry record.
 * ------------------------------------------------------------------------- */

#include "gpio_ematch.hpp"

#include "support/fake_digital_out.hpp"
#include "support/fake_digital_in.hpp"

#include <gtest/gtest.h>

namespace {

using Ematch = logic::fcu::GpioEmatch<FakeDigitalOut, FakeDigitalIn, FakeDigitalOut>;

class GpioEmatchTest : public ::testing::Test {
protected:
    FakeDigitalOut fire_;     // EMATCH_STATE
    FakeDigitalIn  detect_;   // EMATCH_DET
    FakeDigitalOut cont_;     // EMATCH_CONT
    Ematch         ematch_{fire_, detect_, cont_};
};

TEST_F(GpioEmatchTest, DefaultsAreSafe)
{
    const auto info = ematch_.info();
    EXPECT_FALSE(info.status.energised);
    EXPECT_FALSE(info.status.detected);
    EXPECT_EQ(info.last_energised_ms, 0u);
    EXPECT_EQ(info.last_deenergised_ms, 0u);
}

TEST_F(GpioEmatchTest, EnergiseDrivesFiringLineHighAndStampsTheTick)
{
    ematch_.energise(1234);
    EXPECT_TRUE(fire_.state);                          // firing line driven active
    EXPECT_TRUE(ematch_.info().status.energised);
    EXPECT_EQ(ematch_.info().last_energised_ms, 1234u);
}

TEST_F(GpioEmatchTest, DeenergiseDrivesFiringLineLowAndStampsTheTick)
{
    ematch_.energise(1000);
    ematch_.deenergise(2000);
    EXPECT_FALSE(fire_.state);                            // firing line dropped
    EXPECT_FALSE(ematch_.info().status.energised);
    EXPECT_EQ(ematch_.info().last_deenergised_ms, 2000u);
    EXPECT_EQ(ematch_.info().last_energised_ms, 1000u);   // earlier energise tick retained
}

TEST_F(GpioEmatchTest, PollMirrorsDetectOntoContinuityAndInfo)
{
    detect_.level = true;                      // an e-match is plugged in
    EXPECT_TRUE(ematch_.poll());
    EXPECT_TRUE(cont_.state);                  // continuity LED lit
    EXPECT_TRUE(ematch_.info().status.detected);
    EXPECT_TRUE(ematch_.info().status.continuity);   // continuity-line state surfaced in telemetry

    detect_.level = false;                     // unplugged
    EXPECT_FALSE(ematch_.poll());
    EXPECT_FALSE(cont_.state);                 // continuity LED off
    EXPECT_FALSE(ematch_.info().status.detected);
    EXPECT_FALSE(ematch_.info().status.continuity);
}

} // namespace
