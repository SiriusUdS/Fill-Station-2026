/* ------------------------------------------------------------------------- *
 * Unit tests for the main-loop liveness indicator (logic::indication::
 * RunningIndicator).
 *
 * Drives it over a FakeDigitalOut and asserts the blink cadence: it starts off,
 * flips only once a toggle interval has elapsed, holds the level in between, and
 * produces one on/off blink per second at the default interval. A custom interval
 * is honoured too.
 * ------------------------------------------------------------------------- */

#include "indication/running_indicator.hpp"
#include "support/fake_digital_out.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using logic::indication::RunningIndicator;
using logic::indication::RUNNING_INDICATOR_TOGGLE_MS;

class RunningIndicatorTest : public ::testing::Test {
protected:
    FakeDigitalOut                   led_;
    RunningIndicator<FakeDigitalOut> indicator_{led_};

    void SetUp() override { indicator_.init(); }
};

TEST_F(RunningIndicatorTest, StartsOff)
{
    EXPECT_FALSE(led_.state);
}

TEST_F(RunningIndicatorTest, DoesNotToggleBeforeTheInterval)
{
    indicator_.tick(RUNNING_INDICATOR_TOGGLE_MS - 1);
    EXPECT_FALSE(led_.state);
    EXPECT_EQ(led_.toggles, 0u);
}

TEST_F(RunningIndicatorTest, TogglesOnAtTheInterval)
{
    indicator_.tick(RUNNING_INDICATOR_TOGGLE_MS);
    EXPECT_TRUE(led_.state);
    EXPECT_EQ(led_.toggles, 1u);
}

TEST_F(RunningIndicatorTest, HoldsTheLevelBetweenIntervals)
{
    indicator_.tick(RUNNING_INDICATOR_TOGGLE_MS);         // -> on at 500
    indicator_.tick(RUNNING_INDICATOR_TOGGLE_MS + 100);   // still inside the on phase
    EXPECT_TRUE(led_.state);
    EXPECT_EQ(led_.toggles, 1u);
}

TEST_F(RunningIndicatorTest, BlinksOncePerSecondAtTheDefaultInterval)
{
    // Two flips (on then off) make one full blink; at 500 ms each that is one
    // blink per second.
    uint32_t now = 0;
    indicator_.tick(now += RUNNING_INDICATOR_TOGGLE_MS);  // -> on
    indicator_.tick(now += RUNNING_INDICATOR_TOGGLE_MS);  // -> off
    EXPECT_FALSE(led_.state);
    EXPECT_EQ(led_.toggles, 2u);   // one on + one off within 1000 ms
}

TEST(RunningIndicatorConfig, RespectsACustomInterval)
{
    FakeDigitalOut                   led;
    RunningIndicator<FakeDigitalOut> fast{led, /*toggle_interval_ms=*/100};
    fast.init();

    fast.tick(99);
    EXPECT_FALSE(led.state);   // not yet
    fast.tick(100);
    EXPECT_TRUE(led.state);    // flipped at the custom interval
}

} // namespace
