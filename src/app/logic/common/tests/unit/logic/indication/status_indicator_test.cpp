/* ------------------------------------------------------------------------- *
 * Unit tests for the state-coloured main-loop indicator (logic::indication::
 * StatusIndicator).
 *
 * Drives it over three FakeDigitalOut LEDs and asserts that exactly one LED blinks
 * at a time — the one matching the current control state (green = Safe, yellow =
 * any other non-Error/Abort state, red = Error/Abort) — at the heartbeat cadence,
 * and that switching state moves the blink to the new colour.
 * ------------------------------------------------------------------------- */

#include "indication/status_indicator.hpp"
#include "support/fake_digital_out.hpp"
#include "system/state.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using logic::indication::StatusIndicator;
using logic::indication::STATUS_INDICATOR_TOGGLE_MS;
using State = logic::control::State;

constexpr uint32_t T = STATUS_INDICATOR_TOGGLE_MS;

class StatusIndicatorTest : public ::testing::Test {
protected:
    FakeDigitalOut                   green_;
    FakeDigitalOut                   yellow_;
    FakeDigitalOut                   red_;
    StatusIndicator<FakeDigitalOut>  indicator_{green_, yellow_, red_};

    void SetUp() override { indicator_.init(); }
};

TEST_F(StatusIndicatorTest, StartsAllOff)
{
    EXPECT_FALSE(green_.state);
    EXPECT_FALSE(yellow_.state);
    EXPECT_FALSE(red_.state);
}

TEST_F(StatusIndicatorTest, SafeBlinksGreenOnly)
{
    indicator_.tick(T, State::Safe);   // first on-phase
    EXPECT_TRUE(green_.state);
    EXPECT_FALSE(yellow_.state);
    EXPECT_FALSE(red_.state);
}

TEST_F(StatusIndicatorTest, ArmedStatesBlinkYellowOnly)
{
    for (const State s : {State::Unsafe, State::Ignite, State::Launch}) {
        indicator_.init();
        indicator_.tick(T, s);
        EXPECT_TRUE(yellow_.state)  << "state " << static_cast<int>(s);
        EXPECT_FALSE(green_.state)  << "state " << static_cast<int>(s);
        EXPECT_FALSE(red_.state)    << "state " << static_cast<int>(s);
    }
}

TEST_F(StatusIndicatorTest, ErrorAndAbortBlinkRedOnly)
{
    for (const State s : {State::Error, State::Abort}) {
        indicator_.init();
        indicator_.tick(T, s);
        EXPECT_TRUE(red_.state)     << "state " << static_cast<int>(s);
        EXPECT_FALSE(green_.state)  << "state " << static_cast<int>(s);
        EXPECT_FALSE(yellow_.state) << "state " << static_cast<int>(s);
    }
}

TEST_F(StatusIndicatorTest, TheActiveLedBlinks)
{
    indicator_.tick(T, State::Safe);       // on
    EXPECT_TRUE(green_.state);
    indicator_.tick(2 * T, State::Safe);   // off
    EXPECT_FALSE(green_.state);
    EXPECT_EQ(green_.toggles, 2u);         // one on + one off
}

TEST_F(StatusIndicatorTest, SwitchingStateMovesTheActiveLed)
{
    indicator_.tick(T, State::Safe);            // green on
    EXPECT_TRUE(green_.state);

    indicator_.tick(2 * T, State::Unsafe);      // off phase: green released
    EXPECT_FALSE(green_.state);
    indicator_.tick(3 * T, State::Unsafe);      // on phase: now yellow, green stays off
    EXPECT_TRUE(yellow_.state);
    EXPECT_FALSE(green_.state);
}

TEST_F(StatusIndicatorTest, HoldsTheLevelBetweenIntervals)
{
    indicator_.tick(T, State::Safe);         // green -> on
    indicator_.tick(T + 100, State::Safe);   // still inside the on phase
    EXPECT_TRUE(green_.state);
    EXPECT_EQ(green_.toggles, 1u);
}

} // namespace
