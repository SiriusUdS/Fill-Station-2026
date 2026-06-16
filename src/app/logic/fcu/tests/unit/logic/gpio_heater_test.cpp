/* ------------------------------------------------------------------------- *
 * Unit tests for logic::fcu::GpioHeater — the concrete heater composed over a single
 * digital-out seam. Exercised over a fake GPIO line: on/off drive the output and stamp the
 * edge ticks (only on an actual transition, since the FCU drives them every tick); info()
 * reports the lot for the extended telemetry record. A heater has no detect input and no
 * continuity LED, so there is nothing to poll.
 * ------------------------------------------------------------------------- */

#include "gpio_heater.hpp"

#include "support/fake_digital_out.hpp"

#include <gtest/gtest.h>

namespace {

using Heater = logic::fcu::GpioHeater<FakeDigitalOut>;

class GpioHeaterTest : public ::testing::Test {
protected:
    FakeDigitalOut drive_;    // HEATER_STATE
    Heater         heater_{drive_};
};

TEST_F(GpioHeaterTest, DefaultsAreSafe)
{
    const auto info = heater_.info();
    EXPECT_FALSE(info.status.on);
    EXPECT_FALSE(drive_.state);
    EXPECT_EQ(info.last_on_ms, 0u);
    EXPECT_EQ(info.last_off_ms, 0u);
}

TEST_F(GpioHeaterTest, OnDrivesOutputHighAndStampsTheEdgeOnce)
{
    heater_.on(1000);
    EXPECT_TRUE(drive_.state);                       // output energised
    EXPECT_TRUE(heater_.info().status.on);
    EXPECT_EQ(heater_.info().last_on_ms, 1000u);

    heater_.on(2000);                                // already on: idempotent, no re-stamp
    EXPECT_TRUE(heater_.info().status.on);
    EXPECT_EQ(heater_.info().last_on_ms, 1000u);
}

TEST_F(GpioHeaterTest, OffDrivesOutputLowAndStampsTheEdgeOnce)
{
    heater_.on(1000);
    heater_.off(2000);
    EXPECT_FALSE(drive_.state);                       // output dropped
    EXPECT_FALSE(heater_.info().status.on);
    EXPECT_EQ(heater_.info().last_off_ms, 2000u);
    EXPECT_EQ(heater_.info().last_on_ms, 1000u);      // earlier on tick retained

    heater_.off(3000);                               // already off: idempotent, no re-stamp
    EXPECT_EQ(heater_.info().last_off_ms, 2000u);
}

} // namespace
