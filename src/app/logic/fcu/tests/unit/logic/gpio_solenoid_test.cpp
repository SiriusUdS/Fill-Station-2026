/* ------------------------------------------------------------------------- *
 * Unit tests for logic::fcu::GpioSolenoid — the concrete solenoid valve composed
 * over the digital-out seam. Exercised over a fake GPIO line: open/close drive the
 * coil output and stamp the edge ticks (only on an actual transition, since the FCU
 * drives them every tick); info() reports the lot for the extended telemetry record.
 * ------------------------------------------------------------------------- */

#include "gpio_solenoid.hpp"

#include "support/fake_digital_out.hpp"

#include <gtest/gtest.h>

namespace {

using Solenoid = logic::fcu::GpioSolenoid<FakeDigitalOut>;

class GpioSolenoidTest : public ::testing::Test {
protected:
    FakeDigitalOut drive_;    // SOLENOID_VALVE_STATE
    Solenoid       solenoid_{drive_};
};

TEST_F(GpioSolenoidTest, DefaultsAreSafe)
{
    const auto info = solenoid_.info();
    EXPECT_FALSE(info.status.open);
    EXPECT_FALSE(drive_.state);          // coil de-energised before any call
    EXPECT_EQ(info.last_opened_ms, 0u);
    EXPECT_EQ(info.last_closed_ms, 0u);
}

TEST_F(GpioSolenoidTest, OpenDrivesCoilHighAndStampsTheEdgeOnce)
{
    solenoid_.open(1000);
    EXPECT_TRUE(drive_.state);                       // coil energised
    EXPECT_TRUE(solenoid_.info().status.open);
    EXPECT_EQ(solenoid_.info().last_opened_ms, 1000u);

    solenoid_.open(2000);                            // already open: idempotent, no re-stamp
    EXPECT_TRUE(solenoid_.info().status.open);
    EXPECT_EQ(solenoid_.info().last_opened_ms, 1000u);
}

TEST_F(GpioSolenoidTest, CloseDrivesCoilLowAndStampsTheEdgeOnce)
{
    solenoid_.open(1000);
    solenoid_.close(2000);
    EXPECT_FALSE(drive_.state);                       // coil dropped
    EXPECT_FALSE(solenoid_.info().status.open);
    EXPECT_EQ(solenoid_.info().last_closed_ms, 2000u);
    EXPECT_EQ(solenoid_.info().last_opened_ms, 1000u);  // earlier open tick retained

    solenoid_.close(3000);                            // already closed: idempotent, no re-stamp
    EXPECT_EQ(solenoid_.info().last_closed_ms, 2000u);
}

} // namespace
