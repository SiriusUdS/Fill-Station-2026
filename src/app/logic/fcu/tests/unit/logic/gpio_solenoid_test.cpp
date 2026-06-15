/* ------------------------------------------------------------------------- *
 * Unit tests for logic::fcu::GpioSolenoid — the concrete solenoid valve composed
 * over the digital-in/out seams. Exercised over fake GPIO lines: open/close drive the
 * coil output and stamp the edge ticks (only on an actual transition, since the FCU
 * drives them every tick); poll() mirrors the detect input onto the continuity LED;
 * info() reports the lot for the extended telemetry record.
 * ------------------------------------------------------------------------- */

#include "gpio_solenoid.hpp"

#include "support/fake_digital_out.hpp"
#include "support/fake_digital_in.hpp"

#include <gtest/gtest.h>

namespace {

using Solenoid = logic::fcu::GpioSolenoid<FakeDigitalOut, FakeDigitalIn, FakeDigitalOut>;

class GpioSolenoidTest : public ::testing::Test {
protected:
    FakeDigitalOut drive_;    // SOL_VALVE_STATE
    FakeDigitalIn  detect_;   // SOL_VALVE_DET
    FakeDigitalOut cont_;     // SOL_VALVE_CONT
    Solenoid       solenoid_{drive_, detect_, cont_};
};

TEST_F(GpioSolenoidTest, DefaultsAreSafe)
{
    const auto info = solenoid_.info();
    EXPECT_FALSE(info.status.open);
    EXPECT_FALSE(info.status.detected);
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

TEST_F(GpioSolenoidTest, PollMirrorsDetectOntoContinuityAndInfo)
{
    detect_.level = true;                      // solenoid wired up
    EXPECT_TRUE(solenoid_.poll());
    EXPECT_TRUE(cont_.state);                  // continuity LED lit
    EXPECT_TRUE(solenoid_.info().status.detected);

    detect_.level = false;                     // disconnected
    EXPECT_FALSE(solenoid_.poll());
    EXPECT_FALSE(cont_.state);                 // continuity LED off
    EXPECT_FALSE(solenoid_.info().status.detected);
}

} // namespace
