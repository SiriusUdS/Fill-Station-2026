/* ------------------------------------------------------------------------- *
 * Unit test for the class-based valve seam (logic::actuation::Valve).
 *
 * Demonstrates the point of the refactor: a piece of logic is written ONCE,
 * templated on the Valve concept, and the test drives it against a FakeValve
 * with no HAL and no separate link. The same template would instantiate on the
 * platform BallValve in firmware.
 * ------------------------------------------------------------------------- */

#include "actuation/interfaces/valve.hpp"
#include "support/fake_valve.hpp"

#include <optional>

#include "gtest/gtest.h"

using logic::actuation::Valve;
using logic::actuation::ValveError;

namespace {

/* A stand-in "logic component": close the valve unless it is already closed.
   This is the kind of code that previously had to call the free function
   logic::actuation::valve::close(id); now it is generic over any Valve and
   commands a concrete instance by reference, reading state from info(). */
template <Valve V>
std::optional<ValveError> ensureClosed(V& valve)
{
    if (valve.info().state == ValveState::Closed) return std::nullopt;
    return valve.close();
}

TEST(ValveSeam, GenericLogicCommandsTheInjectedValve)
{
    FakeValve valve;
    valve.info_.state = ValveState::Opened;

    const auto err = ensureClosed(valve);

    EXPECT_FALSE(err.has_value());
    EXPECT_EQ(valve.close_calls, 1);
    EXPECT_EQ(valve.info().state, ValveState::Closing);
}

TEST(ValveSeam, GenericLogicSkipsAnAlreadyClosedValve)
{
    FakeValve valve;
    valve.info_.state = ValveState::Closed;

    const auto err = ensureClosed(valve);

    EXPECT_FALSE(err.has_value());
    EXPECT_EQ(valve.close_calls, 0);  // no redundant command
}

TEST(ValveSeam, ErrorsPropagateFromTheValveToLogic)
{
    FakeValve valve;
    valve.info_.state = ValveState::Opened;
    valve.next_error  = ValveError::InternalError;

    const auto err = ensureClosed(valve);

    ASSERT_TRUE(err.has_value());
    EXPECT_EQ(*err, ValveError::InternalError);
}

TEST(ValveSeam, SetOpenPercentRecordsTheCommandedOpening)
{
    FakeValve valve;

    const auto err = valve.setOpenPercent(42.0F);

    EXPECT_FALSE(err.has_value());
    EXPECT_EQ(valve.percent_calls, 1);
    EXPECT_FLOAT_EQ(valve.last_percent, 42.0F);
    EXPECT_EQ(valve.info().current_set_value, 42);
}

} // namespace
