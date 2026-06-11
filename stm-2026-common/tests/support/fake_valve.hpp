#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::actuation::Valve contract.
 *
 * Models the same concept as the platform BallValve, with no HAL: tests inject
 * a FakeValve into logic templated on Valve, script the info it reports (state /
 * status / position), and inspect what the logic commanded (call counts, last
 * percent). Because the concept is structural, this needs no inheritance and no
 * separate link — the test instantiates the logic template on FakeValve directly.
 * ------------------------------------------------------------------------- */

#include "actuation/interfaces/valve.hpp"

#include <optional>

/**
 * @brief In-memory stand-in for a valve.
 *
 * Inputs (what logic sees):  @ref info_ (script its state/status), returned by
 *                            info(); @ref next_error, returned by every command.
 * Outputs (what logic did):  @ref open_calls / @ref close_calls / @ref last_percent.
 */
struct FakeValve {
    /* ---- inputs the test scripts ---- */
    ValveInfo info_{};                                          /**< Scriptable; returned by info(). */
    std::optional<logic::actuation::ValveError> next_error{};   /**< Forced result of the next command(s). */

    /* ---- outputs the test inspects ---- */
    int   open_calls    = 0;
    int   close_calls   = 0;
    int   percent_calls = 0;
    float last_percent  = -1.0F;

    [[nodiscard]] std::optional<logic::actuation::ValveError> open()
    {
        ++open_calls;
        if (!next_error) { info_.state = ValveState::Opening; info_.current_set_value = 100; }
        return next_error;
    }

    [[nodiscard]] std::optional<logic::actuation::ValveError> close()
    {
        ++close_calls;
        if (!next_error) { info_.state = ValveState::Closing; info_.current_set_value = 0; }
        return next_error;
    }

    [[nodiscard]] std::optional<logic::actuation::ValveError> setOpenPercent(float percent)
    {
        ++percent_calls;
        last_percent = percent;
        if (!next_error) { info_.state = ValveState::Floating; info_.current_set_value = static_cast<uint8_t>(percent); }
        return next_error;
    }

    [[nodiscard]] ValveInfo info() const { return info_; }
};

// Same compile-time guarantee the platform driver carries: the double really
// does model the contract the logic is written against.
static_assert(logic::actuation::Valve<FakeValve>);
