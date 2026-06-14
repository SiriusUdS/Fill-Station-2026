#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::indication::DigitalOut contract.
 *
 * Captures every set() so a test can assert what was driven and count the on/off
 * transitions — e.g. that RunningIndicator blinks at the right cadence. Because
 * the contract is structural, logic templates instantiate on this directly.
 * ------------------------------------------------------------------------- */

#include "indication/interfaces/digital_out.hpp"

#include <cstdint>
#include <vector>

/** @brief In-memory on/off output (models logic::indication::DigitalOut). */
struct FakeDigitalOut {
    bool              state     = false;  /**< Current driven level. */
    uint32_t          set_calls = 0;      /**< Total set() calls. */
    uint32_t          toggles   = 0;      /**< set() calls that changed the level. */
    std::vector<bool> history;            /**< Each level passed to set(), in order. */

    void set(bool on)
    {
        ++set_calls;
        if (on != state) {
            ++toggles;
        }
        state = on;
        history.push_back(on);
    }
};

static_assert(logic::indication::DigitalOut<FakeDigitalOut>);
