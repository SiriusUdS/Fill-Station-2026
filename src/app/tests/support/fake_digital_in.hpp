#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::sensing::DigitalIn contract.
 *
 * A settable on/off input: tests poke `level` and the logic reads it through
 * read(). Because the contract is structural, logic templates (e.g. GpioEmatch)
 * instantiate on this directly.
 * ------------------------------------------------------------------------- */

#include "sensing/interfaces/digital_in.hpp"

#include <cstdint>

/** @brief In-memory on/off input (models logic::sensing::DigitalIn). */
struct FakeDigitalIn {
    bool     level      = false;  /**< Value read() returns (the active state). */
    uint32_t read_calls = 0;      /**< Total read() calls. */

    bool read()
    {
        ++read_calls;
        return level;
    }
};

static_assert(logic::sensing::DigitalIn<FakeDigitalIn>);
