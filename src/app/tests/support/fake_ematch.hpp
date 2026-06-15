#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::actuation::Ematch contract.
 *
 * Lets a test drive the e-match-present input (detect_present), then assert what
 * the logic did: energise/deenergise call counts, the firing-line state, the
 * recorded edge timestamps, and the EmatchInfo the telemetry pipeline reads. Models
 * the concept structurally, so the FCU controller instantiates on it directly.
 * ------------------------------------------------------------------------- */

#include "actuation/interfaces/ematch.hpp"
#include "communication/protocol/devices/ematch/ematch_info.hpp"

#include <cstdint>

/** @brief In-memory e-match (models logic::actuation::Ematch). */
struct FakeEmatch {
    bool     detect_present     = false;  /**< Test input: is an e-match "plugged in"? poll() reads this. */
    bool     detected           = false;  /**< Last poll()'s reading (mirrors detect_present). */
    bool     energised          = false;  /**< Firing-line state (true between energise/deenergise). */
    uint32_t last_energised_ms   = 0;     /**< now_ms of the last energise(). */
    uint32_t last_deenergised_ms = 0;     /**< now_ms of the last deenergise(). */
    uint32_t energise_calls   = 0;
    uint32_t deenergise_calls = 0;
    uint32_t poll_calls       = 0;

    void energise(uint32_t now_ms)
    {
        ++energise_calls;
        energised         = true;
        last_energised_ms = now_ms;
    }

    void deenergise(uint32_t now_ms)
    {
        ++deenergise_calls;
        energised           = false;
        last_deenergised_ms = now_ms;
    }

    bool poll()
    {
        ++poll_calls;
        detected = detect_present;
        return detected;
    }

    [[nodiscard]] EmatchInfo info() const
    {
        EmatchInfo i = {};
        i.status.detected     = detected  ? 1u : 0u;
        i.status.energised    = energised ? 1u : 0u;
        i.last_energised_ms   = last_energised_ms;
        i.last_deenergised_ms = last_deenergised_ms;
        return i;
    }
};

static_assert(logic::actuation::Ematch<FakeEmatch>);
