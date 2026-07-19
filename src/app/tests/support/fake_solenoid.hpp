#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::actuation::Solenoid contract.
 *
 * Lets a test assert what the logic did: open/close call counts, the open/closed state,
 * the edge timestamps (stamped only on an actual transition, like the real GpioSolenoid),
 * and the SolenoidInfo the telemetry pipeline reads. Models the concept structurally.
 * ------------------------------------------------------------------------- */

#include "actuation/interfaces/solenoid.hpp"
#include "communication/protocol/devices/solenoid/solenoid_info.hpp"

#include <cstdint>

/** @brief In-memory solenoid valve (models logic::actuation::Solenoid). */
struct FakeSolenoid {
    bool     is_open        = false;  /**< Open/closed state (true between open/close edges). */
    uint32_t last_opened_ms = 0;      /**< now_ms of the last open edge. */
    uint32_t last_closed_ms = 0;      /**< now_ms of the last close edge. */
    uint32_t open_calls  = 0;
    uint32_t close_calls = 0;

    void open(uint32_t now_ms)
    {
        ++open_calls;
        if (!is_open) { last_opened_ms = now_ms; }   // stamp the edge only
        is_open = true;
    }

    void close(uint32_t now_ms)
    {
        ++close_calls;
        if (is_open) { last_closed_ms = now_ms; }    // stamp the edge only
        is_open = false;
    }

    [[nodiscard]] SolenoidInfo info() const
    {
        SolenoidInfo i = {};
        i.status.open    = is_open ? 1u : 0u;
        i.last_opened_ms = last_opened_ms;
        i.last_closed_ms = last_closed_ms;
        return i;
    }
};

static_assert(logic::actuation::Solenoid<FakeSolenoid>);
