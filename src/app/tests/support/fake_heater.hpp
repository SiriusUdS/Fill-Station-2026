#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::actuation::Heater contract.
 *
 * Lets a test assert what the logic did: on/off call counts, the on/off state, the edge
 * timestamps (stamped only on an actual transition, like the real GpioHeater), and the
 * HeaterInfo the telemetry pipeline reads. Models the concept structurally.
 * ------------------------------------------------------------------------- */

#include "actuation/interfaces/heater.hpp"
#include "communication/protocol/devices/heater/heater_info.hpp"

#include <cstdint>

/** @brief In-memory heater (models logic::actuation::Heater). */
struct FakeHeater {
    bool     is_on      = false;  /**< On/off state (true between on/off edges). */
    uint32_t last_on_ms  = 0;     /**< now_ms of the last on edge. */
    uint32_t last_off_ms = 0;     /**< now_ms of the last off edge. */
    uint32_t on_calls  = 0;
    uint32_t off_calls = 0;

    void on(uint32_t now_ms)
    {
        ++on_calls;
        if (!is_on) { last_on_ms = now_ms; }   // stamp the edge only
        is_on = true;
    }

    void off(uint32_t now_ms)
    {
        ++off_calls;
        if (is_on) { last_off_ms = now_ms; }   // stamp the edge only
        is_on = false;
    }

    [[nodiscard]] HeaterInfo info() const
    {
        HeaterInfo i = {};
        i.status.on  = is_on ? 1u : 0u;
        i.last_on_ms  = last_on_ms;
        i.last_off_ms = last_off_ms;
        return i;
    }
};

static_assert(logic::actuation::Heater<FakeHeater>);
