#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::actuation::Solenoid contract.
 *
 * Lets a test drive the present input (detect_present), then assert what the logic
 * did: open/close call counts, the open/closed state, the edge timestamps (stamped
 * only on an actual transition, like the real GpioSolenoid), and the SolenoidInfo the
 * telemetry pipeline reads. Models the concept structurally.
 * ------------------------------------------------------------------------- */

#include "actuation/interfaces/solenoid.hpp"
#include "communication/protocol/devices/solenoid/solenoid_info.hpp"

#include <cstdint>

/** @brief In-memory solenoid valve (models logic::actuation::Solenoid). */
struct FakeSolenoid {
    bool     detect_present = false;  /**< Test input: is the solenoid "present"? poll() reads this. */
    bool     detected       = false;  /**< Last poll()'s reading (mirrors detect_present). */
    bool     continuity     = false;  /**< Last continuity-line state (mirrors detected, like the real driver). */
    bool     is_open        = false;  /**< Open/closed state (true between open/close edges). */
    uint32_t last_opened_ms = 0;      /**< now_ms of the last open edge. */
    uint32_t last_closed_ms = 0;      /**< now_ms of the last close edge. */
    uint32_t open_calls  = 0;
    uint32_t close_calls = 0;
    uint32_t poll_calls  = 0;

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

    bool poll()
    {
        ++poll_calls;
        detected   = detect_present;
        continuity = detected;
        return detected;
    }

    [[nodiscard]] SolenoidInfo info() const
    {
        SolenoidInfo i = {};
        i.status.detected   = detected   ? 1u : 0u;
        i.status.open       = is_open    ? 1u : 0u;
        i.status.continuity = continuity ? 1u : 0u;
        i.last_opened_ms    = last_opened_ms;
        i.last_closed_ms    = last_closed_ms;
        return i;
    }
};

static_assert(logic::actuation::Solenoid<FakeSolenoid>);
