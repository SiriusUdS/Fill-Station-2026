#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::communication::PowerMonitor contract.
 *
 * Stands in for the INA3221: service() is a no-op that just records it was advanced
 * (the real driver drives interrupt-driven I2C there), and info() returns the
 * scriptable snapshot a test sets up. Because the contract is structural, logic
 * templates instantiate on this directly.
 * ------------------------------------------------------------------------- */

#include "communication/interfaces/power_monitor.hpp"

#include <cstdint>

/** @brief In-memory power monitor (models logic::communication::PowerMonitor). */
struct FakePowerMonitor {
    PowerMonitorInfo info_value{};   /**< Returned by info(); script per channel. */
    uint32_t service_calls = 0;      /**< Number of service() calls. */
    uint32_t last_now_ms   = 0;      /**< now_ms of the last service() call. */

    void service(uint32_t now_ms)
    {
        ++service_calls;
        last_now_ms = now_ms;
    }

    [[nodiscard]] PowerMonitorInfo info() const { return info_value; }

    /** @brief Test helper: set the info() snapshot. */
    void set(const PowerMonitorInfo& v) { info_value = v; }
};

static_assert(logic::communication::PowerMonitor<FakePowerMonitor>);
