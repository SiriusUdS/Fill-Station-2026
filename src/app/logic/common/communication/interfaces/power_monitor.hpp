#pragma once

#include <concepts>
#include <cstdint>

#include "communication/protocol/peripherals/power_monitor/power_monitor_info.hpp"   // PowerMonitorInfo

/* ------------------------------------------------------------------------- *
 * Class-based power-monitor contract for the logic layer (C++23 concept).
 *
 * The I2C sibling of the thermocouple seam (thermocouple.hpp): one device (the
 * INA3221 on I2C4 in firmware) polled WITHOUT blocking. service() advances an
 * interrupt-driven round-robin over the device's measurement registers one step at
 * a time, so the foreground loop never stalls on the bus; info() returns the latest
 * snapshot (state + status + per-channel shunt/bus codes) the logic folds into
 * telemetry. Unlike the thermocouple BANK (4 separate MAX31856), this is a single
 * 3-channel device, so info() returns one record, not an array. No HAL / I2C detail
 * appears here — acquisition mechanism is the driver's concern, which this hides.
 * ------------------------------------------------------------------------- */

namespace logic::communication {

/**
 * @brief A polled multi-channel power monitor, acquired without blocking.
 *
 * service(now_ms) — advance acquisition by one non-blocking step (kick the next
 *                   register's interrupt-driven read, or harvest/abandon the one in
 *                   flight). Call every foreground iteration; it self-paces and times
 *                   a stuck transfer out, so it never blocks the loop.
 * info()          — the latest PowerMonitorInfo snapshot (state + status + per-channel
 *                   shunt/bus codes); cheap, owned and kept current by the driver.
 */
template <typename T>
concept PowerMonitor = requires(T pm, uint32_t now_ms) {
    { pm.service(now_ms) } -> std::same_as<void>;
    { pm.info() }          -> std::same_as<::PowerMonitorInfo>;
};

} // namespace logic::communication
