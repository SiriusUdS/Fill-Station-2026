#pragma once

#include <cstdint>

#include "actuation/interfaces/solenoid.hpp"        // logic::actuation::Solenoid (the contract it models)
#include "communication/protocol/devices/solenoid/solenoid_info.hpp"   // SolenoidInfo (the record info() returns)
#include "indication/interfaces/digital_out.hpp"    // logic::indication::DigitalOut

/* ------------------------------------------------------------------------- *
 * GPIO-composed solenoid valve — the FCU's concrete logic::actuation::Solenoid,
 * built over a single on/off seam (a firmware GPIO driver in board bring-up, a fake in
 * tests):
 *   - drive (DigitalOut)  the coil output (SOLENOID_VALVE_STATE). Driven open ONLY while the
 *                         SolenoidValve control flag is set AND the board is in the Unsafe
 *                         state (Control::serviceSolenoid enforces this every tick), so the
 *                         solenoid auto-closes on leaving Unsafe.
 *
 * open()/close() are idempotent and may be called every tick; the open/close edge ticks
 * in info() update only on an actual state change. info() rides the extended telemetry
 * record.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

/**
 * @brief The FCU solenoid valve, composed over its coil line. Models
 *        @ref logic::actuation::Solenoid.
 * @tparam Drive  logic::indication::DigitalOut — the coil output (SOLENOID_VALVE_STATE).
 */
template <logic::indication::DigitalOut Drive>
class GpioSolenoid {
public:
    /** @brief Construct over the drive line; does not touch hardware (the platform
     *         driver owns pin config). */
    explicit GpioSolenoid(Drive& drive) : drive_(drive) {}

    /** @brief Drive the solenoid open (coil energised), stamping now_ms on the closed->open
     *         edge only. Idempotent: safe to call every tick while it should stay open. */
    void open(uint32_t now_ms)
    {
        if (!open_) {
            last_opened_ms_ = now_ms;
        }
        open_ = true;
        drive_.set(true);
    }

    /** @brief Drive the solenoid closed (also the boot-safe state), stamping now_ms on the
     *         open->closed edge only. Idempotent: safe to call every tick while it should stay
     *         closed (e.g. whenever not in the Unsafe state). */
    void close(uint32_t now_ms)
    {
        if (open_) {
            last_closed_ms_ = now_ms;
        }
        open_ = false;
        drive_.set(false);
    }

    /** @brief The solenoid's telemetry unit: open/closed state + the last open/close ticks.
     *         Kept current by open()/close(); the telemetry pipeline reads it into the
     *         extended record. */
    [[nodiscard]] SolenoidInfo info() const
    {
        SolenoidInfo i = {};
        i.status.open    = open_ ? 1u : 0u;
        i.last_opened_ms = last_opened_ms_;
        i.last_closed_ms = last_closed_ms_;
        return i;
    }

private:
    Drive&   drive_;                  // coil output (SOLENOID_VALVE_STATE)
    bool     open_           = false;  // open/closed state (true between open/close edges)
    uint32_t last_opened_ms_ = 0;      // tick of the last open edge (0 = never since boot)
    uint32_t last_closed_ms_ = 0;      // tick of the last close edge (0 = never since boot)
};

} // namespace logic::fcu
