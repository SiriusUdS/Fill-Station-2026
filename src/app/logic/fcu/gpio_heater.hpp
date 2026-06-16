#pragma once

#include <cstdint>

#include "actuation/interfaces/heater.hpp"          // logic::actuation::Heater (the contract it models)
#include "communication/protocol/devices/heater/heater_info.hpp"   // HeaterInfo (the record info() returns)
#include "indication/interfaces/digital_out.hpp"    // logic::indication::DigitalOut

/* ------------------------------------------------------------------------- *
 * GPIO-composed heater — the FCU's concrete logic::actuation::Heater, built over a
 * single on/off seam (a firmware GPIO driver in board bring-up, a fake in tests). The
 * same flavour as GpioSolenoid but stripped to its core: a heater is a bare on/off output,
 * so there is no present-detect input and no continuity LED — just the drive line:
 *   - drive (DigitalOut)  the heater output (HEATER_STATE). Driven on/off straight from the
 *                         Heater control flag (Control::serviceHeater) every tick.
 *
 * on()/off() are idempotent and may be called every tick; the on/off edge ticks in info()
 * update only on an actual state change. info() rides the extended telemetry record.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

/**
 * @brief The FCU heater, composed over its single on/off line. Models
 *        @ref logic::actuation::Heater.
 * @tparam Drive  logic::indication::DigitalOut — the heater output (HEATER_STATE).
 */
template <logic::indication::DigitalOut Drive>
class GpioHeater {
public:
    /** @brief Construct over the drive line; does not touch hardware (the platform
     *         driver owns pin config). */
    explicit GpioHeater(Drive& drive) : drive_(drive) {}

    /** @brief Drive the heater on (output energised), stamping now_ms on the off->on edge
     *         only. Idempotent: safe to call every tick while it should stay on. */
    void on(uint32_t now_ms)
    {
        if (!on_) {
            last_on_ms_ = now_ms;
        }
        on_ = true;
        drive_.set(true);
    }

    /** @brief Drive the heater off (also the boot-safe state), stamping now_ms on the on->off
     *         edge only. Idempotent: safe to call every tick while it should stay off. */
    void off(uint32_t now_ms)
    {
        if (on_) {
            last_off_ms_ = now_ms;
        }
        on_ = false;
        drive_.set(false);
    }

    /** @brief The heater's telemetry unit: on/off state + the last on/off ticks. Kept current
     *         by on()/off(); the telemetry pipeline reads it into the extended record. */
    [[nodiscard]] HeaterInfo info() const
    {
        HeaterInfo i = {};
        i.status.on  = on_ ? 1u : 0u;
        i.last_on_ms  = last_on_ms_;
        i.last_off_ms = last_off_ms_;
        return i;
    }

private:
    Drive&   drive_;            // heater output (HEATER_STATE)
    bool     on_         = false;  // on/off state (true between on/off edges)
    uint32_t last_on_ms_  = 0;     // tick of the last on edge (0 = never since boot)
    uint32_t last_off_ms_ = 0;     // tick of the last off edge (0 = never since boot)
};

} // namespace logic::fcu
