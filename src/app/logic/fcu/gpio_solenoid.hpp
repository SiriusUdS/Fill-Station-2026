#pragma once

#include <cstdint>

#include "actuation/interfaces/solenoid.hpp"        // logic::actuation::Solenoid (the contract it models)
#include "communication/protocol/devices/solenoid/solenoid_info.hpp"   // SolenoidInfo (the record info() returns)
#include "indication/interfaces/digital_out.hpp"    // logic::indication::DigitalOut
#include "sensing/interfaces/digital_in.hpp"        // logic::sensing::DigitalIn

/* ------------------------------------------------------------------------- *
 * GPIO-composed solenoid valve — the FCU's concrete logic::actuation::Solenoid,
 * built over three on/off seams (firmware GPIO drivers in board bring-up, fakes in
 * tests). The same shape as GpioEmatch (output + present-detect input + continuity
 * LED), but binary-valve flavoured:
 *   - drive (DigitalOut)       the coil output (SOL_VALVE_STATE). Driven open ONLY while
 *                              the SolenoidValve control flag is set AND the board is in
 *                              the Unsafe state (Control::serviceSolenoid enforces this
 *                              every tick), so the solenoid auto-closes on leaving Unsafe.
 *   - detect (DigitalIn)       the present input (SOL_VALVE_DET): is the solenoid wired up?
 *                              Its active level is the driver's concern (active-high for now).
 *   - continuity (DigitalOut)  an indicator LED (SOL_VALVE_CONT), lit whenever detected.
 *
 * open()/close() are idempotent and may be called every tick; the open/close edge ticks
 * in info() update only on an actual state change. poll() samples detect and mirrors it
 * onto the continuity LED. info() rides the extended telemetry record.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

/**
 * @brief The FCU solenoid valve, composed over its three on/off lines. Models
 *        @ref logic::actuation::Solenoid.
 * @tparam Drive       logic::indication::DigitalOut — the coil output (SOL_VALVE_STATE).
 * @tparam Detect      logic::sensing::DigitalIn     — the present input (SOL_VALVE_DET).
 * @tparam Continuity  logic::indication::DigitalOut — the continuity LED (SOL_VALVE_CONT).
 */
template <logic::indication::DigitalOut Drive, logic::sensing::DigitalIn Detect,
          logic::indication::DigitalOut Continuity>
class GpioSolenoid {
public:
    /** @brief Construct over the three lines; does not touch hardware (the platform
     *         drivers own pin config). */
    GpioSolenoid(Drive& drive, Detect& detect, Continuity& continuity)
        : drive_(drive), detect_(detect), continuity_(continuity) {}

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

    /** @brief Sample the present (detect) input and mirror it onto the continuity LED. Cheap
     *         and non-blocking; call every loop. Returns whether the solenoid is detected. */
    bool poll()
    {
        detected_       = detect_.read();
        continuity_lit_ = detected_;          // the continuity indicator mirrors presence for now
        continuity_.set(continuity_lit_);
        return detected_;
    }

    /** @brief The solenoid's telemetry unit: presence + open/closed state + the last
     *         open/close ticks. Kept current by poll()/open()/close(); the telemetry pipeline
     *         reads it into the extended record. */
    [[nodiscard]] SolenoidInfo info() const
    {
        SolenoidInfo i = {};
        i.status.detected   = detected_       ? 1u : 0u;
        i.status.open       = open_           ? 1u : 0u;
        i.status.continuity = continuity_lit_ ? 1u : 0u;
        i.last_opened_ms    = last_opened_ms_;
        i.last_closed_ms    = last_closed_ms_;
        return i;
    }

private:
    Drive&      drive_;       // coil output (SOL_VALVE_STATE)
    Detect&     detect_;      // present input (SOL_VALVE_DET)
    Continuity& continuity_;  // continuity indicator LED (SOL_VALVE_CONT)
    bool        detected_       = false;  // last poll()'s detect reading (drives the LED)
    bool        continuity_lit_ = false;  // last state driven onto the continuity line (SOL_VALVE_CONT)
    bool        open_           = false;  // open/closed state (true between open/close edges)
    uint32_t    last_opened_ms_ = 0;  // tick of the last open edge (0 = never since boot)
    uint32_t    last_closed_ms_ = 0;  // tick of the last close edge (0 = never since boot)
};

} // namespace logic::fcu
