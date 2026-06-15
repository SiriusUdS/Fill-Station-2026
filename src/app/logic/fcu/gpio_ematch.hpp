#pragma once

#include <cstdint>

#include "actuation/interfaces/ematch.hpp"          // logic::actuation::Ematch (the contract it models)
#include "communication/protocol/devices/ematch/ematch_info.hpp"   // EmatchInfo (the record info() returns)
#include "indication/interfaces/digital_out.hpp"    // logic::indication::DigitalOut
#include "sensing/interfaces/digital_in.hpp"        // logic::sensing::DigitalIn

/* ------------------------------------------------------------------------- *
 * GPIO-composed e-match (igniter) — the FCU's concrete logic::actuation::Ematch,
 * built over three on/off seams (the firmware GPIO drivers in board bring-up, fakes
 * in tests). It owns no HAL detail itself; like RunningIndicator it is a logic
 * component templated on the digital-in/out seams:
 *   - fire (DigitalOut)        the firing/energise output (EMATCH_STATE). Held active
 *                              ONLY while in the Ignite state: Control energises it on
 *                              Unsafe -> Ignite and de-energises it on leaving Ignite by
 *                              ANY path (Launch, Abort, Safe), so the igniter is never
 *                              left hot once Ignite is exited.
 *   - detect (DigitalIn)       the safety/presence input (EMATCH_DET): is an e-match
 *                              plugged in? Its active level is the driver's concern
 *                              (active-high for now; flip in board wiring if it changes).
 *   - continuity (DigitalOut)  an indicator LED (EMATCH_CONT), lit whenever an e-match
 *                              is detected.
 *
 * poll() samples detect and mirrors it onto the continuity LED; it carries no timing,
 * so the controller just calls it every tick. Firing is NOT gated on detection here —
 * entry into Ignite is already gated by the state machine; detect/continuity surface
 * presence to the operator (and ride the extended telemetry record).
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

/**
 * @brief The FCU e-match, composed over its three on/off lines. Models
 *        @ref logic::actuation::Ematch.
 * @tparam Fire        logic::indication::DigitalOut — the firing output (EMATCH_STATE).
 * @tparam Detect      logic::sensing::DigitalIn     — the e-match-present input (EMATCH_DET).
 * @tparam Continuity  logic::indication::DigitalOut — the continuity LED (EMATCH_CONT).
 */
template <logic::indication::DigitalOut Fire, logic::sensing::DigitalIn Detect,
          logic::indication::DigitalOut Continuity>
class GpioEmatch {
public:
    /** @brief Construct over the three lines; does not touch hardware (the platform
     *         drivers own pin config). */
    GpioEmatch(Fire& fire, Detect& detect, Continuity& continuity)
        : fire_(fire), detect_(detect), continuity_(continuity) {}

    /** @brief Energise the firing line and stamp now_ms as the last-energised time. Called
     *         on Unsafe -> Ignite; holds the e-match hot for the duration of the Ignite state. */
    void energise(uint32_t now_ms)
    {
        fire_.set(true);
        energised_         = true;
        last_energised_ms_ = now_ms;
    }

    /** @brief De-energise the firing line and stamp now_ms as the last-deenergised time. Called
     *         on leaving Ignite by ANY path (Launch, Abort, Safe), so the igniter is never left hot. */
    void deenergise(uint32_t now_ms)
    {
        fire_.set(false);
        energised_           = false;
        last_deenergised_ms_ = now_ms;
    }

    /** @brief Sample the detect (e-match-present) input and mirror it onto the continuity
     *         LED. Cheap and non-blocking; call every loop. Returns whether an e-match is
     *         currently detected. */
    bool poll()
    {
        detected_ = detect_.read();
        continuity_.set(detected_);
        return detected_;
    }

    /** @brief The e-match's telemetry unit: presence + firing-line state + the last
     *         energise/deenergise ticks. Kept current by poll()/energise()/deenergise();
     *         the telemetry pipeline reads it into the extended record. */
    [[nodiscard]] EmatchInfo info() const
    {
        EmatchInfo i = {};
        i.status.detected     = detected_  ? 1u : 0u;
        i.status.energised    = energised_ ? 1u : 0u;
        i.last_energised_ms   = last_energised_ms_;
        i.last_deenergised_ms = last_deenergised_ms_;
        return i;
    }

private:
    Fire&       fire_;        // firing/energise output (EMATCH_STATE)
    Detect&     detect_;      // e-match-present input (EMATCH_DET)
    Continuity& continuity_;  // continuity indicator LED (EMATCH_CONT)
    bool        detected_  = false;  // last poll()'s detect reading (drives the LED)
    bool        energised_ = false;  // firing line state (true between energise/deenergise)
    uint32_t    last_energised_ms_   = 0;  // tick of the last energise() (0 = never since boot)
    uint32_t    last_deenergised_ms_ = 0;  // tick of the last deenergise() (0 = never since boot)
};

} // namespace logic::fcu
