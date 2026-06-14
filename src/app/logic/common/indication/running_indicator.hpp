#pragma once

#include <cstdint>

#include "indication/interfaces/digital_out.hpp"   // logic::indication::DigitalOut

/* ------------------------------------------------------------------------- *
 * Main-loop liveness indicator (HAL-free).
 *
 * Blinks a digital output at a fixed cadence while it is being ticked: a steadily
 * blinking LED means the main loop is running, a frozen LED means it has stalled
 * (or hard-faulted). It is driven straight from the board's for(;;) loop, NOT the
 * controller, so it reflects the loop's own liveness independently of any
 * peripheral or the record timer. Templated on a DigitalOut (a platform LED in
 * firmware, a fake in tests); board bring-up owns the actual pin.
 * ------------------------------------------------------------------------- */

namespace logic::indication {

/* Default half-period: the output toggles every 500 ms, so its on and off phases
   each last 500 ms — one on/off blink per second ("blink each second"). */
inline constexpr uint32_t RUNNING_INDICATOR_TOGGLE_MS = 500;

/**
 * @brief A main-loop heartbeat that blinks a DigitalOut at a fixed interval.
 * @tparam Out logic::indication::DigitalOut (a status LED in firmware).
 */
template <DigitalOut Out>
class RunningIndicator {
public:
    /**
     * @brief Construct over the output to blink and the toggle interval.
     * @param out                 the on/off line to drive (e.g. a status LED).
     * @param toggle_interval_ms  ms between state flips; the on and off phases each
     *                            last this long (default 500 -> one blink/second).
     */
    explicit RunningIndicator(Out& out, uint32_t toggle_interval_ms = RUNNING_INDICATOR_TOGGLE_MS)
        : out_(out), toggle_interval_ms_(toggle_interval_ms) {}

    /** @brief Drive the output off and reset the cadence; call once at startup. */
    void init()
    {
        on_             = false;
        last_toggle_ms_ = 0;
        out_.set(false);
    }

    /**
     * @brief Advance the blink: flip the output if a toggle interval has elapsed
     *        since the last flip. Call every main-loop iteration with the current
     *        tick. Cheap and non-blocking, so it is safe in the hot loop.
     */
    void tick(uint32_t now_ms)
    {
        if ((now_ms - last_toggle_ms_) < toggle_interval_ms_) {
            return;  // still inside the current on/off phase
        }
        last_toggle_ms_ = now_ms;
        on_             = !on_;
        out_.set(on_);
    }

private:
    Out&     out_;                  // injected output line, blinked by tick()
    uint32_t toggle_interval_ms_;   // ms per on/off phase
    uint32_t last_toggle_ms_ = 0;   // tick of the last flip (unsigned wrap-safe)
    bool     on_             = false;
};

} // namespace logic::indication
