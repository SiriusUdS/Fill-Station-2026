#pragma once

#include <cstdint>

#include "indication/interfaces/digital_out.hpp"   // logic::indication::DigitalOut
#include "system/state.hpp"                         // logic::control::State

/* ------------------------------------------------------------------------- *
 * Status / main-loop liveness indicator (HAL-free).
 *
 * A state-coloured heartbeat over three LEDs: exactly one LED blinks at a time —
 * the one whose colour matches the current control state — while the other two stay
 * off. The blink (not a steady level) is the loop-alive signal, as on the single-LED
 * RunningIndicator: a steadily blinking LED means the main loop is running, a frozen
 * one means it stalled. WHICH LED blinks encodes the state category:
 *   - GREEN  while Safe (the at-rest state);
 *   - YELLOW while in any armed / non-safe state (Unsafe / Ignite / Launch, and the
 *            transient Init / unused Test) — i.e. any state other than Safe/Error/Abort;
 *   - RED    while Error or Abort (a latched fault / abort).
 *
 * Driven straight from the board's for(;;) loop (NOT the controller) so it reflects
 * the loop's own liveness, with the current state passed in each tick. Templated on a
 * DigitalOut (platform LEDs in firmware, fakes in tests); board bring-up owns the pins
 * and their colours.
 * ------------------------------------------------------------------------- */

namespace logic::indication {

/* Default half-period: each LED's on and off phases last this long (one blink/second). */
inline constexpr uint32_t STATUS_INDICATOR_TOGGLE_MS = 500;

/**
 * @brief A state-coloured main-loop heartbeat over green / yellow / red LEDs.
 * @tparam Out logic::indication::DigitalOut (a status LED in firmware).
 */
template <DigitalOut Out>
class StatusIndicator {
public:
    /**
     * @brief Construct over the three LEDs to drive and the toggle interval.
     * @param green   lit (blinking) while Safe.
     * @param yellow  lit (blinking) while in any state other than Safe/Error/Abort.
     * @param red     lit (blinking) while Error or Abort.
     * @param toggle_interval_ms  ms per on/off phase (default 500 -> one blink/second).
     */
    StatusIndicator(Out& green, Out& yellow, Out& red,
                    uint32_t toggle_interval_ms = STATUS_INDICATOR_TOGGLE_MS)
        : green_(green), yellow_(yellow), red_(red), toggle_interval_ms_(toggle_interval_ms) {}

    /** @brief Drive all three LEDs off and reset the cadence; call once at startup. */
    void init()
    {
        on_             = false;
        last_toggle_ms_ = 0;
        green_.set(false);
        yellow_.set(false);
        red_.set(false);
    }

    /**
     * @brief Advance the heartbeat: flip the phase if a toggle interval has elapsed, then
     *        drive the LED matching @p state with the current phase and the other two off.
     *        Call every main-loop iteration with the current tick + control state. Cheap and
     *        non-blocking, so it is safe in the hot loop.
     */
    void tick(uint32_t now_ms, logic::control::State state)
    {
        if ((now_ms - last_toggle_ms_) >= toggle_interval_ms_) {
            last_toggle_ms_ = now_ms;
            on_             = !on_;
        }
        const Colour active = colourFor(state);
        green_.set(active == Colour::Green && on_);
        yellow_.set(active == Colour::Yellow && on_);
        red_.set(active == Colour::Red && on_);
    }

private:
    enum class Colour { Green, Yellow, Red };

    // State -> heartbeat colour: Error/Abort are RED, Safe is GREEN, every other state
    // (Unsafe/Ignite/Launch, plus the transient Init / unused Test) is YELLOW.
    [[nodiscard]] static Colour colourFor(logic::control::State state)
    {
        switch (state) {
            case logic::control::State::Error:
            case logic::control::State::Abort:
                return Colour::Red;
            case logic::control::State::Safe:
                return Colour::Green;
            default:
                return Colour::Yellow;
        }
    }

    Out&     green_;                // lit while Safe
    Out&     yellow_;               // lit while in any non-Safe/Error/Abort state
    Out&     red_;                  // lit while Error / Abort
    uint32_t toggle_interval_ms_;   // ms per on/off phase
    uint32_t last_toggle_ms_ = 0;   // tick of the last flip (unsigned wrap-safe)
    bool     on_             = false;
};

} // namespace logic::indication
