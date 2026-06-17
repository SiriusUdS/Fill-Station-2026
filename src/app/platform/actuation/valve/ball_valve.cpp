/**
  ******************************************************************************
  * @file    actuation/valve/ball_valve.cpp
  * @brief   Ball valve driver: one object per physical valve, owning the servo,
  *          limit switches and transition/timeout state machine, and modelling
  *          the logic-side logic::actuation::Valve contract. The valve keeps its
  *          own ValveInfo (state + status + commanded position) current; tick()
  *          samples the switches into it and advances the state machine.
  ******************************************************************************
  */

#include "actuation/valve/ball_valve.hpp"

#include <cstdint>
#include <optional>

using logic::actuation::ValveError;

namespace servo = platform::actuation::servomotor;

namespace {

constexpr float MIN_OPEN_PERCENT = 0.0F;
constexpr float MAX_OPEN_PERCENT = 100.0F;

/* Map a desired opening percentage to a servo travel percentage.
   TODO: a ball valve's flow vs. ball rotation is non-linear; replace this
   linear pass-through with the real curve (coefficients or a lookup table). */
float open_percent_to_servo_percent(float open_percent)
{
    return open_percent;  // linear stub
}

bool read_limit(const platform::actuation::valve::LimitSwitchConfig& limit)
{
    return HAL_GPIO_ReadPin(limit.port, limit.pin) == GPIO_PIN_SET;
}

/* Idle the servo: a 0-tick pulse width is 0 % duty — no pulse — so the servo is no longer
   driven (it relaxes). Used once a movement is complete (the valve rests on a mechanical
   stop) so the servo is not held forced. */
void idle_servo(const servo::ServoConfig& s)
{
    servo::setPulseWidth(s, 0);
}

} // namespace

namespace platform::actuation::valve {

void BallValve::init(const BallValveConfig& config)
{
    config_                          = config;

    // Translate the valve's PWM duty-cycle calibration into the servo's pulse-tick endpoints:
    // duty is a percentage of the full PWM period (ARR + 1), so setPercent() then maps 0..100 %
    // open linearly across the [closed, open] duty range. The board configures the timer's ARR
    // before calling init(), so the period read here is valid.
    const float period_ticks      = static_cast<float>(__HAL_TIM_GET_AUTORELOAD(config_.servo.htim)) + 1.0F;
    config_.servo.min_pulse_ticks = (config_.duty_closed_percent / 100.0F) * period_ticks;
    config_.servo.max_pulse_ticks = (config_.duty_open_percent   / 100.0F) * period_ticks;

    info_                            = ValveInfo{};   // state Unknown, no status bits, 0 % commanded
    info_.status.initialized         = 1u;            // bound and ready to operate
    info_.status.opened_switch_ignored = config_.opened_switch_ignored ? 1u : 0u;
    start_movement_ms_               = 0;
    end_movement_ms_                 = 0;
    servo::init(config_.servo);
}

std::optional<ValveError> BallValve::open(uint32_t bypass_ms)
{
    // With no physical open switch there is nothing to confirm an Opened state,
    // so "open" just drives fully open and floats there (== setOpenPercent(100)).
    if (config_.opened_switch_ignored) {
        return setOpenPercent(MAX_OPEN_PERCENT);   // clears bypass_ms_; no open switch to bypass
    }
    bypass_ms_ = bypass_ms;   // record the forced-actuation window (0 = a normal switch-monitored open)
    if (bypass_ms == 0 &&
        (info_.state == ValveState::Opened || info_.state == ValveState::Opening)) {
        // Normal open with no motion needed (already open / opening), but still record the
        // commanded position: an open command means 100 % regardless of whether it caused
        // movement, so telemetry reflects the operator's intent. A FORCED open (bypass_ms > 0)
        // skips this no-op and always (re)drives below, so the actuation is guaranteed.
        info_.current_set_value = static_cast<uint8_t>(MAX_OPEN_PERCENT);
        return std::nullopt;
    }
    info_.state                = ValveState::Opening;
    info_.current_set_value    = static_cast<uint8_t>(MAX_OPEN_PERCENT);
    info_.status.in_transition = 1u;
    start_movement_ms_         = HAL_GetTick();
    servo::setPercent(config_.servo, open_percent_to_servo_percent(MAX_OPEN_PERCENT));
    return std::nullopt;
}

std::optional<ValveError> BallValve::close(uint32_t bypass_ms)
{
    bypass_ms_ = bypass_ms;   // record the forced-actuation window (0 = a normal switch-monitored close)
    if (bypass_ms == 0 &&
        (info_.state == ValveState::Closed || info_.state == ValveState::Closing)) {
        // Normal close with no motion needed (already closed / closing), but still record the
        // commanded position: a close command means 0 % regardless of whether it caused movement,
        // so telemetry reflects the operator's intent. A FORCED close (bypass_ms > 0) skips this
        // no-op and always (re)drives below, so the actuation is guaranteed.
        info_.current_set_value = static_cast<uint8_t>(MIN_OPEN_PERCENT);
        return std::nullopt;
    }
    info_.state                = ValveState::Closing;
    info_.current_set_value    = static_cast<uint8_t>(MIN_OPEN_PERCENT);
    info_.status.in_transition = 1u;
    start_movement_ms_         = HAL_GetTick();
    servo::setPercent(config_.servo, open_percent_to_servo_percent(MIN_OPEN_PERCENT));
    return std::nullopt;
}

std::optional<ValveError> BallValve::setOpenPercent(float percent)
{
    if (percent < MIN_OPEN_PERCENT) percent = MIN_OPEN_PERCENT;
    if (percent > MAX_OPEN_PERCENT) percent = MAX_OPEN_PERCENT;

    bypass_ms_                 = 0;  // a proportional hold is never a forced switch-bypass move
    info_.current_set_value    = static_cast<uint8_t>(percent);
    info_.state                = ValveState::Floating;  // proportional hold, off both limits
    info_.status.in_transition = 1u;   // driving to the position; tick() idles the servo once it settles
    start_movement_ms_         = HAL_GetTick();  // transit-timeout base: idle after this elapses
    servo::setPercent(config_.servo, open_percent_to_servo_percent(percent));
    return std::nullopt;
}

void BallValve::tick(uint32_t now_ms)
{
    // The open switch is ignored on valves wired without one (its GPIO would float).
    const bool open_hit  = config_.opened_switch_ignored ? false : read_limit(config_.open_limit);
    const bool close_hit = read_limit(config_.close_limit);
    info_.status.open_limit_high   = open_hit  ? 1u : 0u;
    info_.status.closed_limit_high = close_hit ? 1u : 0u;

    // Both switches asserted at once is physically impossible — a hard fault.
    if (open_hit && close_hit) {
        info_.status.fault_both_switches = 1u;
        info_.status.in_transition       = 0u;
        info_.state                      = ValveState::Faulted;
        idle_servo(config_.servo);   // movement over (faulted): stop driving the servo
        return;
    }
    info_.status.fault_both_switches = 0u;

    switch (info_.state) {
        case ValveState::Opening:
            if (open_hit) {
                info_.state = ValveState::Opened;
                info_.status.in_transition = 0u;
                end_movement_ms_ = now_ms;
                idle_servo(config_.servo);   // reached the open stop: idle the servo (duty 0)
            } else if ((now_ms - start_movement_ms_) >= config_.max_transit_timeout_ms &&
                       (now_ms - start_movement_ms_) >= bypass_ms_) {
                // While inside the forced-actuation window (now - start < bypass_ms_) keep driving
                // open (Opening) without faulting, whether or not the switch asserts; once it
                // elapses the normal transit-timeout fault resumes.
                info_.state = ValveState::Faulted;
                info_.status.in_transition = 0u;
                end_movement_ms_ = now_ms;
                idle_servo(config_.servo);   // movement timed out: stop driving the servo
            }
            break;
        case ValveState::Closing:
            if (close_hit) {
                info_.state = ValveState::Closed;
                info_.status.in_transition = 0u;
                end_movement_ms_ = now_ms;
                idle_servo(config_.servo);   // reached the closed stop: idle the servo (duty 0)
            } else if ((now_ms - start_movement_ms_) >= config_.max_transit_timeout_ms &&
                       (now_ms - start_movement_ms_) >= bypass_ms_) {
                info_.state = ValveState::Faulted;
                info_.status.in_transition = 0u;
                end_movement_ms_ = now_ms;
                idle_servo(config_.servo);   // movement timed out: stop driving the servo
            }
            break;
        case ValveState::Opened:
            if (!open_hit) info_.state = ValveState::Floating;   // drifted off the open limit
            break;
        case ValveState::Closed:
            if (!close_hit) info_.state = ValveState::Floating;  // drifted off the closed limit
            break;
        case ValveState::Floating:
            if (open_hit) {
                info_.state = ValveState::Opened;
                info_.status.in_transition = 0u;
                idle_servo(config_.servo);   // drifted onto the open stop: idle the servo
            } else if (close_hit) {
                info_.state = ValveState::Closed;
                info_.status.in_transition = 0u;
                idle_servo(config_.servo);   // drifted onto the closed stop: idle the servo
            } else if (info_.status.in_transition &&
                       (now_ms - start_movement_ms_) >= config_.max_transit_timeout_ms) {
                // The proportional move (setOpenPercent) has had the full transit time to settle:
                // idle the servo (duty 0) so it is not held at the position, like a completed move.
                info_.status.in_transition = 0u;
                idle_servo(config_.servo);
            }
            break;
        default:  // Unknown / Faulted: adopt a limit if one is now asserted
            if (open_hit)       info_.state = ValveState::Opened;
            else if (close_hit) info_.state = ValveState::Closed;
            break;
    }
}

} // namespace platform::actuation::valve
