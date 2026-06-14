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

std::optional<ValveError> BallValve::open()
{
    // With no physical open switch there is nothing to confirm an Opened state,
    // so "open" just drives fully open and floats there (== setOpenPercent(100)).
    if (config_.opened_switch_ignored) {
        return setOpenPercent(MAX_OPEN_PERCENT);
    }
    if (info_.state == ValveState::Opened || info_.state == ValveState::Opening) {
        // No motion needed (already open / opening), but still record the commanded position:
        // an open command means 100 % regardless of whether it caused movement, so telemetry
        // reflects the operator's intent even when the actuation is a no-op (e.g. limit
        // switches unplugged and the valve already reads Opened).
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

std::optional<ValveError> BallValve::close()
{
    if (info_.state == ValveState::Closed || info_.state == ValveState::Closing) {
        // No motion needed (already closed / closing), but still record the commanded position:
        // a close command means 0 % regardless of whether it caused movement, so telemetry
        // reflects the operator's intent even when the actuation is a no-op (e.g. limit
        // switches unplugged and the valve already reads Closed).
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

    info_.current_set_value    = static_cast<uint8_t>(percent);
    info_.state                = ValveState::Floating;  // proportional hold, off both limits
    info_.status.in_transition = 0u;
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
        return;
    }
    info_.status.fault_both_switches = 0u;

    switch (info_.state) {
        case ValveState::Opening:
            if (open_hit) {
                info_.state = ValveState::Opened;
                info_.status.in_transition = 0u;
                end_movement_ms_ = now_ms;
            } else if ((now_ms - start_movement_ms_) >= config_.max_transit_timeout_ms) {
                info_.state = ValveState::Faulted;
                info_.status.in_transition = 0u;
                end_movement_ms_ = now_ms;
            }
            break;
        case ValveState::Closing:
            if (close_hit) {
                info_.state = ValveState::Closed;
                info_.status.in_transition = 0u;
                end_movement_ms_ = now_ms;
            } else if ((now_ms - start_movement_ms_) >= config_.max_transit_timeout_ms) {
                info_.state = ValveState::Faulted;
                info_.status.in_transition = 0u;
                end_movement_ms_ = now_ms;
            }
            break;
        case ValveState::Opened:
            if (!open_hit) info_.state = ValveState::Floating;   // drifted off the open limit
            break;
        case ValveState::Closed:
            if (!close_hit) info_.state = ValveState::Floating;  // drifted off the closed limit
            break;
        case ValveState::Floating:
            if (open_hit)       info_.state = ValveState::Opened;
            else if (close_hit) info_.state = ValveState::Closed;
            break;
        default:  // Unknown / Faulted: adopt a limit if one is now asserted
            if (open_hit)       info_.state = ValveState::Opened;
            else if (close_hit) info_.state = ValveState::Closed;
            break;
    }
}

} // namespace platform::actuation::valve
