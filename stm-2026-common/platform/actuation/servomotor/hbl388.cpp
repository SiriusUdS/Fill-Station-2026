/**
  ******************************************************************************
  * @file    actuation/servomotor/hbl388.cpp
  * @brief   HBL388 servo DIL: maps a 0..100% travel command to a PWM compare
  *          value and drives it on a HAL timer channel.
  ******************************************************************************
  */

#include "actuation/servomotor/hbl388.hpp"

namespace {

constexpr float MIN_PERCENT = 0.0F;
constexpr float MAX_PERCENT = 100.0F;

} // namespace

namespace platform::actuation::servomotor {

void init(const ServoConfig& servo)
{
    HAL_TIM_PWM_Start(servo.htim, servo.channel);
}

void setPulseWidth(const ServoConfig& servo, uint32_t width_ticks)
{
    __HAL_TIM_SET_COMPARE(servo.htim, servo.channel, width_ticks);
}

void setPercent(const ServoConfig& servo, float percent)
{
    if (percent < MIN_PERCENT) percent = MIN_PERCENT;
    if (percent > MAX_PERCENT) percent = MAX_PERCENT;

    const float span_ticks  = servo.max_pulse_ticks - servo.min_pulse_ticks;
    const auto  width_ticks = static_cast<uint32_t>(
        servo.min_pulse_ticks + (percent / MAX_PERCENT) * span_ticks);
    setPulseWidth(servo, width_ticks);
}

} // namespace platform::actuation::servomotor
