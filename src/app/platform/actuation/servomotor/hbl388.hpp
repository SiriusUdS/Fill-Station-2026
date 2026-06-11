#pragma once

#include <cstdint>

#include "stm32h7xx_hal.h"   // TIM_HandleTypeDef + TIM HAL API

/* ------------------------------------------------------------------------- *
 * HBL388 servo DIL (Driver Interface Layer).
 *
 * Drives a single PWM servo channel on a HAL timer. Platform-only: valves and
 * other actuators build on top of it; the logic layer never includes this.
 * ------------------------------------------------------------------------- */

namespace platform::actuation::servomotor {

/**
 * @brief Configuration of one PWM-driven servo channel.
 */
struct ServoConfig {
    TIM_HandleTypeDef* htim;             /**< Timer generating the servo PWM. */
    uint32_t           channel;          /**< Timer channel (TIM_CHANNEL_x). */
    float              min_pulse_ticks;  /**< PWM compare value (timer ticks) at 0%. */
    float              max_pulse_ticks;  /**< PWM compare value (timer ticks) at 100%. */
};

/**
 * @brief Start PWM generation on the servo's timer channel.
 * @param servo  The servo to start.
 */
void init(const ServoConfig& servo);

/**
 * @brief Set the raw PWM compare value (pulse width) directly.
 * @param servo        The servo to drive.
 * @param width_ticks  Compare value in timer ticks.
 */
void setPulseWidth(const ServoConfig& servo, uint32_t width_ticks);

/**
 * @brief Drive the servo to a fraction of its travel.
 * @param servo    The servo to drive.
 * @param percent  0..100; values outside the range are clamped.
 */
void setPercent(const ServoConfig& servo, float percent);

} // namespace platform::actuation::servomotor
