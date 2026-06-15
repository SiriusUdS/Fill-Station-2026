#pragma once

#include <cstdint>

#include "stm32h7xx_hal.h"   // GPIO_TypeDef, GPIO_InitTypeDef, HAL_GPIO_*

#include "sensing/interfaces/digital_in.hpp"   // logic::sensing::DigitalIn

/* ------------------------------------------------------------------------- *
 * GPIO-backed digital input behind the logic DigitalIn seam — the input sibling
 * of platform::gpio::DigitalOutput. init() configures the pin as an input with
 * the requested pull; read() samples it and maps the level to active/inactive per
 * the configured polarity. The HAL/pin detail stays here; the logic only ever sees
 * read() -> bool. One instance per line; the board supplies the port/pin/polarity.
 * ------------------------------------------------------------------------- */

namespace platform::gpio {

class DigitalInput {
public:
    /** @brief Board wiring for the input: which port/pin, its active polarity, and pull. */
    struct Config {
        GPIO_TypeDef* port        = nullptr;        /**< GPIO port, e.g. GPIOD. */
        uint16_t      pin         = 0;              /**< GPIO pin, e.g. GPIO_PIN_13. */
        bool          active_high = true;           /**< true: active = pin high; false: active = pin low. */
        uint32_t      pull        = GPIO_NOPULL;    /**< GPIO_NOPULL / GPIO_PULLUP / GPIO_PULLDOWN. */
    };

    DigitalInput() = default;

    /**
     * @brief  Configure the pin as an input with the requested pull. The GPIO port clock
     *         must already be enabled (MX_GPIO_Init does this at bring-up). Call once
     *         before the first read().
     */
    void init(const Config& config)
    {
        cfg_ = config;

        GPIO_InitTypeDef gpio = {};
        gpio.Pin   = cfg_.pin;
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = cfg_.pull;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(cfg_.port, &gpio);
    }

    /** @brief Sample the line; true when ACTIVE per the configured polarity. */
    [[nodiscard]] bool read() const
    {
        const bool high = HAL_GPIO_ReadPin(cfg_.port, cfg_.pin) == GPIO_PIN_SET;
        return high == cfg_.active_high;
    }

private:
    Config cfg_{};
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::sensing::DigitalIn<DigitalInput>);

} // namespace platform::gpio
