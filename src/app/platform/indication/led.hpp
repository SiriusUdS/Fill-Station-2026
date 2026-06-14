#pragma once

#include <cstdint>

#include "stm32h7xx_hal.h"   // GPIO_TypeDef, GPIO_InitTypeDef, HAL_GPIO_*

#include "indication/interfaces/digital_out.hpp"   // logic::indication::DigitalOut

/* ------------------------------------------------------------------------- *
 * GPIO-backed status LED. A single push-pull output pin behind the logic
 * DigitalOut seam: init() configures the pin and drives it off; set() writes the
 * level honouring the configured polarity. The HAL/pin detail stays here; the
 * logic (RunningIndicator) only ever sees set(bool). One instance per LED; the
 * board supplies the port/pin, so the driver carries no board-specific pin
 * numbers and ports across boards.
 * ------------------------------------------------------------------------- */

namespace platform::indication {

class Led {
public:
    /** @brief Board wiring for the LED: which port/pin, and its active polarity. */
    struct Config {
        GPIO_TypeDef* port        = nullptr;  /**< GPIO port, e.g. GPIOF. */
        uint16_t      pin         = 0;        /**< GPIO pin, e.g. GPIO_PIN_1. */
        bool          active_high = true;     /**< true: on = pin high; false: on = pin low. */
    };

    Led() = default;

    /**
     * @brief  Configure the pin as a push-pull output and drive it off. The GPIO
     *         port clock must already be enabled (MX_GPIO_Init does this at
     *         bring-up). Call once before the first set().
     */
    void init(const Config& config)
    {
        cfg_ = config;

        GPIO_InitTypeDef gpio = {};
        gpio.Pin   = cfg_.pin;
        gpio.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(cfg_.port, &gpio);

        set(false);
    }

    /** @brief Drive the LED on/off, honouring the configured polarity. */
    void set(bool on)
    {
        const GPIO_PinState level = (on == cfg_.active_high) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(cfg_.port, cfg_.pin, level);
    }

private:
    Config cfg_{};
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::indication::DigitalOut<Led>);

} // namespace platform::indication
