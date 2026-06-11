#pragma once

#include <cstdint>
#include <optional>

#include "stm32h7xx_hal.h"                   // GPIO_TypeDef + GPIO HAL API
#include "actuation/servomotor/hbl388.hpp"   // ServoConfig
#include "actuation/interfaces/valve.hpp"    // logic::actuation::Valve contract

/* ------------------------------------------------------------------------- *
 * Ball valve platform driver.
 *
 * Each BallValve is ONE physical valve: it owns its servo, its limit switches
 * and its transition/timeout state machine, and models the logic-side seam
 * declared in actuation/interfaces/valve.hpp (logic::actuation::Valve). A board
 * holds one instance per valve (e.g. std::array<BallValve, N>) and ticks each;
 * there is no hidden id-indexed table.
 *
 * open()/close()/setOpenPercent()/state() are the HAL-free contract the logic
 * layer commands through. init() (servo/GPIO config) and tick() (sampling the
 * switches + advancing the timeout) are the HAL-coupled, platform-only members.
 * ------------------------------------------------------------------------- */

namespace platform::actuation::valve {

/**
 * @brief A limit switch wired to a GPIO input.
 */
struct LimitSwitchConfig {
    GPIO_TypeDef* port;  /**< GPIO port of the switch. */
    uint16_t      pin;   /**< GPIO pin of the switch. */
};

/**
 * @brief Static configuration of one ball valve.
 */
struct BallValveConfig {
    servomotor::ServoConfig servo;            /**< Servo actuating the ball. */
    LimitSwitchConfig       open_limit;       /**< Switch asserted when fully open. */
    LimitSwitchConfig       close_limit;      /**< Switch asserted when fully closed. */
    uint32_t                max_transit_timeout_ms;  /**< Open/close time before declaring Fault. */
    bool                    opened_switch_ignored = false;  /**< No physical open switch: open() floats at 100 %. */
};

/**
 * @brief One servo-actuated ball valve with limit switches and a transition
 *        state machine. Models @ref logic::actuation::Valve.
 */
class BallValve {
public:
    /** @brief Construct an unconfigured valve (state Unknown); call init() before use. */
    BallValve() = default;

    /**
     * @brief  Bind the valve's static configuration and start its servo PWM.
     * @param  config  Servo, limit switches and timeout for this valve.
     */
    void init(const BallValveConfig& config);

    /* ---- logic::actuation::Valve contract (HAL-free to the caller) -------- */

    /**
     * @brief  Command the valve to its fully-open position.
     * @return std::nullopt on success, or a ValveError describing the failure.
     */
    [[nodiscard]] std::optional<logic::actuation::ValveError> open();

    /**
     * @brief  Command the valve to its fully-closed position.
     * @return std::nullopt on success, or a ValveError describing the failure.
     */
    [[nodiscard]] std::optional<logic::actuation::ValveError> close();

    /**
     * @brief  Command the valve to a proportional open position.
     * @param  percent  Desired opening, 0 (closed) to 100 (open); clamped.
     * @return std::nullopt on success, or a ValveError describing the failure.
     */
    [[nodiscard]] std::optional<logic::actuation::ValveError> setOpenPercent(float percent);

    /**
     * @brief  The valve's own info record (state + status + commanded position),
     *         kept up to date by the valve as it operates.
     */
    [[nodiscard]] ValveInfo info() const { return info_; }

    /* ---- platform-only (not part of the contract) ------------------------ */

    /**
     * @brief  Service the valve: sample the limit switches into info_ and advance
     *         the opening/closing/timeout state machine. Call periodically.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void tick(uint32_t now_ms);

private:
    BallValveConfig config_{};
    ValveInfo       info_{};               /**< State + status + commanded position; see info(). */
    uint32_t        start_movement_ms_ = 0;  /**< When the current open/close began (timeout base). */
    uint32_t        end_movement_ms_   = 0;  /**< When the valve last reached a limit / faulted. */
};

// The driver is the logic seam: enforce conformance at compile time, here, so a
// contract drift is caught in the platform layer rather than at a logic call.
static_assert(logic::actuation::Valve<BallValve>);

} // namespace platform::actuation::valve
