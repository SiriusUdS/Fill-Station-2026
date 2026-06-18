#pragma once

#include "stm32h7xx_hal.h"   // IWDG_HandleTypeDef + IWDG HAL API

/* ------------------------------------------------------------------------- *
 * Platform watchdog DIL (Driver Interface Layer).
 *
 * Backs the logic-layer watchdog feed seam (logic::control::watchdog::kick) with
 * the STM32H7 independent watchdog (IWDG). The board has exactly one IWDG, so the
 * handle is file-static in the .cpp and the seam is a plain free function over it.
 *
 * MX_IWDG_Init() (CubeMX) both creates AND starts the watchdog; init() here just
 * binds the handle so kick() can refresh it. Call it once after MX_IWDG_Init.
 * ------------------------------------------------------------------------- */

namespace platform::system::watchdog {

/**
 * @brief  Bind the IWDG peripheral so the logic kick() seam can refresh it.
 * @param  hiwdg  IWDG handle (e.g. &hiwdg1), already started by MX_IWDG_Init.
 */
void init(IWDG_HandleTypeDef* hiwdg);

} // namespace platform::system::watchdog
