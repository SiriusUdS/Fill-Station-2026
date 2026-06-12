#pragma once

#include "stm32h7xx_hal.h"   // CRC_HandleTypeDef + CRC HAL API

/* ------------------------------------------------------------------------- *
 * Platform CRC DIL (Driver Interface Layer).
 *
 * Backs the logic-layer data-integrity seam (logic::data_integrity::crc32) with
 * the STM32H7 CRC peripheral. The board has exactly one CRC unit, so the handle
 * is file-static in the .cpp and the seam is a plain free function over it.
 *
 * init() is the HAL-coupled bring-up: it (re)configures the unit for the zlib /
 * reflected CRC-32 variant — input byte-reversal + output bit-reversal — so the
 * hardware reproduces the exact value of the software reference (the same variant
 * persistent_state computes early in boot, and that the ground station verifies).
 * Call it once after MX_CRC_Init and before the first crc32() call.
 * ------------------------------------------------------------------------- */

namespace platform::data_integrity::crc {

/**
 * @brief  Bind + configure the CRC peripheral for the zlib/reflected CRC-32 variant.
 * @param  hcrc  CRC peripheral handle (e.g. &hcrc), already created by MX_CRC_Init.
 */
void init(CRC_HandleTypeDef* hcrc);

} // namespace platform::data_integrity::crc
