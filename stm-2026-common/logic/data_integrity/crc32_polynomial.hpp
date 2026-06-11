#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * CRC-32 polynomial — single source of truth.
 *
 * The value is the STM32H7 CRC peripheral's default polynomial (RM0433,
 * CRC_POL reset value): the standard CRC-32 polynomial in normal / MSB-first
 * form. Sharing it here lets the platform's hardware-CRC bring-up and any
 * software fallback agree on one polynomial.
 *
 * This logic-layer constant is the source of truth; the platform asserts the
 * hardware actually matches it. main.cpp (platform) carries a static_assert
 * that CRC32_POLYNOMIAL equals the HAL/register polynomial it configures the
 * CRC peripheral with, so the two can never silently drift apart.
 * ------------------------------------------------------------------------- */

namespace logic::data_integrity {

/** @brief CRC-32 polynomial, normal (MSB-first) form — STM32H7 hardware default. */
inline constexpr uint32_t CRC32_POLYNOMIAL = 0x04C11DB7U;

/** @brief Bit-reversed form, for LSB-first (reflected) software CRC loops. */
inline constexpr uint32_t CRC32_POLYNOMIAL_REFLECTED = 0xEDB88320U;

} // namespace logic::data_integrity
