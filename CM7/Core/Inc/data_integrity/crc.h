#pragma once
#include <stdint.h>

/**
 * @brief  Calculate CRC-32 for a uint8_t payload
 * @param  data     Pointer to uint8_t data buffer
 * @param  length   Number of bytes in the buffer
 * @retval CRC-32 value
 */
uint32_t CRC32_Calculate(const uint8_t *data, uint32_t length);