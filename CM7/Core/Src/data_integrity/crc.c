#include "data_integrity/crc.h"

#define CRC_GENERATING_POLYNOMIAL (uint32_t)0x04C11DB7
#define CRC_INITIAL_VALUE         (uint32_t)0xFFFFFFFF

/**
 * @brief  Calculate CRC-32 for a uint8_t payload
 * @param  data     Pointer to uint8_t data buffer
 * @param  length   Number of bytes in the buffer
 * @retval CRC-32 value
 */
uint32_t CRC32_Calculate(const uint8_t *data, uint32_t length)
{
    uint32_t crc = CRC_INITIAL_VALUE;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= ((uint32_t)data[i] << 24);

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80000000UL)
                crc = (crc << 1) ^ CRC_GENERATING_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }

    return crc;
}