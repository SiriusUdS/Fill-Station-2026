/* ------------------------------------------------------------------------- *
 * Host definition of the data-integrity CRC-32 seam (logic::data_integrity::
 * crc32). On firmware this is the STM32 CRC peripheral (platform DIL); in host
 * tests there is no peripheral, so we provide the software reference — the same
 * zlib / reflected-0xEDB88320 variant the hardware is configured to reproduce.
 * ------------------------------------------------------------------------- */

#include "data_integrity/crc32.hpp"
#include "system/crc32_polynomial.hpp"   // CRC32_POLYNOMIAL_REFLECTED

uint32_t logic::data_integrity::crc32(const uint8_t* data, std::size_t length_bytes)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < length_bytes; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = ~(crc & 1U) + 1U;
            crc = (crc >> 1) ^ (logic::data_integrity::CRC32_POLYNOMIAL_REFLECTED & mask);
        }
    }
    return ~crc;
}
