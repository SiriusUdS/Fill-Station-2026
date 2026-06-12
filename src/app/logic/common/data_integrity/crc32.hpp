#pragma once

#include <cstddef>
#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Data-integrity CRC-32 seam (the zlib / reflected-0xEDB88320 variant).
 *
 * Firmware-internal — NOT part of the wire protocol (the ground station computes
 * its own CRC over the received bytes), so this lives outside communication/
 * protocol/. The polynomial it reproduces is the only GS-shared part and stays
 * in protocol/system/crc32_polynomial.hpp.
 *
 * This is a DECLARATION only: the definition is supplied by the platform DIL.
 *   - Firmware: the STM32 CRC peripheral (platform::data_integrity::crc), which
 *     must be initialised (crc::init) before the first call.
 *   - Host tests: a software definition in the test support.
 *
 * It produces the same value as persistent_state's early-boot software crc32
 * (which stays software because it runs before the CRC peripheral is up), so a
 * record CRC'd through this seam verifies identically on the ground station.
 * ------------------------------------------------------------------------- */

namespace logic::data_integrity {

/**
 * @brief  CRC-32 (zlib/Ethernet variant) over a byte span.
 * @param  data         First byte of the span.
 * @param  length_bytes Number of bytes to hash.
 * @return The 32-bit CRC.
 */
[[nodiscard]] uint32_t crc32(const uint8_t* data, std::size_t length_bytes);

} // namespace logic::data_integrity
