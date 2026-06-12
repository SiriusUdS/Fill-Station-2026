/* ------------------------------------------------------------------------- *
 * Platform CRC DIL: the STM32H7 CRC peripheral behind the logic-layer
 * data-integrity seam (logic::data_integrity::crc32).
 *
 * The CubeMX-generated config (crc.c) sets the unit up non-reflected with no
 * output inversion. We override that in init() to the zlib / reflected CRC-32
 * variant so the hardware result equals the software reference byte-for-byte:
 *   - default polynomial 0x04C11DB7, default init 0xFFFFFFFF,
 *   - input data byte-reversed, output data bit-reversed,
 *   - final XOR with 0xFFFFFFFF applied in software (the unit has no xor-out).
 * ------------------------------------------------------------------------- */

#include "data_integrity/crc/crc.hpp"
#include "data_integrity/crc32.hpp"   // the logic seam we define here

namespace {

/* The board's single CRC unit, bound by init(). */
CRC_HandleTypeDef* g_hcrc = nullptr;

} // namespace

namespace platform::data_integrity::crc {

void init(CRC_HandleTypeDef* hcrc)
{
    g_hcrc = hcrc;

    // Reconfigure for the reflected zlib variant (CubeMX generates the
    // non-reflected one). We own this here, not in crc.c, so the variant lives
    // in one place alongside the code that depends on it.
    hcrc->Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;     // 0x04C11DB7
    hcrc->Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;     // 0xFFFFFFFF
    hcrc->Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_BYTE;  // refin
    hcrc->Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE; // refout
    hcrc->InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;
    (void)HAL_CRC_Init(hcrc);
}

} // namespace platform::data_integrity::crc

// ---- The logic-layer data-integrity seam, backed by the hardware unit -------
uint32_t logic::data_integrity::crc32(const uint8_t* data, std::size_t length_bytes)
{
    // HAL_CRC_Calculate seeds the unit with the init value each call (independent,
    // not accumulating). With INPUTDATA_FORMAT_BYTES the buffer is read byte-wise,
    // so alignment of `data` does not matter. The unit has no xor-out, so apply
    // the zlib final XOR (0xFFFFFFFF) here.
    const uint32_t raw = HAL_CRC_Calculate(
        g_hcrc,
        reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(data)),
        static_cast<uint32_t>(length_bytes));
    return raw ^ 0xFFFFFFFFU;
}
