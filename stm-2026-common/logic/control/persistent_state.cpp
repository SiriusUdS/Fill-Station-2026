/**
  ******************************************************************************
  * @file    persistent_state.cpp
  * @brief   Self-validating, Backup-SRAM-resident snapshot of the global state
  *          machine. HAL-free: it only reads/writes its own bytes and computes
  *          a CRC. Where the live instance physically lives (the Backup SRAM
  *          section) and the backup-domain bring-up are the platform's job.
  ******************************************************************************
  */

#include "control/persistent_state.hpp"

#include "data_integrity/crc32_polynomial.hpp"  // CRC32_POLYNOMIAL[_REFLECTED]

#include <cstddef>

namespace logic::control {

/* The single persistent-state instance. On the firmware target it is placed in
   the `.backup_sram` linker section (mapped to battery-backed Backup SRAM,
   NOLOAD) so it survives a reset; PersistentState is a trivial aggregate, so no
   constructor runs at startup to overwrite the retained value. On the host
   (unit tests) the attribute is dropped and it is ordinary zero-initialised
   memory — i.e. an invalid (cold) blob until the first saveState(). */
#if defined(__arm__)
__attribute__((section(".backup_sram")))
#endif
PersistentState persistent_state;

namespace {

using logic::data_integrity::CRC32_POLYNOMIAL_REFLECTED;

/**
 * @brief  CRC-32 over a byte span (reflected poly, zlib/Ethernet variant).
 * @note   This is a SOFTWARE implementation on purpose. validateLoadedState()
 *         and saveState() run during very early boot — before the platform
 *         brings up the STM32 hardware CRC peripheral — so this code cannot use
 *         it. Once the hardware CRC is initialised, later callers may switch to
 *         it; to reproduce this exact result it must be configured with
 *         CRC32_POLYNOMIAL plus input/output bit reversal to match this
 *         reflected variant. Same variant as the FCU controller's frame CRC.
 */
uint32_t crc32(const uint8_t* data, std::size_t length_bytes)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < length_bytes; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = ~(crc & 1U) + 1U;
            crc = (crc >> 1) ^ (CRC32_POLYNOMIAL_REFLECTED & mask);
        }
    }
    return ~crc;
}

} // namespace

uint32_t PersistentState::computeCrc() const
{
    // Covers every byte before `crc`: magic + fill_state + reserved.
    return crc32(reinterpret_cast<const uint8_t*>(this), offsetof(PersistentState, crc));
}

bool PersistentState::validateLoadedState() const
{
    if (magic != PERSISTENT_STATE_MAGIC) {
        return false;  // cold boot or wrong/garbage memory — not ours.
    }
    return crc == computeCrc();  // intact only if the payload still hashes the same.
}

std::optional<State> PersistentState::loadState() const
{
    if (!validateLoadedState()) {
        return std::nullopt;
    }
    return fill_state;
}

void PersistentState::saveState(State state)
{
    fill_state  = state;
    reserved[0] = 0;  // keep padding deterministic so the CRC is reproducible.
    reserved[1] = 0;
    reserved[2] = 0;
    magic       = PERSISTENT_STATE_MAGIC;
    crc         = computeCrc();  // committed LAST: a torn write fails validation.
}

} // namespace logic::control
