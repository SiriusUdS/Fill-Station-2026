#pragma once

#include <concepts>
#include <cstdint>
#include <span>

#include "communication/protocol/peripherals/storage/storage_info.hpp"          // StorageInfo (the store's own info record)
#include "communication/protocol/peripherals/storage/sd_write_engine_info.hpp"  // SdWriteEngineInfo (shared write-engine health)

/* ------------------------------------------------------------------------- *
 * Class-based storage contract for the logic layer (C++23 concept).
 *
 * Mirrors the valve/adc seams: the contract is a concept, a platform driver
 * (the SD card) models it, and a host fake models it for tests. No HAL/FatFs
 * type appears here, and the store reports its health through the same
 * State/Status info() record every peripheral uses.
 * ------------------------------------------------------------------------- */

namespace logic::storage {

/**
 * @brief The contract a backing store must satisfy.
 *
 * A conforming type exposes:
 *   - init()        — bring the store online (mount); sets state Active/Error.
 *   - write(data)   — persist a block; no-op unless the store is ready. The store
 *                     just saves whatever the telemetry pipeline hands it.
 *   - finalize()    — clean-shutdown close: flush/commit and release any unused
 *                     pre-allocated space (end-of-run). A no-op for stores that
 *                     have nothing to reclaim.
 *   - info()        — the store's own StorageInfo (state + status), kept current.
 *   - engineInfo()  — the board-wide write-engine health behind this store (dropped
 *                     blocks + sticky DMA error). Shared by every store on the card, so
 *                     each reports the same value; the recorder surfaces it once.
 */
template <typename T>
concept Storage = requires(T store, std::span<const uint8_t> data) {
    { store.init() }         -> std::same_as<void>;
    { store.write(data) }    -> std::same_as<void>;
    { store.finalize() }     -> std::same_as<void>;
    { store.info() }         -> std::same_as<::StorageInfo>;
    { store.engineInfo() }   -> std::same_as<::SdWriteEngineInfo>;
};

} // namespace logic::storage
