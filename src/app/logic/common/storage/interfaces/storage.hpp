#pragma once

#include <concepts>
#include <cstdint>
#include <span>

#include "communication/protocol/peripherals/storage/storage_info.hpp"   // StorageInfo (the store's own info record)

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
 *   - write(data)   — persist a block; no-op unless the store is ready. The decision
 *                     of *whether* to persist (the PersistingData control flag) is the
 *                     caller's (the telemetry pipeline) — the store just saves when told.
 *   - info()        — the store's own StorageInfo (state + status), kept current.
 */
template <typename T>
concept Storage = requires(T store, std::span<const uint8_t> data) {
    { store.init() }       -> std::same_as<void>;
    { store.write(data) }  -> std::same_as<void>;
    { store.info() }       -> std::same_as<::StorageInfo>;
};

} // namespace logic::storage
