#pragma once

#include <concepts>
#include <cstdint>
#include <span>

#include "sirius-headers-common/Storage/StorageStatus.h"   // StorageStatus

/* ------------------------------------------------------------------------- *
 * Class-based storage contract for the logic layer (C++23 concept).
 *
 * Mirrors the valve/adc seams: the contract is a concept, a platform driver
 * (the SD card) models it, and a host fake models it for tests. No HAL/FatFs
 * type appears here.
 *
 * Storage is the one peripheral the controller writes to *synchronously*, but a
 * HAL-free, non-templated controller can't hold a concept-typed object. So the
 * integration mirrors the ADC: bring-up owns the modelling instance and connects
 * it to the controller through a registered callback (the persist sink), and
 * reports its status back. The concept still pins the driver/fake to one shape;
 * the controller talks to the sink rather than to a Storage directly.
 * ------------------------------------------------------------------------- */

namespace logic::storage {

/**
 * @brief The contract a backing store must satisfy.
 *
 * A conforming type exposes:
 *   - init()        — bring the store online (mount); sets status ACTIVE/ERROR.
 *   - write(data)   — persist a block; no-op unless the store is ready.
 *   - status()      — coarse state in the shared StorageStatus protocol.
 */
template <typename T>
concept Storage = requires(T store, std::span<const uint8_t> data) {
    { store.init() }       -> std::same_as<void>;
    { store.write(data) }  -> std::same_as<void>;
    { store.status() }     -> std::same_as<StorageStatus>;
};

} // namespace logic::storage
