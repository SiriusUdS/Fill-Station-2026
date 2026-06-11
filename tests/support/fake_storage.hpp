#pragma once

/* ------------------------------------------------------------------------- *
 * Host test double for the logic::storage::Storage contract.
 *
 * Injected into Controller<FakeStorage> in place of the SD card: captures every
 * write() block so tests can assert what was persisted, and exposes a scriptable
 * status() so tests can drive the storage-health telemetry flag. Because the
 * contract is structural, the controller template instantiates on this directly
 * — no separate link, no HAL/FatFs.
 * ------------------------------------------------------------------------- */

#include "storage/interfaces/storage.hpp"

#include "sirius-headers-common/Storage/StorageState.h"   // STORAGE_STATE_* (to script status)

#include <cstdint>
#include <span>
#include <vector>

/** @brief In-memory stand-in for the backing store (models logic::storage::Storage). */
struct FakeStorage {
    /* ---- inputs the test scripts ---- */
    StorageStatus status_value{};   /**< Returned verbatim by status(); script .bits.state. */

    /* ---- outputs the test inspects ---- */
    int                               init_calls = 0;
    std::vector<std::vector<uint8_t>> writes;   /**< One captured copy per write() block. */

    void init() { ++init_calls; }

    void write(std::span<const uint8_t> data)
    {
        writes.emplace_back(data.begin(), data.end());
    }

    [[nodiscard]] StorageStatus status() const { return status_value; }
};

// Same compile-time guarantee the SD driver carries: the double really models
// the contract the controller is written against.
static_assert(logic::storage::Storage<FakeStorage>);
