#pragma once

#include <cstdint>

#include "peripherals/storage/storage_state.hpp"
#include "peripherals/storage/storage_status.hpp"

/* Telemetry unit for one backing store (SD card): its state and status bits,
 * owned and kept current by the store driver. The store has no per-record value
 * to report (unlike a valve's position or an ADC's channels), so the record is
 * just the State + Status pair. */

/** @brief One store's reported state + status. */
struct StorageInfo {
    StorageState  state;
    StorageStatus status;
};

// Wire layout guard: must be packed with no implicit padding.
static_assert(sizeof(StorageInfo) == 2, "StorageInfo must be exactly 2 bytes");
static_assert(sizeof(StorageInfo) == 2 * sizeof(uint8_t),
              "StorageInfo has implicit padding — keep it two packed bytes");
