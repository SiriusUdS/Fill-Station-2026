#pragma once

#include <cstdint>

/* Failure cause for a backing store, carried in StorageStatus so it is saved /
 * downlinked with the rest of the record (not a driver-private side channel).
 * Mutually exclusive causes — the store fails one operation at a time — so they
 * fit a 2-bit coded field rather than separate flags. */

/** @brief Why a store is in StorageState::Error (None while healthy). */
enum class StorageError : uint8_t {
    None          = 0x00,  /**< no failure */
    MountFail     = 0x01,  /**< f_mount failed */
    FileOpenFail  = 0x02,  /**< f_open of the log file failed */
    FileWriteFail = 0x03,  /**< f_write / f_sync of a record failed */
};

// Must fit the 2-bit StorageStatus::error field.
static_assert(static_cast<uint8_t>(StorageError::FileWriteFail) <= 0x03,
              "StorageError must fit in 2 bits (StorageStatus::error)");
