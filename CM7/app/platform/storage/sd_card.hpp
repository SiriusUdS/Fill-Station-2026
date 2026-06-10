#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "stm32h7xx_hal.h"   // SD_HandleTypeDef
#include "fatfs.h"           // FATFS, SDPath, f_* (FatFs middleware)

#include "sirius-headers-common/Storage/StorageState.h"   // STORAGE_STATE_*
#include "sirius-headers-common/Storage/StorageStatus.h"  // StorageStatus

/* ------------------------------------------------------------------------- *
 * FatFs-backed SD card driver. One instance per physical card: bind a HAL SD
 * handle and a logical drive, then init() mounts the volume and opens a log
 * file that stays open for the driver's lifetime. write() appends a record and
 * syncs (flushes) it, so each save is power-loss safe without the cost of
 * reopening the file. The FatFs/HAL detail stays in the .cpp; coarse status is
 * reported through the shared StorageStatus protocol, the detailed failure
 * cause through Error.
 * ------------------------------------------------------------------------- */

namespace platform::storage {

class SdCard {
public:
    /** @brief Detailed failure cause, beyond the coarse StorageStatus state. */
    enum class Error : uint8_t {
        None = 0,
        MountFail,
        FileOpenFail,
        FileWriteFail,
    };

    /**
     * @brief  Bind to a HAL SD handle and a FatFs logical drive (e.g. "0:/").
     *         Does not touch hardware. Each card owns its own drive and FATFS,
     *         so two instances share no state — they back each other up
     *         independently. @p drive must outlive the instance (a string
     *         literal or other static-lifetime buffer).
     */
    SdCard(SD_HandleTypeDef* handle, const char* drive);

    /**
     * @brief  Mount the volume and open the log file (kept open afterwards).
     *         Status becomes ACTIVE on success, ERROR otherwise.
     */
    void init();

    /**
     * @brief  Append @p data to the open file and sync it to the card. No-op
     *         unless the store is ready (init() succeeded).
     */
    void write(std::span<const uint8_t> data);

    /** @brief Coarse status (state + plugged-in) in the shared storage protocol. */
    StorageStatus status() const { return status_; }

    /** @brief Last detailed failure cause. */
    Error error() const { return error_; }

private:
    void fail(Error code);

    // Retained to identify the card; FatFs reaches the media through the linked
    // diskio driver, not this handle directly.
    SD_HandleTypeDef* handle_;
    const char*       drive_;   // FatFs logical drive, e.g. "0:/"
    FATFS             fs_{};
    FIL               file_{};  // log file, opened by init() and kept open
    StorageStatus     status_{};
    Error             error_ = Error::None;
};

} // namespace platform::storage
