#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "stm32h7xx_hal.h"   // SD_HandleTypeDef
#include "fatfs.h"           // FATFS, SDPath, f_* (FatFs middleware)

#include "storage/interfaces/storage.hpp"   // logic::storage::Storage contract + StorageInfo

/* ------------------------------------------------------------------------- *
 * FatFs-backed SD card driver. One instance per physical card: bind a HAL SD
 * handle and a logical drive, then init() mounts the volume and opens a log
 * file that stays open for the driver's lifetime. write() appends a record and
 * syncs (flushes) it, so each save is power-loss safe without the cost of
 * reopening the file. The FatFs/HAL detail stays in the .cpp; the store owns its
 * StorageInfo (state + status), exposed through info(), and reports the detailed
 * failure cause through Error.
 * ------------------------------------------------------------------------- */

namespace platform::storage {

class SdCard {
public:
    /**
     * @brief  Bind to a HAL SD handle and a FatFs logical drive (e.g. "0:/").
     *         Does not touch hardware. Each card owns its own drive and FATFS,
     *         so two instances share no state — they back each other up
     *         independently. @p drive must outlive the instance (a string
     *         literal or other static-lifetime buffer).
     */
    SdCard(SD_HandleTypeDef* handle, const char* drive);

    /** @brief  Default-construct unbound; call bind() before init(). Lets the app
     *          composition declare the card without naming a board HAL handle. */
    SdCard() = default;

    /**
     * @brief  Bind the HAL SD handle + FatFs drive after construction, so the
     *         board layer owns the handle name and the app composition stays
     *         handle-free. @p drive must outlive the instance. Does not touch
     *         hardware; init() mounts.
     */
    void bind(SD_HandleTypeDef* handle, const char* drive) { handle_ = handle; drive_ = drive; }

    /**
     * @brief  Mount the volume and open the log file (kept open afterwards).
     *         Status becomes ACTIVE on success, ERROR otherwise.
     */
    void init();

    /**
     * @brief  Append @p data to the open file and sync it to the card. No-op
     *         unless the store is ready (init() succeeded) — a store that never
     *         mounted stays inert. Whether records *should* be persisted at all
     *         (the PersistingData control flag) is decided upstream in the
     *         telemetry pipeline; this just saves whatever it is handed.
     */
    void write(std::span<const uint8_t> data);

    /** @brief The store's own info record: state + status (incl. last error cause). */
    StorageInfo info() const { return info_; }

private:
    void fail(StorageError code);

    // Retained to identify the card; FatFs reaches the media through the linked
    // diskio driver, not this handle directly.
    SD_HandleTypeDef* handle_{};
    const char*       drive_{};   // FatFs logical drive, e.g. "0:/"
    FATFS             fs_{};
    FIL               file_{};  // log file, opened by init() and kept open
    StorageInfo       info_{};  // state + status (incl. last error cause); see info()
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::storage::Storage<SdCard>);

} // namespace platform::storage
