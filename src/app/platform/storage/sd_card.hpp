#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "stm32h7xx_hal.h"   // SD_HandleTypeDef
#include "fatfs.h"           // FATFS, SDPath, f_* (FatFs middleware)

#include "storage/interfaces/storage.hpp"   // logic::storage::Storage contract + StorageInfo

/* ------------------------------------------------------------------------- *
 * FatFs-backed SD card log file. One instance per LOG FILE (not per card): bind a
 * HAL SD handle, a logical drive and a file name, then init() ensures the volume is
 * mounted and opens that file, kept open for the driver's lifetime. write() appends
 * a record and syncs (flushes) it, so each save is power-loss safe without the cost
 * of reopening the file. The FatFs/HAL detail stays in the .cpp; the store owns its
 * StorageInfo (state + status), exposed through info(), and reports the failure
 * cause through Error.
 *
 * A single physical card can carry several streams in separate files (e.g. the FCU
 * writes the 2 kHz data_fast.bin and the 10 Hz data_slow.bin). Both live inside a
 * fresh per-boot session folder (the highest existing numeric folder + 1), created
 * once via beginSession() — shared by every SdCard on that card — and each instance
 * opens its own file inside it; the mount + folder are NOT per-instance because two
 * FATFS objects on one drive would collide.
 * ------------------------------------------------------------------------- */

namespace platform::storage {

/**
 * @brief  Mount @p drive's FAT volume and create this boot's session folder (the
 *         highest existing numeric folder + 1) the first time it is called;
 *         idempotent afterwards. Every SdCard on the same physical card shares this
 *         single mount + folder and opens its own file inside it. Called by
 *         SdCard::init(); also safe to call directly from board bring-up.
 * @return The session folder path prefix (e.g. "0:/7/"), or nullptr on failure.
 */
const char* beginSession(const char* drive);

class SdCard {
public:
    /** @brief  Default-construct unbound; call bind() before init(). Lets the app
     *          composition declare the file without naming a board HAL handle. */
    SdCard() = default;

    /**
     * @brief  Bind the HAL SD handle, FatFs drive and this stream's file name (e.g.
     *         "data_fast.bin"). @p drive and @p filename must outlive the instance
     *         (string literals / static-lifetime buffers). Does not touch hardware;
     *         init() mounts the volume and opens the file.
     */
    void bind(SD_HandleTypeDef* handle, const char* drive, const char* filename)
    {
        handle_   = handle;
        drive_    = drive;
        filename_ = filename;
    }

    /**
     * @brief  Ensure the volume is mounted and open this stream's log file (kept
     *         open afterwards). Status becomes ACTIVE on success, ERROR otherwise.
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
    const char*       drive_{};     // FatFs logical drive, e.g. "0:/"
    const char*       filename_{};  // log file name on the volume, e.g. "data_fast.bin"
    FIL               file_{};      // log file, opened by init() and kept open
    StorageInfo       info_{};      // state + status (incl. last error cause); see info()
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::storage::Storage<SdCard>);

} // namespace platform::storage
