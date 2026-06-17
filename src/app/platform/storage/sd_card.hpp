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
 * a record and flushes (f_sync) it in BATCHES — every sync_period_ writes (set per file in
 * bind()) rather than every block — because the FAT/directory flush is the costly part of a save and
 * syncing every block stalls the producer enough to overrun the telemetry double buffer.
 * The first write is always synced so the file's directory entry is committed immediately.
 * The trade is that up to one sync period's records can be lost on a power cut. The
 * FatFs/HAL detail stays in the .cpp; the store owns its
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

    /** @brief Default batched-flush period: f_sync once every this many writes. The fast
     *         (high-rate) stream uses this; slower streams pass a smaller period to bind(). */
    static constexpr unsigned DEFAULT_SYNC_PERIOD_WRITES = 16;

    /**
     * @brief  Bind the HAL SD handle, FatFs drive and this stream's file name (e.g.
     *         "data_fast.bin"). @p drive and @p filename must outlive the instance
     *         (string literals / static-lifetime buffers). Does not touch hardware;
     *         init() mounts the volume and opens the file.
     * @param  sync_period_writes  f_sync once every this many writes (batched flush). Tune
     *         per stream: the high-rate data_fast.bin syncs rarely to avoid stalling the
     *         producer, while a low-rate stream can sync every write (pass 1) cheaply. A
     *         period of 0 is clamped to 1 (sync every write) so the file never goes unsynced.
     */
    void bind(SD_HandleTypeDef* handle, const char* drive, const char* filename,
              unsigned sync_period_writes = DEFAULT_SYNC_PERIOD_WRITES)
    {
        handle_      = handle;
        drive_       = drive;
        filename_    = filename;
        sync_period_ = sync_period_writes == 0u ? 1u : sync_period_writes;
    }

    /**
     * @brief  Ensure the volume is mounted and open this stream's log file (kept
     *         open afterwards). Status becomes ACTIVE on success, ERROR otherwise.
     */
    void init();

    /**
     * @brief  Append @p data to the open file, syncing to the card in batches (every
     *         sync_period_ writes, plus the first write). No-op unless the store
     *         is ready (init() succeeded) — a store that never mounted stays inert.
     *         This just saves whatever the telemetry pipeline hands it.
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
    unsigned          sync_period_       = DEFAULT_SYNC_PERIOD_WRITES; // f_sync every this many writes (per stream)
    unsigned          writes_since_sync_ = 0;     // writes accrued since the last f_sync (batched flush)
    bool              first_sync_done_   = false; // the first write force-syncs to commit the dir entry
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::storage::Storage<SdCard>);

} // namespace platform::storage
