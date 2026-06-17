#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "stm32h7xx_hal.h"   // SD_HandleTypeDef
#include "fatfs.h"           // FATFS, SDPath, f_* (FatFs middleware)

#include "storage/interfaces/storage.hpp"   // logic::storage::Storage contract + StorageInfo
#include "storage/sd_write_engine.hpp"      // the shared async write engine + SD_WRITE_BLOCK_BYTES

/* ------------------------------------------------------------------------- *
 * Pre-allocated, raw-sector SD log file (the level-3 datalogger). Each instance is one LOG FILE
 * (not one card): bind a HAL SD handle, a logical drive, a file name and a pre-allocation size,
 * then init() mounts the shared volume, opens the file and f_expands it to a fixed CONTIGUOUS
 * region — so the whole FAT/directory cost is paid once, up front. The file's absolute start
 * sector (LBA) is then resolved from the cluster chain, and write() simply DMAs each whole
 * sector-aligned block to the next sector through the shared SdWriteEngine, bumping a cursor.
 *
 * Why: the old design appended through f_write + a batched f_sync, and the FAT/dir flush is the
 * costly, latency-spiky part of a save — it stalled the foreground and overran the telemetry
 * ring. Here there is NO f_write and NO f_sync on the hot path: the FAT was written by f_expand
 * at init, and writes are fire-and-forget DMA into the pre-allocated extent. The foreground never
 * blocks on the card. finalize() (clean shutdown) truncates the file back to the bytes actually
 * written, returning the unused pre-allocated tail to the volume; a power-cut instead leaves the
 * file at full size, recoverable via the per-block footer/CRC that marks the true end-of-data.
 *
 * One physical card carries several streams in separate files (data_fast/slow/ext), all sharing
 * the single mounted volume + per-boot session folder (beginSession) and the single SdWriteEngine
 * that arbitrates the one SDMMC peripheral. The HAL/FatFs detail stays in the .cpp; the store owns
 * its StorageInfo (state + status, incl. last error cause), exposed through info().
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

    /** @brief Default pre-allocation for a stream's contiguous region. Sized for a generous
     *         worst-case run; the unused tail is reclaimed by finalize() on a clean shutdown.
     *         Tune per stream in bind() — the fast stream wants far more than the slow/ext ones. */
    static constexpr uint32_t DEFAULT_PREALLOC_BYTES = 64u * 1024u * 1024u;  // 64 MiB

    /**
     * @brief  Bind the HAL SD handle, FatFs drive and this stream's file name (e.g.
     *         "data_fast.bin"). @p drive and @p filename must outlive the instance
     *         (string literals / static-lifetime buffers). Does not touch hardware;
     *         init() mounts the volume, opens and pre-allocates the file.
     * @param  prealloc_bytes  contiguous region to reserve for this run with f_expand. Rounded
     *         down to a whole block; must exceed what the stream will write in the run (writes
     *         stop, not grow, when the region fills). The tail is reclaimed by finalize().
     */
    void bind(SD_HandleTypeDef* handle, const char* drive, const char* filename,
              uint32_t prealloc_bytes = DEFAULT_PREALLOC_BYTES)
    {
        handle_         = handle;
        drive_          = drive;
        filename_       = filename;
        prealloc_bytes_ = prealloc_bytes;
    }

    /**
     * @brief  Ensure the volume is mounted, open this stream's log file and f_expand it to a
     *         fixed contiguous region, then resolve its absolute start sector. Status becomes
     *         ACTIVE on success, ERROR otherwise. The shared SdWriteEngine must be init()'d first.
     */
    void init();

    /**
     * @brief  Hand one whole sector-aligned block (SD_WRITE_BLOCK_BYTES) to the async write
     *         engine, targeted at the next sector of the pre-allocated region, and advance the
     *         cursor. Non-blocking: copies into the engine ring and returns. No-op unless the
     *         store is ready; silently stops once the region is full (the run outran its
     *         pre-allocation) — surfaced via the engine's overrun count / this store's state.
     */
    void write(std::span<const uint8_t> data);

    /**
     * @brief  Clean-shutdown finalize: wait for the engine to drain, then truncate the file to
     *         the bytes actually written (freeing the unused pre-allocated tail) and f_sync the
     *         new size. Optional — skipping it (e.g. on a power-cut) just leaves the file at full
     *         pre-alloc size, still readable via the per-block footers.
     */
    void finalize();

    /** @brief The store's own info record: state + status (incl. last error cause). */
    StorageInfo info() const { return info_; }

    /** @brief The board-wide async write engine's health (dropped-block count + sticky DMA
     *         error). Read from the single shared SdWriteEngine, so every file on the card
     *         reports the same value; the recorder surfaces it once on the extended record. */
    SdWriteEngineInfo engineInfo() const
    {
        const SdWriteEngine& engine = sd_write_engine();
        return SdWriteEngineInfo{ engine.overrun_count(),
                                  static_cast<uint8_t>(engine.errored() ? 1u : 0u),
                                  /*reserved=*/0u };
    }

private:
    void fail(StorageError code);

    // Retained to identify the card; FatFs reaches the media through the linked
    // diskio driver, and the engine drives the same handle for DMA writes.
    SD_HandleTypeDef* handle_{};
    const char*       drive_{};     // FatFs logical drive, e.g. "0:/"
    const char*       filename_{};  // log file name on the volume, e.g. "data_fast.bin"
    uint32_t          prealloc_bytes_ = DEFAULT_PREALLOC_BYTES;  // contiguous region reserved at init()
    FIL               file_{};      // log file, opened + pre-allocated by init(), kept open
    uint32_t          base_lba_ = 0;          // absolute first card sector of the pre-allocated extent
    uint32_t          capacity_sectors_ = 0;  // pre-allocated length, in 512-byte sectors
    uint32_t          sector_cursor_ = 0;     // next sector to write, as an offset from base_lba_
    StorageInfo       info_{};      // state + status (incl. last error cause); see info()
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::storage::Storage<SdCard>);

} // namespace platform::storage
