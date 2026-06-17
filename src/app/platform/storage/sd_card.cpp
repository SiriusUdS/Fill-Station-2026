/**
 ******************************************************************************
 * @file    storage/sd_card.cpp
 * @brief   Pre-allocated, raw-sector SD log file. The first init() mounts the volume and
 *          creates a fresh per-boot session folder (the highest existing numeric folder + 1),
 *          both shared across every file on the card. Each instance then opens one log file
 *          inside that folder and f_expands it to a fixed CONTIGUOUS region, paying the whole
 *          FAT/directory cost once. write() then DMAs each whole block to the next sector of
 *          that extent through the shared SdWriteEngine — no f_write, no f_sync, no foreground
 *          stall. finalize() truncates the file back to the bytes written on a clean shutdown.
 *          The store owns its StorageInfo (state + status, carrying the last error cause).
 ******************************************************************************
 */

#include "storage/sd_card.hpp"

#include <cstdio>   // std::snprintf (build the drive-relative file path)

namespace platform::storage {

namespace {

/* The single mounted FAT volume + this boot's session folder, shared by every
   SdCard on this board (one physical card). The mount and folder are created once
   (by the first SdCard to init()); each SdCard then opens its own file inside the
   session folder. One static volume per firmware binary is enough — each board has
   exactly one card. */
FATFS s_volume_fs;
bool  s_session_ready      = false;
char  s_session_dir[24]    = {};   // this boot's folder, e.g. "0:/7/"

/* Highest existing numeric folder name in the volume root, + 1 (0 if none). Folder
   names are short numerics, so the 8.3 fname carries them regardless of LFN. */
int nextSessionNumber(const char* drive)
{
    DIR     dir;
    FILINFO fno;
    int     max = -1;

    if (f_opendir(&dir, drive) != FR_OK) {
        return 0;
    }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        if (!(fno.fattrib & AM_DIR)) {
            continue;
        }
        int  value     = 0;
        bool all_digits = true;
        for (const char* p = fno.fname; *p != '\0'; ++p) {
            if (*p < '0' || *p > '9') { all_digits = false; break; }
            value = value * 10 + (*p - '0');
        }
        if (all_digits && value > max) {
            max = value;
        }
    }
    f_closedir(&dir);
    return max + 1;
}

} // namespace

const char* beginSession(const char* drive)
{
    if (s_session_ready) {
        return s_session_dir;   // mounted + folder created by the first SdCard to init()
    }
    if (f_mount(&s_volume_fs, drive, 1) != FR_OK) {
        return nullptr;
    }

    // A fresh folder per boot: highest existing numeric folder + 1, e.g. "0:/7".
    const int n = nextSessionNumber(drive);
    char created[24];
    std::snprintf(created, sizeof(created), "%s%d", drive, n);
    if (f_mkdir(created) != FR_OK) {
        return nullptr;
    }

    std::snprintf(s_session_dir, sizeof(s_session_dir), "%s%d/", drive, n);
    s_session_ready = true;
    return s_session_dir;
}

void SdCard::init()
{
    info_.state = StorageState::Init;

    // Mount the volume and create/locate this boot's session folder (shared, once).
    const char* session = beginSession(drive_);
    if (session == nullptr) {
        fail(StorageError::MountFail);
        return;
    }

    // This stream's file inside the session folder, e.g. "0:/7/data_fast.bin". CREATE_ALWAYS
    // truncates it to empty, which f_expand requires. Kept open for the driver's lifetime.
    char path[40];
    std::snprintf(path, sizeof(path), "%s%s", session, filename_);
    if (f_open(&file_, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        fail(StorageError::FileOpenFail);
        return;
    }

    // Reserve a fixed CONTIGUOUS region up front (rounded down to whole blocks), so the FAT +
    // directory entry are written once, here — and every later write is a raw-sector DMA into the
    // extent with no FAT growth. opt=1 allocates now (not lazily). FR_DENIED means there is no
    // contiguous free run this large (a fragmented / near-full card).
    const uint32_t blocks   = prealloc_bytes_ / SD_WRITE_BLOCK_BYTES;
    const FSIZE_t  reserve  = static_cast<FSIZE_t>(blocks) * SD_WRITE_BLOCK_BYTES;
    if (blocks == 0 || f_expand(&file_, reserve, 1) != FR_OK) {
        fail(StorageError::FileWriteFail);
        return;
    }

    // Resolve the absolute first card sector (LBA) of the contiguous extent from its start
    // cluster: data-area base + (start cluster - 2) * sectors-per-cluster. The engine writes
    // sequential sectors from here, so no further FatFs lookups are needed during the run.
    const FATFS* fs   = file_.obj.fs;
    base_lba_         = (file_.obj.sclust - 2u) * fs->csize + fs->database;
    capacity_sectors_ = blocks * SECTORS_PER_BLOCK;
    sector_cursor_    = 0;

    info_.status.initialized = 1u;             // volume mounted, session folder + file open + reserved
    info_.status.error       = StorageError::None;
    info_.state              = StorageState::Active;
}

void SdCard::write(std::span<const uint8_t> data)
{
    if (info_.state != StorageState::Active) {
        return;  // not ready (init() failed or never ran)
    }
    // The pipeline always hands whole sector-aligned blocks; a short/over-long span is a caller
    // bug (it would misalign the raw-sector cursor), so refuse it rather than corrupt the extent.
    if (data.size() != SD_WRITE_BLOCK_BYTES) {
        fail(StorageError::FileWriteFail);
        return;
    }
    // Out of pre-allocated room: the run outran its reservation. Stop writing this stream (we
    // cannot grow without re-entering FatFs, which is exactly the stall we removed). Records the
    // pipeline keeps producing are dropped here; the engine/telemetry overrun counts surface it.
    if (sector_cursor_ + SECTORS_PER_BLOCK > capacity_sectors_) {
        return;
    }

    const uint32_t lba = base_lba_ + sector_cursor_;
    if (sd_write_engine().enqueue(lba, data.data())) {
        sector_cursor_ += SECTORS_PER_BLOCK;   // only advance once the block is safely staged
    }
    // else: engine ring full — block dropped (counted by the engine); cursor unchanged so the
    // next write reuses this sector. No FatFs touched either way: this never blocks.
}

void SdCard::finalize()
{
    if (info_.state != StorageState::Active) {
        return;
    }
    // Let every staged block reach the card before we change the file size (the engine shares the
    // peripheral across all three files, so this drains the whole pipe). Bounded by the ring depth
    // and the card's program time — tick() is driven from the main loop, which is paused here.
    SdWriteEngine& engine = sd_write_engine();
    while (!engine.idle()) {
        engine.tick();
    }

    // Truncate the pre-allocated extent back to what we actually wrote, returning the unused tail
    // to the volume, and commit the new size. Skipping finalize (a power-cut) just leaves the file
    // at full pre-alloc size — still readable, since each block carries its own footer/CRC.
    const FSIZE_t written = static_cast<FSIZE_t>(sector_cursor_) * SD_SECTOR_BYTES;
    if (f_lseek(&file_, written) != FR_OK || f_truncate(&file_) != FR_OK ||
        f_sync(&file_) != FR_OK) {
        fail(StorageError::FileWriteFail);
    }
}

void SdCard::fail(StorageError code)
{
    info_.status.error = code;
    info_.state        = StorageState::Error;
}

} // namespace platform::storage
