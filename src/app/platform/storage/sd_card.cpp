/**
 ******************************************************************************
 * @file    storage/sd_card.cpp
 * @brief   FatFs-backed SD log file. At the first init() it mounts the volume and
 *          creates a fresh per-boot session folder (the highest existing numeric
 *          folder + 1); both shared across every file on the card. Each instance
 *          then opens one log file inside that folder and appends each record,
 *          flushing (f_sync) in batches so the costly FAT/directory write does not
 *          stall the producer into a telemetry overrun. The store owns its
 *          StorageInfo (state + status, carrying the last error cause), exposed
 *          through info(). C++ port of the original sd_card.c.
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

/* f_sync (the FAT + directory-entry flush) is the costly, latency-spiky part of a save;
   doing it every block stalls the producer enough to overrun the telemetry double buffer.
   Batch it: sync once every this many writes. The trade is that up to this many writes'
   worth of records can be lost on a power cut — small relative to a full run, and the
   telemetry stream is also downlinked live regardless. */
constexpr unsigned SYNC_PERIOD_WRITES = 16;

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

    // This stream's file inside the session folder, e.g. "0:/7/data_fast.bin".
    // Opened once and kept open for the driver's lifetime.
    char path[40];
    std::snprintf(path, sizeof(path), "%s%s", session, filename_);
    if (f_open(&file_, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        fail(StorageError::FileOpenFail);
        return;
    }

    info_.status.initialized = 1u;             // volume mounted, session folder + log file open
    info_.status.error       = StorageError::None;
    info_.state              = StorageState::Active;
}

void SdCard::write(std::span<const uint8_t> data)
{
    if (info_.state != StorageState::Active) {
        return;  // not ready (init() failed or never ran)
    }

    const UINT len = static_cast<UINT>(data.size());
    UINT written = 0;
    if (f_write(&file_, data.data(), len, &written) != FR_OK || written != len) {
        fail(StorageError::FileWriteFail);
        return;
    }

    // Flush the FAT + directory entry in batches, not every block: f_sync is the costly,
    // latency-spiky part of a save, and syncing every write stalls the producer enough to
    // overrun the telemetry double buffer. The first write always syncs so the file's
    // directory entry is committed immediately (the file appears on the card right away,
    // not only once a full period of records has accrued).
    if (++writes_since_sync_ >= SYNC_PERIOD_WRITES || !first_sync_done_) {
        if (f_sync(&file_) != FR_OK) {
            fail(StorageError::FileWriteFail);
            return;
        }
        writes_since_sync_ = 0;
        first_sync_done_   = true;
    }
}

void SdCard::fail(StorageError code)
{
    info_.status.error = code;
    info_.state        = StorageState::Error;
}

} // namespace platform::storage
