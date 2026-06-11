/**
 ******************************************************************************
 * @file    storage/sd_card.cpp
 * @brief   FatFs-backed SD card driver: mounts the volume and opens a log file
 *          at init, then appends + syncs each record so writes stay cheap and
 *          power-loss safe. The store owns its StorageInfo (state + status, the
 *          status carrying the last error cause), exposed through info(). C++
 *          port of the original sd_card.c.
 ******************************************************************************
 */

#include "storage/sd_card.hpp"

#include <cstdio>   // std::snprintf (build the drive-relative file path)

namespace platform::storage {

SdCard::SdCard(SD_HandleTypeDef* handle, const char* drive)
    : handle_(handle), drive_(drive)
{
    info_.state = StorageState::Init;
}

void SdCard::init()
{
    if (f_mount(&fs_, drive_, 1) != FR_OK) {
        fail(StorageError::MountFail);
        return;
    }

    // Drive-relative path so each card writes to its own volume, e.g.
    // "0:/runtime.bin". Opened once and kept open for the driver's lifetime.
    char path[32];
    std::snprintf(path, sizeof(path), "%sruntime.bin", drive_);
    if (f_open(&file_, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        fail(StorageError::FileOpenFail);
        return;
    }

    info_.status.initialized = 1u;             // mounted + log file open
    info_.status.error       = StorageError::None;
    info_.state              = StorageState::Active;
}

void SdCard::write(std::span<const uint8_t> data)
{
    if (info_.state != StorageState::Active) {
        return;  // not ready (init() failed or never ran)
    }
    if (info_.status.write_enabled == 0) {
        return;  // writing disarmed (default) — intentional no-op, store stays Active
    }

    const UINT len = static_cast<UINT>(data.size());
    UINT written = 0;
    if (f_write(&file_, data.data(), len, &written) != FR_OK || written != len) {
        fail(StorageError::FileWriteFail);
        return;
    }

    // Flush data + directory entry so the appended record survives a power loss.
    f_sync(&file_);
}

void SdCard::fail(StorageError code)
{
    info_.status.error = code;
    info_.state        = StorageState::Error;
}

} // namespace platform::storage
