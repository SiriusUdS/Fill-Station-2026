/**
 ******************************************************************************
 * @file    storage/storage.cpp
 * @brief   Platform definition of the logic Storage seam. Binds the SD card
 *          (SDMMC2, hsd2) and routes Storage's statically-linked methods to it,
 *          so the controller reaches storage without seeing FatFs/HAL.
 ******************************************************************************
 */

#include "storage/storage.hpp"   // logic::storage::Storage (the interface)
#include "storage/sd_card.hpp"   // platform::storage::SdCard

#include "sdmmc.h"               // hsd2 (CubeMX handle)

namespace {

// The store behind the Storage seam, on SDMMC2 / FatFs drive 0. Constructed at
// static-init with just the handle (no hardware touched); init() mounts it.
//
// Pinned in D1 AXI-SRAM: the SDMMC data path always moves through the SDMMC
// internal DMA, which cannot reach DTCM (the default .bss). The FATFS window and
// FIL buffer inside this object are handed to that DMA on f_sync/f_write, so
// they must live in DMA-reachable memory. (D-cache is off, so no invalidation
// is needed.)
__attribute__((section(".axisram"))) platform::storage::SdCard s_card{&hsd2, "0:/"};

} // namespace

namespace logic::storage {

void Storage::init()
{
    s_card.init();
}

void Storage::write(std::span<const uint8_t> data)
{
    s_card.write(data);
}

StorageStatus Storage::status() const
{
    return s_card.status();
}

} // namespace logic::storage
