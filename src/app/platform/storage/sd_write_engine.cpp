/**
 ******************************************************************************
 * @file    storage/sd_write_engine.cpp
 * @brief   The board's single asynchronous SD write engine. Arbitrates the one SDMMC
 *          peripheral across the three pre-allocated log files: enqueue() copies a block
 *          into an AXI-SRAM ring and returns; tick() kicks the next over the internal
 *          DMA when the card is idle; the SDMMC ISR (onComplete/onError) frees the slot.
 *          No FatFs, no f_sync — the FAT was committed up front by f_expand, so the
 *          foreground never blocks on a save. See sd_write_engine.hpp for the model.
 ******************************************************************************
 */

#include "storage/sd_write_engine.hpp"

#include <cstring>   // std::memcpy

namespace platform::storage {

/* The single engine instance, pinned in DMA-reachable D1 AXI-SRAM (the SDMMC internal DMA
   cannot reach DTCM). Declared in the NOLOAD .axisram section with a trivial constructor, so
   it carries no static-init cost — init() brings it online before first use, mirroring how the
   telemetry ring and the SdCard files are placed (see main.cpp). */
__attribute__((section(".axisram"))) static SdWriteEngine s_engine;

SdWriteEngine& sd_write_engine() { return s_engine; }

void SdWriteEngine::init(SD_HandleTypeDef* handle)
{
    handle_ = handle;
    for (unsigned i = 0; i < SD_WRITE_QUEUE_DEPTH; ++i) {
        state_[i] = Slot::Free;
        lba_[i]   = 0;
    }
    head_          = 0;
    tx_            = 0;
    busy_          = false;
    errored_       = false;
    overrun_count_ = 0;
    // buffers_ left untouched: every byte is overwritten by enqueue() before transmission.
}

bool SdWriteEngine::enqueue(uint32_t lba, const uint8_t* block)
{
    // The producer owns head_. The ISR only ever frees the slot at tx_, so reading state_[head_]
    // here races at most with a completion that turns a full ring not-full — which would only
    // cost us this one (legitimately dropped) block, never corrupt a slot.
    if (state_[head_] != Slot::Free) {
        if (overrun_count_ != UINT16_MAX) {
            overrun_count_ = static_cast<uint16_t>(overrun_count_ + 1);  // ring full: drop (saturating)
        }
        return false;
    }

    std::memcpy(buffers_[head_], block, SD_WRITE_BLOCK_BYTES);
    lba_[head_]   = lba;
    state_[head_] = Slot::Pending;             // publish AFTER the copy + lba are in place
    head_         = static_cast<uint8_t>((head_ + 1) % SD_WRITE_QUEUE_DEPTH);
    return true;
}

void SdWriteEngine::tick()
{
    // While busy_ is false no DMA is outstanding, so the SDMMC ISR cannot fire and this read-then-
    // start sequence cannot race the completer. While busy_ is true we do nothing.
    if (busy_) {
        return;
    }
    if (state_[tx_] != Slot::Pending) {
        return;   // ring empty
    }
    // The previous block's data transfer is done, but the card may still be programming it
    // internally (DAT0 busy). A quick CMD13 poll — not a data transfer — gates the next kick so
    // we never issue a write into a busy card; retry on the next tick if it is not ready yet.
    if (HAL_SD_GetCardState(handle_) != HAL_SD_CARD_TRANSFER) {
        return;
    }

    busy_         = true;
    state_[tx_]   = Slot::InFlight;
    if (HAL_SD_WriteBlocks_DMA(handle_, buffers_[tx_], lba_[tx_], SECTORS_PER_BLOCK) != HAL_OK) {
        // Failed to even start: drop this block rather than wedge the pipe, and flag the fault.
        errored_    = true;
        state_[tx_] = Slot::Free;
        tx_         = static_cast<uint8_t>((tx_ + 1) % SD_WRITE_QUEUE_DEPTH);
        busy_       = false;
    }
}

void SdWriteEngine::onComplete()
{
    // The in-flight block at tx_ is now on the card: release the slot and open the gate.
    state_[tx_] = Slot::Free;
    tx_         = static_cast<uint8_t>((tx_ + 1) % SD_WRITE_QUEUE_DEPTH);
    busy_       = false;
}

void SdWriteEngine::onError()
{
    // The in-flight block failed mid-transfer: drop it, flag the fault, and free the gate so the
    // following blocks still flow (a single bad write does not stall the whole stream).
    errored_    = true;
    state_[tx_] = Slot::Free;
    tx_         = static_cast<uint8_t>((tx_ + 1) % SD_WRITE_QUEUE_DEPTH);
    busy_       = false;
}

} // namespace platform::storage

/* ------------------------------------------------------------------------- *
 * SDMMC transfer-complete / error hooks (C linkage).
 *
 * HAL_SD_IRQHandler (driven by SDMMC2_IRQHandler) calls HAL_SD_TxCpltCallback on a finished DMA
 * write, which the BSP forwards to BSP_SD_WriteCpltCallback (a __weak stub in bsp_driver_sd.c).
 * We provide the strong definition here so completion reaches the engine. HAL_SD_ErrorCallback is
 * likewise __weak in the HAL driver; override it to drop the bad block. Both run in ISR context.
 * ------------------------------------------------------------------------- */

extern "C" void BSP_SD_WriteCpltCallback(void)
{
    platform::storage::sd_write_engine().onComplete();
}

extern "C" void HAL_SD_ErrorCallback(SD_HandleTypeDef* /*hsd*/)
{
    platform::storage::sd_write_engine().onError();
}
