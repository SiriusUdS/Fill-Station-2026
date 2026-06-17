#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32h7xx_hal.h"   // SD_HandleTypeDef, HAL_SD_*

#include "communication/protocol/telemetry/sd_block_footer.hpp"  // SD_LOG_BLOCK_BYTES (the on-disk block-size SSOT)

/* ------------------------------------------------------------------------- *
 * Asynchronous, fire-and-forget SD write engine — the level-3 datalogger seam.
 *
 * The board logs three streams (data_fast/slow/ext) to ONE physical card through ONE
 * SDMMC peripheral, so only one DMA write can be in flight at a time. This engine is the
 * single arbiter for that peripheral: each SdCard pre-allocates a contiguous file (f_expand)
 * and hands the engine whole sector-aligned blocks tagged with their ABSOLUTE card sector
 * (LBA). enqueue() copies the block into an AXI-SRAM ring and returns immediately — it never
 * touches FatFs and never blocks. tick() (called once per main-loop iteration) kicks the
 * next queued block over the SDMMC internal DMA when the engine is idle and the card is ready;
 * the SDMMC transfer-complete interrupt (onComplete) frees the slot and lets the next go.
 *
 * The effect: the control loop never spins on an f_write or an f_sync — the costly FAT/dir
 * flush is gone entirely (the FAT was written up front by f_expand), and the byte transfer
 * runs on the DMA, not the CPU. A burst of drained slots is absorbed by the ring and paced out
 * one block per iteration, so no single tick stalls on the card.
 *
 * Memory/coherency: the queue buffers must live in DMA-reachable D1 AXI-SRAM (NOT DTCM), so the
 * single engine instance is placed there (see sd_write_engine.cpp). The CM7 data cache is
 * disabled on this board (only SCB_EnableICache() runs), so no cache maintenance is needed
 * around the DMA — were the D-cache ever enabled, enqueue() would have to clean the buffer.
 *
 * Concurrency: single-producer (enqueue + tick, both main-loop) / single-completer
 * (onComplete/onError, SDMMC2 ISR). Exactly one block is ever in flight, gated by busy_, so the
 * producer side runs lock-free: while busy_ is false no DMA is outstanding and the ISR cannot
 * fire, and while it is true tick() does nothing. head_ is touched only by the producer; tx_
 * only by the completer. See the per-member notes below.
 * ------------------------------------------------------------------------- */

namespace platform::storage {

/** @brief One written block is one sector-aligned SD_LOG_BLOCK_BYTES region — the same on-disk
 *         block size the telemetry rings and the SD recorder use (no sub-block writes). */
inline constexpr std::size_t SD_WRITE_BLOCK_BYTES = logic::telemetry::SD_LOG_BLOCK_BYTES;

/** @brief SD logical sector size (bytes). SDHC/SDXC use 512-byte block addressing, so an LBA is
 *         a 512-byte sector index and a block spans SECTORS_PER_BLOCK of them. */
inline constexpr uint32_t SD_SECTOR_BYTES = 512u;
inline constexpr uint32_t SECTORS_PER_BLOCK = SD_WRITE_BLOCK_BYTES / SD_SECTOR_BYTES;  // 4096/512 = 8

/** @brief Depth of the write ring: how many whole blocks the producer can stage ahead of the
 *         in-flight DMA. Sized to absorb a drain() burst (the fast ring's full slots plus the
 *         slow/ext blocks that can come due in one tick) while the card programs the previous
 *         block. Costs SD_WRITE_QUEUE_DEPTH * SD_WRITE_BLOCK_BYTES of AXI-SRAM. */
inline constexpr unsigned SD_WRITE_QUEUE_DEPTH = 8u;

class SdWriteEngine {
public:
    /** @brief Default-construct trivially (no member initializers) so the single instance lands
     *         in NOLOAD AXI-SRAM with no static-init cost; init() sets it up before first use. */
    SdWriteEngine() = default;

    /** @brief Bind the SDMMC handle (+ the optional card-detect line) and reset the ring (all slots
     *         free, nothing in flight). Must run once at bring-up before any enqueue()/tick(); the
     *         data buffers are left untouched (every byte is overwritten by enqueue before it is
     *         transmitted).
     * @param  handle       The SDMMC SD handle this engine drives.
     * @param  detect_port  GPIO port of the SD_DETECT socket switch, or nullptr if unwired.
     * @param  detect_pin   GPIO pin of SD_DETECT (ignored when detect_port is nullptr). */
    void init(SD_HandleTypeDef* handle, GPIO_TypeDef* detect_port = nullptr, uint16_t detect_pin = 0);

    /**
     * @brief  Stage one whole block for writing at absolute card sector @p lba. Copies
     *         @p block (SD_WRITE_BLOCK_BYTES) into the ring and returns immediately — the
     *         caller's buffer is free on return. Non-blocking; safe to call from the foreground.
     * @return true if queued; false if the ring is full (the block is dropped and the saturating
     *         overrun count is bumped — the caller must NOT advance its sector cursor).
     */
    bool enqueue(uint32_t lba, const uint8_t* block);

    /** @brief Kick the next staged block over the SDMMC DMA if the engine is idle AND the card is
     *         ready (done programming the previous block). Non-blocking — at most one quick card-
     *         state poll plus one DMA start. Call once per main-loop iteration (the main-loop
     *         advance, like the valves' / controller's tick(); takes no timestamp). */
    void tick();

    /** @brief SDMMC transfer-complete hook (call from the SDMMC2 ISR): release the in-flight slot
     *         and clear busy_ so tick() can start the next block. */
    void onComplete();

    /** @brief SDMMC error hook (call from the SDMMC2 ISR): drop the in-flight block, flag the
     *         error and clear busy_ so the pipe does not wedge on one bad write. */
    void onError();

    /** @brief True when nothing is in flight and the ring is empty — used by finalize() to wait
     *         for the tail to drain before truncating a file to its written length. */
    [[nodiscard]] bool idle() const { return !busy_ && state_[tx_] != Slot::Pending; }

    /** @brief Saturating count of blocks dropped because the ring was full — surfaced in telemetry
     *         as a logging-pipeline shortfall (distinct from the card's own health). */
    [[nodiscard]] uint16_t overrun_count() const { return overrun_count_; }

    /** @brief True if a DMA write ever failed to start or errored mid-transfer (sticky). */
    [[nodiscard]] bool errored() const { return errored_; }

    /** @brief True when a card is seated in the socket (SD_DETECT active-low: present = pin low).
     *         Reports present when no detect line was wired at init(). A live GPIO read — no state. */
    [[nodiscard]] bool cardDetected() const;

private:
    enum class Slot : uint8_t { Free = 0, Pending, InFlight };

    // The single SDMMC handle this engine drives (set by init()).
    SD_HandleTypeDef* handle_;

    // The SD_DETECT socket switch (set by init(); nullptr if unwired). Read live by cardDetected().
    GPIO_TypeDef*     detect_port_;
    uint16_t          detect_pin_;

    // The staged blocks, one per ring slot. 32-byte aligned for the SDMMC internal DMA.
    alignas(32) uint8_t buffers_[SD_WRITE_QUEUE_DEPTH][SD_WRITE_BLOCK_BYTES];
    uint32_t            lba_[SD_WRITE_QUEUE_DEPTH];     // absolute target sector per slot
    volatile Slot       state_[SD_WRITE_QUEUE_DEPTH];   // Free / Pending / InFlight

    volatile uint8_t  head_;           // next slot the PRODUCER fills (enqueue only)
    volatile uint8_t  tx_;             // slot the COMPLETER frees (tick starts it, ISR frees it)
    volatile bool     busy_;           // a DMA write is in flight (gates the producer/ISR handoff)
    volatile bool     errored_;        // sticky: a write failed to start or errored
    volatile uint16_t overrun_count_;  // blocks dropped because the ring was full (saturating)
};

/** @brief The single shared write engine for this board's one physical card. Placed in AXI-SRAM
 *         (DMA-reachable) in the .cpp; every SdCard routes its blocks through it. */
SdWriteEngine& sd_write_engine();

/** @brief Non-fatal SDMMC card bring-up — the replacement for the generated MX_SDMMCx_SD_Init().
 *         That generated init calls Error_Handler() (a hard __disable_irq + while(1) lock) when
 *         HAL_SD_Init fails, so a board booted with no card seated would wedge there before ever
 *         reaching the main loop. tryInitSd() instead records the result: a missing/dead card just
 *         yields false, leaving logging off (SdCard::init() short-circuits, write() no-ops) while
 *         valves, the state machine, telemetry and CAN run normally. HAL_SD_Init still invokes
 *         HAL_SD_MspInit (GPIO/clock/NVIC) internally, so the peripheral is configured either way.
 *         The board fills hsd->Instance + hsd->Init.* (the per-board config mirrored from sdmmc.c)
 *         before calling this.
 * @return true if a card enumerated (HAL_OK); the result is also cached for sdPresent(). */
bool tryInitSd(SD_HandleTypeDef* hsd);

/** @brief Whether the boot-time tryInitSd() identified a card. SdCard::init() gates the FatFs
 *         mount/open path on this so an absent card never exercises the diskio driver. */
bool sdPresent();

} // namespace platform::storage
