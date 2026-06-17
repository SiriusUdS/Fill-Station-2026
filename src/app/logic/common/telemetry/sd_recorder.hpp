#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "storage/interfaces/storage.hpp"        // logic::storage::Storage, StorageInfo, StorageState
#include "control/control_flags.hpp"              // control_flags, ControlFlag
#include "communication/protocol/peripherals/adc/adc_info.hpp"   // ADC_CHANNEL_COUNT
#include "communication/protocol/telemetry/sd_block_footer.hpp"  // SD_LOG_BLOCK_BYTES, SD_BLOCK_PAYLOAD_CAP, stampSdBlockFooter
#include "data_integrity/crc32.hpp"              // logic::data_integrity::crc32 (the HW-backed CRC seam)

/* ------------------------------------------------------------------------- *
 * Shared SD recording policy for both boards (HAL-free).
 *
 * The FCU and ECU log telemetry to the SD card identically — three fixed-size
 * record streams in separate files — so that
 * policy lives HERE, not duplicated in each board's telemetry pipeline (they can't
 * drift). A board's telemetry owns the record TYPE and the downlink transport; it
 * hands drained SystemState halves and serialized ExtendedSystemState bytes to this
 * recorder, which routes them to the right file:
 *
 *   - data_fast.bin : raw high-rate SystemState (whole sector-aligned block)
 *   - data_slow.bin : the same stream block-averaged to 125 Hz (SLOW_WINDOW samples)
 *   - data_ext.bin  : the low-rate ExtendedSystemState (written verbatim)
 *
 * Telemetry is always persisted; FastRecording picks data_fast vs data_slow for
 * the SystemState stream. The slow averaging only touches the numeric ADC channels in
 * the shared SystemStateBase prefix, so it is record-type agnostic beyond `.base`.
 * ------------------------------------------------------------------------- */

namespace logic::telemetry {

/* SD_LOG_BLOCK_BYTES (the sector-aligned size of one written block) is the on-disk-format SSOT,
   defined alongside the footer in sd_block_footer.hpp and shared by both boards' buffer rings,
   this recorder, and any reader. Every file — data_fast/slow/ext — is written one footer-stamped
   SD_LOG_BLOCK_BYTES block at a time (no sub-block, unaligned writes). */

namespace detail {

/* Slow-mode block averaging: a window of SLOW_WINDOW samples (power of two, so the
   mean is sum >> SLOW_SHIFT) turns the 2 kHz stream into the 125 Hz data_slow.bin —
   an anti-aliasing decimator, not a throw-away decimation. */
inline constexpr unsigned SLOW_WINDOW = 16;   // 2 kHz / 16 = 125 Hz
inline constexpr unsigned SLOW_SHIFT  = 4;    // log2(SLOW_WINDOW)
static_assert((1u << SLOW_SHIFT) == SLOW_WINDOW, "SLOW_SHIFT must be log2(SLOW_WINDOW)");

} // namespace detail

/**
 * @brief The three-file SD recording policy, shared by both boards.
 * @tparam S      logic::storage::Storage (one open file each, on the shared volume).
 * @tparam Record the board's high-rate record (Fcu/EcuSystemState); must embed a
 *                SystemStateBase `base` with adc_info.channels (that is all the
 *                averaging touches).
 */
template <logic::storage::Storage S, typename Record>
class SdRecorder {
public:
    /** @brief Construct over the three log files (raw / averaged / extended). */
    SdRecorder(S& fast, S& slow, S& ext) : fast_(fast), slow_(slow), ext_(ext) {}

    /** @brief Bring all three files online and reset the slow/ext accumulators. The first
     *         init() mounts the volume + creates this boot's session folder (shared); the
     *         others reuse it. */
    void init()
    {
        fast_.init();
        slow_.init();
        ext_.init();
        slow_used_ = 0;
        ext_used_  = 0;
        count_     = 0;
        finalized_ = false;
        sum_.fill(0);
    }

    /**
     * @brief  Record one drained high-rate slot to the card. FastRecording routes it to the
     *         raw 2 kHz data_fast.bin (the whole @p block, footer-stamped) or the 125 Hz
     *         averaged data_slow.bin.
     * @param  block   the full sector-aligned slot (SD_LOG_BLOCK_BYTES); mutable so the fast
     *                 path can stamp the footer into its unused tail in place.
     * @param  used    payload bytes filled (a whole number of Records; <= SD_BLOCK_PAYLOAD_CAP).
     * @param  now_ms  the saving timestamp stamped into the footer.
     */
    void recordSystemState(std::span<uint8_t> block, std::size_t used, uint32_t now_ms)
    {
        if (loggingDisabled(now_ms)) {
            return;   // run finalized (tail reclaimed); stream stopped for the session
        }
        if (logic::control::base_control_flags.get(ControlFlagBase::FastRecording)) {
            // The fast block is the drained ring slot (already SD_LOG_BLOCK_BYTES): stamp the
            // footer over its unused tail [used, end) and write the whole sector-aligned block.
            logic::telemetry::stampSdBlockFooter(block, used, now_ms, &logic::data_integrity::crc32);
            fast_.write(block);
        } else {
            averageSlow(block.data(), used, now_ms);
        }
    }

    /** @brief Accumulate one ExtendedSystemState into the data_ext.bin block; when the block
     *         fills it is footer-stamped and written as one sector-aligned SD_LOG_BLOCK_BYTES
     *         block (no sub-block, unaligned writes). Independent of the Fast/Slow flag. */
    template <typename Ext>
    void recordExtended(const Ext& record, uint32_t now_ms)
    {
        static_assert(sizeof(Ext) <= logic::telemetry::SD_BLOCK_PAYLOAD_CAP,
                      "an ExtendedSystemState must fit in one SD block with room for the footer");
        if (loggingDisabled(now_ms)) {
            return;   // run finalized (tail reclaimed); stream stopped for the session
        }
        appendBlock(ext_, ext_block_.data(), ext_used_,
                    reinterpret_cast<const uint8_t*>(&record), sizeof(Ext), now_ms);
    }

    /** @brief Worst-of-three SD health: the first file in Error (with its cause), else
     *         the fast file. The three share one card but track their own FIL, so a
     *         failure on any stream surfaces (not just data_fast's). */
    [[nodiscard]] StorageInfo health() const
    {
        const StorageInfo infos[] = { fast_.info(), slow_.info(), ext_.info() };
        for (const StorageInfo& s : infos) {
            if (s.state == StorageState::Error) {
                return s;
            }
        }
        return infos[0];
    }

    /** @brief The board-wide async SD write engine's health (dropped-block count + sticky DMA
     *         error), for the low-rate ExtendedSystemState. The engine is shared by all three
     *         files, so any store reports the same value — read it once off the fast stream. */
    [[nodiscard]] SdWriteEngineInfo engineHealth() const { return fast_.engineInfo(); }

private:
    // True once SD logging has been stopped for this session — both record paths no-op after.
    // Driven by the DisableLogging base control flag: the FIRST record seen with the flag set
    // finalizes the run (below) and latches finalized_; thereafter logging stays off regardless
    // of the flag, because finalize() reclaimed each file's pre-allocated tail and the raw-sector
    // cursor can no longer safely advance into freed space — a reboot is required to log again.
    [[nodiscard]] bool loggingDisabled(uint32_t now_ms)
    {
        if (finalized_) {
            return true;   // already finalized this session: permanently off (cannot resume safely)
        }
        if (!logic::control::base_control_flags.get(ControlFlagBase::DisableLogging)) {
            return false;  // logging active
        }
        finalizeAll(now_ms);
        finalized_ = true;
        return true;
    }

    // End-of-run finalize across all three files: flush the partial slow/ext accumulators so the
    // last records are not lost (the fast stream has no accumulator — it writes drained slots
    // directly), then finalize each file. The engine is shared, so the first finalize() drains
    // every just-flushed block off the card; the rest then truncate their own files' unused tail.
    void finalizeAll(uint32_t now_ms)
    {
        flushBlock(slow_, slow_block_.data(), slow_used_, now_ms);
        flushBlock(ext_,  ext_block_.data(),  ext_used_,  now_ms);
        fast_.finalize();
        slow_.finalize();
        ext_.finalize();
    }

    // Block-average @p used bytes of high-rate records into the data_slow.bin block. Accumulates
    // per-channel ADC sums; every SLOW_WINDOW samples emits one averaged record (channels =
    // sum >> SLOW_SHIFT; other fields from the window's last sample) into the slow accumulator,
    // which flushes a footer-stamped SD_LOG_BLOCK_BYTES block once full. The averaging
    // accumulator persists across calls (a window may straddle two slots).
    void averageSlow(const uint8_t* block, std::size_t used, uint32_t now_ms)
    {
        for (std::size_t off = 0; off + sizeof(Record) <= used; off += sizeof(Record)) {
            Record rec;
            std::memcpy(&rec, block + off, sizeof(rec));

            for (unsigned c = 0; c < ADC_CHANNEL_COUNT; ++c) {
                sum_[c] += rec.base.adc_info.channels[c];
            }
            last_ = rec;   // template for the non-numeric fields (the window's latest)

            if (++count_ >= detail::SLOW_WINDOW) {
                Record avg = last_;
                for (unsigned c = 0; c < ADC_CHANNEL_COUNT; ++c) {
                    avg.base.adc_info.channels[c] = sum_[c] >> detail::SLOW_SHIFT;
                    sum_[c] = 0;
                }
                count_ = 0;
                appendBlock(slow_, slow_block_.data(), slow_used_,
                            reinterpret_cast<const uint8_t*>(&avg), sizeof(avg), now_ms);
            }
        }
    }

    // Append one `len`-byte record to a SD_LOG_BLOCK_BYTES accumulator (`block`/`used`); if it
    // would not leave room for the footer, flush the current block first. The slow + ext streams
    // use this so each writes one footer-stamped sector-aligned block at a time (the fast stream
    // writes its drained ring slot directly). Records are assumed <= SD_BLOCK_PAYLOAD_CAP.
    void appendBlock(S& store, uint8_t* block, std::size_t& used,
                     const uint8_t* rec, std::size_t len, uint32_t now_ms)
    {
        if (used + len > logic::telemetry::SD_BLOCK_PAYLOAD_CAP) {
            flushBlock(store, block, used, now_ms);
        }
        std::memcpy(block + used, rec, len);
        used += len;
    }

    // Footer-stamp the accumulated [0, used) payload and write the whole sector-aligned block,
    // then reset. The MAGIC fill spans [used, end), so the on-disk block is exactly
    // SD_LOG_BLOCK_BYTES with no dead padding. No-op on an empty block.
    void flushBlock(S& store, uint8_t* block, std::size_t& used, uint32_t now_ms)
    {
        if (used == 0) {
            return;
        }
        const std::span<uint8_t> full(block, SD_LOG_BLOCK_BYTES);
        logic::telemetry::stampSdBlockFooter(full, used, now_ms, &logic::data_integrity::crc32);
        store.write(full);
        used = 0;
    }

    S& fast_;   // data_fast.bin (raw 2 kHz SystemState)
    S& slow_;   // data_slow.bin (125 Hz averaged SystemState)
    S& ext_;    // data_ext.bin  (ExtendedSystemState)

    // Slow-mode averaging accumulator (persists across drained slots).
    std::array<int32_t, ADC_CHANNEL_COUNT> sum_{};   // per-channel sum over the current window
    unsigned count_ = 0;                             // samples accumulated in the window
    Record   last_{};                                // last sample seen (non-numeric template)
    bool     finalized_ = false;                     // DisableLogging latched: run finalized, streams off

    // Per-file SD_LOG_BLOCK_BYTES accumulators for the slow + ext streams, so each writes one
    // footer-stamped sector-aligned block at a time (no sub-block, unaligned writes). NOT zeroed
    // at construction — every byte is overwritten by records or the footer before a write — so
    // they add no static-init cost; the used-counters start empty. Ride the controller's
    // AXI-SRAM placement in firmware (the fast slot already does).
    std::array<uint8_t, SD_LOG_BLOCK_BYTES> slow_block_;
    std::array<uint8_t, SD_LOG_BLOCK_BYTES> ext_block_;
    std::size_t slow_used_ = 0;
    std::size_t ext_used_  = 0;
};

} // namespace logic::telemetry
