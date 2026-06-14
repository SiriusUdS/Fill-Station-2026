#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "storage/interfaces/storage.hpp"        // logic::storage::Storage, StorageInfo, StorageState
#include "control/control_flags.hpp"              // control_flags, ControlFlag
#include "communication/protocol/peripherals/adc/adc_info.hpp"   // ADC_CHANNEL_COUNT

/* ------------------------------------------------------------------------- *
 * Shared SD recording policy for both boards (HAL-free).
 *
 * The FCU and ECU log telemetry to the SD card identically — three fixed-size
 * record streams in separate files, gated by the same control flags — so that
 * policy lives HERE, not duplicated in each board's telemetry pipeline (they can't
 * drift). A board's telemetry owns the record TYPE and the downlink transport; it
 * hands drained SystemState halves and serialized ExtendedSystemState bytes to this
 * recorder, which routes them to the right file:
 *
 *   - data_fast.bin : raw high-rate SystemState (whole sector-aligned block)
 *   - data_slow.bin : the same stream block-averaged to 125 Hz (SLOW_WINDOW samples)
 *   - data_ext.bin  : the low-rate ExtendedSystemState (written verbatim)
 *
 * PersistingData gates all SD writes; FastRecording picks data_fast vs data_slow for
 * the SystemState stream. The slow averaging only touches the numeric ADC channels in
 * the shared SystemStateBase prefix, so it is record-type agnostic beyond `.base`.
 * ------------------------------------------------------------------------- */

namespace logic::telemetry {

/* Sector-aligned size of one drained telemetry half — the unit handed to
   recordSystemState() and written verbatim to data_fast.bin. Shared so both boards'
   double buffers and this recorder agree. */
inline constexpr std::size_t SD_LOG_BLOCK_BYTES = 4096;

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

    /** @brief Bring all three files online. The first init() mounts the volume +
     *         creates this boot's session folder (shared); the others reuse it. */
    void init()
    {
        fast_.init();
        slow_.init();
        ext_.init();
    }

    /**
     * @brief  Record one drained high-rate half per the control flags. No-op unless
     *         PersistingData is set; then FastRecording routes it to the raw 2 kHz
     *         data_fast.bin (the whole @p block) or the 125 Hz averaged data_slow.bin.
     * @param  block  the full sector-aligned half (SD_LOG_BLOCK_BYTES).
     * @param  used   bytes actually filled (a whole number of Records).
     */
    void recordSystemState(std::span<const uint8_t> block, std::size_t used)
    {
        if (!logic::control::control_flags.get(ControlFlag::PersistingData)) {
            return;
        }
        if (logic::control::control_flags.get(ControlFlag::FastRecording)) {
            fast_.write(block);
        } else {
            averageSlow(block.data(), used);
        }
    }

    /** @brief Log one ExtendedSystemState (already serialized) to data_ext.bin, while
     *         PersistingData is set. Independent of the Fast/Slow flag. */
    void recordExtended(std::span<const uint8_t> bytes)
    {
        if (logic::control::control_flags.get(ControlFlag::PersistingData)) {
            ext_.write(bytes);
        }
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

private:
    // Upper bound on the bytes one block contributes to the slow log (one averaged
    // record per SLOW_WINDOW samples), so the slow writes batch into one f_write.
    static constexpr std::size_t SLOW_BUFFER_BYTES =
        SD_LOG_BLOCK_BYTES / detail::SLOW_WINDOW + sizeof(Record);

    // Block-average @p used bytes of high-rate records into data_slow.bin. Accumulates
    // per-channel ADC sums; every SLOW_WINDOW samples emits one averaged record
    // (channels = sum >> SLOW_SHIFT; other fields from the window's last sample). The
    // accumulator persists across calls (a window may straddle two halves).
    void averageSlow(const uint8_t* block, std::size_t used)
    {
        std::array<uint8_t, SLOW_BUFFER_BYTES> out;
        std::size_t out_len = 0;

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
                std::memcpy(out.data() + out_len, &avg, sizeof(avg));
                out_len += sizeof(avg);
            }
        }
        if (out_len > 0) {
            slow_.write(std::span<const uint8_t>(out.data(), out_len));
        }
    }

    S& fast_;   // data_fast.bin (raw 2 kHz SystemState)
    S& slow_;   // data_slow.bin (125 Hz averaged SystemState)
    S& ext_;    // data_ext.bin  (ExtendedSystemState)

    // Slow-mode averaging accumulator (persists across drained halves).
    std::array<int32_t, ADC_CHANNEL_COUNT> sum_{};   // per-channel sum over the current window
    unsigned count_ = 0;                             // samples accumulated in the window
    Record   last_{};                                // last sample seen (non-numeric template)
};

} // namespace logic::telemetry
