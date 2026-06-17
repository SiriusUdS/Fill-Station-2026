#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

#include "storage/interfaces/storage.hpp"        // logic::storage::Storage + StorageInfo
#include "actuation/interfaces/valve.hpp"        // logic::actuation::Valve
#include "communication/interfaces/adc.hpp"      // logic::communication::StreamingAdc + AdcInfo
#include "communication/interfaces/can.hpp"      // logic::communication::CanFrame
#include "communication/interfaces/power_monitor.hpp"  // logic::communication::PowerMonitor + PowerMonitorInfo
#include "control/persistent_state.hpp"          // fill_state — tags each downlinked record with the ECU's state
#include "telemetry/sd_recorder.hpp"             // logic::telemetry::SdRecorder (shared 3-file SD policy)
#include "communication/protocol/telemetry/sd_block_footer.hpp"  // SD_BLOCK_PAYLOAD_CAP (footer reservation)
#include "telemetry/extended_base.hpp"           // logic::telemetry::fillExtendedBase (shared prefix)

#include "communication/protocol/telemetry/ecu_system_state.hpp"  // EcuSystemState (+ SystemStateBase)
#include "communication/protocol/telemetry/ecu_extended_system_state.hpp"  // EcuExtendedSystemState (low-rate)
#include "communication/system_state_codec.hpp"  // packSystemState (CAN fragments)
#include "system/valves/ecu.hpp"                                  // EcuValves (valve identity / array index SSOT)
#include "system/board_id.hpp"

/* ------------------------------------------------------------------------- *
 * ECU telemetry pipeline (HAL-free) — the record-production + downlink half of
 * the ECU: the ADC and the SD card, plus the live downlink to the FCU over CAN.
 * The CAN-only sibling of logic::fcu::Telemetry (no Ethernet egress).
 *
 * It owns the telemetry buffer ring (log_), turns each ADC conversion into an
 * EcuSystemState record (produce), and flushes full slots to SD AND downlinks every
 * record in the slot onto the CAN bus (drain): the SD and the CAN downlink both carry
 * the full-rate log, so the FCU relays the ECU's live stream to the ground station at
 * the same rate the FCU sends its own. A record is one CAN-FD frame now (the codec is
 * a single fragment); the platform CAN driver paces the per-drain burst through a
 * software TX ring so it cannot overflow the hardware FIFO. It speaks through the
 * injected Communication layer; the shared SystemState codec does the CAN framing.
 *
 * In firmware the owning controller instance is placed in D1 AXI-SRAM (see main.cpp)
 * so the SD write can hand a buffer half straight to the SDMMC DMA.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

namespace detail {

/* Telemetry buffer slot size — shared with the SD recorder, which writes a whole slot
   verbatim to data_fast.bin. */
inline constexpr std::size_t LOG_HALF_BYTES = logic::telemetry::SD_LOG_BLOCK_BYTES;

/* Depth of the telemetry buffer ring: how many LOG_HALF_BYTES slots the producer can run
   ahead of the consumer. This was a 2-slot double buffer, which overran when an f_sync stall
   held off the drain; a deeper ring absorbs those bursts. Costs LOG_BUFFER_COUNT * LOG_HALF_BYTES
   of (AXI-SRAM) RAM. */
inline constexpr std::size_t LOG_BUFFER_COUNT = 4;

/* drain() downlinks at most ONE telemetry slot per call, and within it advances a byte cursor
   (Telemetry::downlink_off_) that honours CAN backpressure: it sends records until the driver's
   software TX ring rejects one (full), then returns, holding the cursor so the next tick resumes
   the same record — nothing is dropped. This is what fixes the 16 KB-block regression: a slot now
   holds ~292 records (was ~72 at 4 KB), and dumping all of them into the 192-deep TX ring in one
   burst overflowed it and dropped ~⅓ of the stream (GS rate fell from ~2000 to ~1400/s). A fixed
   records-per-tick cap would NOT help — the main loop free-spins, so it would just dump the same
   292 across a handful of back-to-back iterations and overflow the ring all the same; gating on the
   ring's own fullness paces the drain to the wire. The deep ring (LOG_BUFFER_COUNT) still absorbs
   an SD stall; the cursor absorbs the per-slot CAN burst. */

/* If the ADC ring stays empty this long, the ADC is presumed silent and the record
   timer emits filler records (flagged invalid) so the downlink rate holds. */
inline constexpr uint32_t ADC_TIMEOUT_MS = 10;

/* ExtendedSystemState cadence (low-rate; for now control flags + refused transition +
   power monitor, later event timestamps). Logged to data_ext.bin from the foreground AND
   downlinked to the FCU over CAN once per record (unbatched), so the FCU can relay the
   ECU's slow state to the GS the moment it lands. */
inline constexpr uint32_t EXTENDED_INTERVAL_MS = 100;   // ~10 Hz

/* The telemetry buffer ring. Single-producer (the record-timer ISR fills `head`) /
   single-consumer (the foreground drain empties `tail`), draining in fill order. A plain
   aggregate (no member initializers) so the constructor does not touch its memory at
   static-init; init() zeroes it. Pinned in D1 AXI-SRAM via the controller instance's
   placement in firmware. */
struct LogBuffer {
    uint8_t           data[LOG_BUFFER_COUNT][LOG_HALF_BYTES];
    volatile uint16_t used [LOG_BUFFER_COUNT];   // bytes filled in each slot
    volatile bool     ready[LOG_BUFFER_COUNT];   // slot full, awaiting drain
    uint8_t           head;   // slot the producer is filling
    uint8_t           tail;   // next slot the consumer will drain
    volatile uint16_t overrun_count;  // slots dropped because the ring was full (saturating)
};

} // namespace detail

/**
 * @brief The ECU telemetry pipeline, parameterised on its held drivers + the
 *        communication layer it downlinks through.
 * @tparam S logic::storage::Storage (the SD card in firmware).
 * @tparam V logic::actuation::Valve (read for telemetry; both IPA and NOS).
 * @tparam A logic::communication::StreamingAdc (the ADS131M08).
 * @tparam Comm The ECU Communication layer (frames + sends to the FCU).
 * @tparam PM logic::communication::PowerMonitor (the INA3221 on I2C4).
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, typename Comm,
          logic::communication::PowerMonitor PM>
class Telemetry {
public:
    /** @brief Construct over the held drivers + the communication layer. The three SD
     *         streams are the raw / 125 Hz-averaged SystemState and the ExtendedSystemState
     *         (same shared recorder policy as the FCU). */
    Telemetry(S& storage_fast, S& storage_slow, S& storage_ext,
              V& ipa_valve, V& nos_valve, A& adc, Comm& comm, PM& power_monitor)
        : recorder_(storage_fast, storage_slow, storage_ext),
          ipa_valve_(ipa_valve), nos_valve_(nos_valve), adc_(adc), comm_(comm),
          power_monitor_(power_monitor) {}

    /** @brief Zero the buffer ring and bring the three SD log files online. */
    void init()
    {
        std::memset(log_.data, 0, sizeof(log_.data));
        for (std::size_t i = 0; i < detail::LOG_BUFFER_COUNT; ++i) {
            log_.used[i]  = 0;
            log_.ready[i] = false;
        }
        log_.head = 0;
        log_.tail = 0;
        log_.overrun_count = 0;
        last_adc_ms_   = 0;
        downlink_off_  = 0;
        tail_recorded_ = false;

        recorder_.init();   // mounts the volume + opens data_fast/slow/ext.bin
    }

    /**
     * @brief  Produce telemetry record(s) — the comms/save cadence, driven by a
     *         dedicated timer (NOT the ADC). Drains every conversion queued in the
     *         ADC's ring (each a fresh record); on a silent ADC emits one filler
     *         flagged invalid. Runs in the timer ISR — keep it off the SD/CAN paths.
     */
    void produce(uint32_t now_ms)
    {
        bool produced = false;
        while (auto sample = adc_.pop()) {     // every queued conversion, exactly once
            last_adc_ms_ = now_ms;
            logAppend(buildSystemState(*sample, now_ms));
            produced = true;
        }
        if (!produced && (now_ms - last_adc_ms_) > detail::ADC_TIMEOUT_MS) {
            AdcInfo info = adc_.info();
            info.state             = AdcState::Faulted;
            info.status.data_valid = 0u;
            logAppend(buildSystemState(info, now_ms));
        }
    }

    /** @brief Flush every ready slot in fill order (from the tail): SD-log the SystemState
     *         through the shared recorder (raw data_fast.bin vs 125 Hz averaged data_slow.bin,
     *         per the flags), and downlink EVERY record in the slot to the FCU over CAN (always,
     *         full-rate —
     *         the CAN stream is the ECU's live link, independent of SD recording mode, and
     *         now carries the same 2 kHz the SD log does). This only fills the driver's
     *         software TX ring and returns; the ring paces the burst onto the bus in the
     *         background (TX-FIFO-empty ISR), so the loop never waits on the wire. */
    void drain(uint32_t now_ms)
    {
        if (!log_.ready[log_.tail]) {
            return;   // nothing ready to drain
        }
        // Acquire: the producer (TIM6 ISR) published the slot's bytes before setting ready;
        // read them only after observing the flag. Pairs with the release in logAppend.
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint8_t  t     = log_.tail;
        const uint16_t bytes = log_.used[t];

        // SD-log the whole slot once, the first time we touch it — independent of the CAN
        // downlink pacing below, so a busy CAN ring never re-writes the slot to the card.
        // Footer stamping touches only [used, end); the records we downlink are [0, used), so
        // the order is immaterial. SD has its own (engine-ring) backpressure and is best-effort.
        if (!tail_recorded_) {
            recorder_.recordSystemState(
                std::span<uint8_t>(log_.data[t], detail::LOG_HALF_BYTES), bytes, now_ms);
            tail_recorded_ = true;
        }

        // Downlink whole records from where we left off (one FD frame each) until the CAN TX ring
        // rejects one, then yield — the cursor stays put so the rejected record rides the next
        // tick instead of being dropped. This paces the ~292-record burst to the wire so it never
        // overflows the 192-deep software TX ring.
        while (downlink_off_ + sizeof(EcuSystemState) <= bytes) {
            EcuSystemState rec;
            std::memcpy(&rec, &log_.data[t][downlink_off_], sizeof(EcuSystemState));
            if (!sendRecordCan(rec)) {
                return;   // TX ring full: resume this record next tick (not dropped)
            }
            downlink_off_ += sizeof(EcuSystemState);
        }

        // Slot fully downlinked: release it and advance to the next.
        log_.ready[t]  = false;
        log_.tail      = static_cast<uint8_t>((t + 1) % detail::LOG_BUFFER_COUNT);
        downlink_off_  = 0;
        tail_recorded_ = false;
    }

    /** @brief Build the low-rate ExtendedSystemState (~10 Hz), log it to data_ext.bin,
     *         AND downlink it to the FCU over CAN — one record
     *         per send, unbatched (unlike the SystemState stream, which drains a full half
     *         at a time), so the FCU relays it straight on to the GS. Foreground-driven;
     *         self-throttled. */
    void produceExtended(uint32_t now_ms)
    {
        if ((now_ms - last_extended_ms_) < detail::EXTENDED_INTERVAL_MS) {
            return;
        }
        last_extended_ms_ = now_ms;

        EcuExtendedSystemState ext = {};
        // Shared prefix: timestamp + base control flags + refused-command diagnostics. The ECU
        // has no per-board flags, so its per-board control-flags byte is 0.
        logic::telemetry::fillExtendedBase(ext.base, now_ms, /*control_flags_board=*/0);
        ext.base.sd_write_engine_info = recorder_.engineHealth();  // board-wide async SD write-engine health
        ext.power_monitor = power_monitor_.info();  // INA3221 (I2C4), polled at ~10 Hz
        recorder_.recordExtended(ext, now_ms);   // accumulate -> data_ext.bin
        sendExtendedCan(ext);   // downlink to the FCU (unbatched), which relays it to the GS
    }

    /** @brief Advance the power monitor's non-blocking acquisition one step (off the record-timer
     *         ISR). The latest reading is folded into the extended record by produceExtended. */
    void servicePowerMonitor(uint32_t now_ms) { power_monitor_.service(now_ms); }

private:
    // Build an EcuSystemState from the ADC's info record (a fresh conversion on the
    // sample path, or a synthesized Faulted record on the silent-ADC fallback path).
    EcuSystemState buildSystemState(const AdcInfo& adc, uint32_t now_ms)
    {
        EcuSystemState state = {};
        state.base.creation_timestamp_ms = now_ms;

        // Each peripheral OWNS its telemetry record; the pipeline only reads it. The
        // recorder reports the worst of its three SD files (any stream's failure shows).
        state.base.adc_info     = adc;
        state.base.storage_info = recorder_.health();
        state.base.can_info     = comm_.canInfo();
        // The ECU has no Ethernet peripheral, so EcuSystemState carries no eth_info.

        // The double-buffer overrun count (halves dropped because the previous one could
        // not be flushed in time) is a logging-pipeline fault — surface it on the SD card's
        // own record. A running count, not a sticky flag, so a one-off boot stall reads
        // differently from a sustained shortfall.
        state.base.storage_info.overrun_count = log_.overrun_count;

        // The ECU's two valves report their own info, indexed by the EcuValves SSOT.
        state.base.valve_info[static_cast<std::size_t>(EcuValves::IPA)] = ipa_valve_.info();
        state.base.valve_info[static_cast<std::size_t>(EcuValves::NOS)] = nos_valve_.info();

        return state;
    }

    // Append one record to the head slot (single-producer: produce() in the record-timer
    // ISR). When a slot fills it is marked ready for drain() and the head advances; if the
    // next slot is still unflushed (the ring is full) the record is dropped and overrun is
    // flagged.
    void logAppend(const EcuSystemState& record)
    {
        uint8_t h = log_.head;
        // Flip at the payload cap (slot size minus the footer), so the drained slot always has
        // room for the SD block footer the recorder stamps into its tail.
        if (log_.used[h] + sizeof(EcuSystemState) > logic::telemetry::SD_BLOCK_PAYLOAD_CAP) {
            const uint8_t next = static_cast<uint8_t>((h + 1) % detail::LOG_BUFFER_COUNT);
            if (log_.ready[next]) {      // ring full: the next slot is the oldest, not drained yet
                if (log_.overrun_count != UINT16_MAX) {
                    log_.overrun_count = static_cast<uint16_t>(log_.overrun_count + 1);  // dropped record (saturating)
                }
                return;
            }
            // Release: commit the slot's record bytes (filled by prior logAppend calls) before
            // publishing ready, so the foreground drain never sees ready ahead of the data.
            std::atomic_thread_fence(std::memory_order_release);
            log_.ready[h]   = true;      // hand the now-full slot to the consumer
            log_.head       = next;
            log_.used[next] = 0;
            h = next;
        }
        std::memcpy(&log_.data[h][log_.used[h]], &record, sizeof(EcuSystemState));
        log_.used[h] = static_cast<uint16_t>(log_.used[h] + sizeof(EcuSystemState));
    }

    // Fragment one EcuSystemState into CAN frames (shared codec) and send them to the FCU through
    // the communication layer. Returns true if every fragment was queued; false if the CAN TX ring
    // rejected one, so the caller holds its cursor and retries the whole record next tick (the
    // sequence is only bumped on success, so a retried record keeps its number — no torn record on
    // the wire). With one fragment per record today this is just that frame's result; a multi-
    // fragment record would need the ring to have room for all fragments before the first send.
    bool sendRecordCan(const EcuSystemState& record)
    {
        namespace codec = logic::communication::can;
        std::array<logic::communication::CanFrame, codec::SYSTEM_STATE_FRAGMENTS> frames;
        codec::packSystemState(
            record, BoardId::Engine, BoardId::FillingStation,
            static_cast<uint8_t>(logic::control::persistent_state.fill_state), telemetry_seq_,
            std::span<logic::communication::CanFrame, codec::SYSTEM_STATE_FRAGMENTS>(frames));
        for (const auto& f : frames) {
            if (comm_.sendFrame(f).has_value()) {
                return false;   // TX ring full: caller must not advance its cursor
            }
        }
        ++telemetry_seq_;
        return true;
    }

    // Fragment one EcuExtendedSystemState into CAN frames (shared codec) and send them to the
    // FCU. Unbatched: produceExtended emits one record at ~10 Hz and hands it straight to the
    // bus, so the FCU relays the ECU's slow state to the GS the moment it arrives. Its own 4-bit
    // sequence (separate from the SystemState stream the FCU reassembles independently).
    void sendExtendedCan(const EcuExtendedSystemState& record)
    {
        namespace codec = logic::communication::can;
        std::array<logic::communication::CanFrame, codec::EXTENDED_STATE_FRAGMENTS> frames;
        codec::packExtendedSystemState(
            record, BoardId::Engine, BoardId::FillingStation,
            static_cast<uint8_t>(logic::control::persistent_state.fill_state), extended_seq_,
            std::span<logic::communication::CanFrame, codec::EXTENDED_STATE_FRAGMENTS>(frames));
        for (const auto& f : frames) {
            comm_.sendFrame(f);
        }
        ++extended_seq_;
    }

    logic::telemetry::SdRecorder<S, EcuSystemState> recorder_;  // the 3-file SD recording policy
    V&    ipa_valve_;    // injected IPA / NOS valves, read for telemetry
    V&    nos_valve_;
    A&    adc_;         // injected streaming ADC; produce() drains its ring
    Comm& comm_;        // injected communication layer; frames + downlinks records
    PM&   power_monitor_;  // injected INA3221 (I2C4); read into the extended record
    detail::LogBuffer log_;            // .axisram in firmware; left uninitialised until init()
    volatile uint32_t  last_adc_ms_      = 0;  // last tick a conversion was drained; gates the silent-ADC filler
    uint32_t           last_extended_ms_ = 0;  // throttles produceExtended() to ~10 Hz
    std::size_t        downlink_off_     = 0;  // byte cursor into the tail slot: how far the CAN downlink got
    bool               tail_recorded_    = false;  // the tail slot has been SD-logged (once, regardless of downlink pacing)
    uint8_t            telemetry_seq_     = 0;  // 4-bit CAN SystemState record sequence (wraps)
    uint8_t            extended_seq_      = 0;  // 4-bit CAN ExtendedSystemState record sequence (wraps)

    static_assert(std::extent_v<decltype(SystemStateBase::valve_info)> == 2,
                  "EcuSystemState expects exactly two valves (IPA, NOS)");
};

} // namespace logic::ecu
