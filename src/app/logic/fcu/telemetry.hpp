#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

#include "storage/interfaces/storage.hpp"        // logic::storage::Storage + StorageInfo
#include "actuation/interfaces/valve.hpp"        // logic::actuation::Valve
#include "communication/interfaces/adc.hpp"      // logic::communication::StreamingAdc + AdcInfo
#include "communication/interfaces/thermocouple.hpp"  // logic::communication::ThermocoupleBank + ThermocoupleInfo
#include "control/persistent_state.hpp"          // Backup-SRAM state snapshot (drain tags the source state)
#include "control/control_flags.hpp"             // control_flags — PersistingData gates the SD write
#include "control/refused_transition.hpp"        // last_refused_transition — surfaced in ExtendedSystemState
#include "telemetry/sd_recorder.hpp"             // logic::telemetry::SdRecorder (shared 3-file SD policy)

#include "communication/protocol/framing/payload_type.hpp"        // PayloadType
#include "communication/protocol/telemetry/fcu_system_state.hpp"  // FcuSystemState
#include "communication/protocol/telemetry/fcu_extended_system_state.hpp"  // FcuExtendedSystemState (low-rate)
#include "communication/system_state_codec.hpp"  // EcuSystemState + SystemStateReassembler
#include "system/valves/fcu.hpp"                                  // FcuValves (valve identity / array index SSOT)
#include "system/board_id.hpp"
#include "system/state.hpp"
#include "telemetry/telemetry_type.hpp"

/* ------------------------------------------------------------------------- *
 * FCU telemetry pipeline (HAL-free) — the record-production + downlink half of
 * the FCU: the ADC and the SD card, plus the live downlink to the ground station.
 *
 * It owns the telemetry double buffer (log_), turns each ADC conversion into an
 * FcuSystemState record (produce), flushes full halves to SD and streams them to
 * the GS (drain), and relays the ECU's telemetry off the CAN bus to the GS
 * (relayEcuFrame). It does NOT own a transport: it speaks through the injected
 * Communication layer, which frames and sends. The only wire detail it keeps is
 * its own record batching (fitting fixed-size records into a datagram).
 *
 * In firmware the owning controller instance is placed in D1 AXI-SRAM (see
 * main.cpp) so the SD write can hand a buffer half straight to the SDMMC DMA;
 * this pipeline holds log_ as a member, so it rides along in that placement.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

namespace detail {

/* Telemetry double buffer: each FcuSystemState (one per ADC sample) is appended to
   the active half; when a half fills it is flushed to SD and streamed to the GS while
   the producer fills the other half. The half size is shared with the SD recorder
   (it writes a whole half verbatim to data_fast.bin). */
inline constexpr std::size_t LOG_HALF_BYTES = logic::telemetry::SD_LOG_BLOCK_BYTES;

/* If the ADC ring stays empty this long, the ADC is presumed silent and the
   record timer emits filler records (flagged invalid) so the packet rate holds. */
inline constexpr uint32_t ADC_TIMEOUT_MS = 10;   // > worst-case stall before declaring the ADC silent

/* ExtendedSystemState cadence (the slow/bulky record: thermocouples, later event
   timestamps). Built + downlinked + logged to data_ext.bin from the foreground. */
inline constexpr uint32_t EXTENDED_INTERVAL_MS = 100;   // ~10 Hz

/* The telemetry double buffer. A plain aggregate (no member initializers) so the
   owner's constructor does not touch its 8 KB at static-init; init() zeroes it.
   Pinned in D1 AXI-SRAM via the controller instance's placement in firmware. */
struct LogBuffer {
    uint8_t           data[2][LOG_HALF_BYTES];
    volatile uint16_t used[2];   // bytes filled in each half
    volatile bool     ready[2];  // half full, awaiting drain
    uint8_t           active;    // half the producer is filling (0/1)
    volatile bool     overrun;   // a half filled before the other was drained
};

} // namespace detail

/**
 * @brief The FCU telemetry pipeline, parameterised on its held drivers + the
 *        communication layer it downlinks through.
 * @tparam S logic::storage::Storage (the SD card in firmware).
 * @tparam V logic::actuation::Valve (read for telemetry; both Fill and Dump).
 * @tparam A logic::communication::StreamingAdc (the ADS131M08).
 * @tparam Comm The FCU Communication layer (frames + sends to the GS).
 * @tparam TC logic::communication::ThermocoupleBank (the 4 MAX31856 on SPI6).
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, typename Comm,
          logic::communication::ThermocoupleBank TC>
class Telemetry {
public:
    /** @brief Construct over the held drivers + the communication layer. The three
     *         SD streams are the high-rate SystemState (fast/slow, picked by the
     *         FastRecording flag) and the low-rate ExtendedSystemState. */
    Telemetry(S& storage_fast, S& storage_slow, S& storage_ext,
              V& fill_valve, V& dump_valve, A& adc, Comm& comm, TC& thermocouples)
        : recorder_(storage_fast, storage_slow, storage_ext),
          fill_valve_(fill_valve), dump_valve_(dump_valve),
          adc_(adc), comm_(comm), thermocouples_(thermocouples) {}

    /** @brief Zero the double buffer and bring the three SD log files online. */
    void init()
    {
        // The telemetry double buffer lives in NOLOAD .axisram — clear it and its
        // indices before the producer starts.
        std::memset(log_.data, 0, sizeof(log_.data));
        log_.used[0]  = 0;
        log_.used[1]  = 0;
        log_.ready[0] = false;
        log_.ready[1] = false;
        log_.active   = 0;
        log_.overrun  = false;
        last_adc_ms_  = 0;

        recorder_.init();   // mounts the volume + opens data_fast/slow/ext.bin
    }

    /**
     * @brief  Produce telemetry record(s) — the comms/save cadence, driven by a
     *         dedicated timer (NOT the ADC). Drains every conversion queued in the
     *         ADC's ring (each a fresh record); if the ring is empty and the ADC
     *         has timed out, emits one filler with the last-known data flagged
     *         invalid (Faulted / !data_valid). Runs in the timer ISR — keep it off
     *         the SD and Ethernet paths.
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
            // ADC silent: filler with the actual last-known record, flagged bogus.
            AdcInfo info = adc_.info();
            info.state             = AdcState::Faulted;
            info.status.data_valid = 0u;
            logAppend(buildSystemState(info, now_ms));
        }
    }

    /** @brief Advance the thermocouple bank's non-blocking acquisition one step.
     *         Driven from the foreground loop (NOT the record-timer ISR), since the
     *         bank talks to SPI6; it self-paces and never blocks. The latest readings
     *         are folded into each record by produce()/buildSystemState. */
    void serviceThermocouples(uint32_t now_ms) { thermocouples_.service(now_ms); }

    /** @brief Flush any full half: stream its records to the GS (always, full-rate),
     *         and — while PersistingData is set — log the SystemState to SD per the
     *         FastRecording flag: the raw 2 kHz block to data_fast.bin, or the 125 Hz
     *         block-averaged stream to data_slow.bin. The half is always released. */
    void drain(uint32_t now_ms)
    {
        for (uint8_t h = 0; h < 2; ++h) {
            if (!log_.ready[h]) {
                continue;
            }
            const uint16_t bytes = log_.used[h];

            // SD logging is the shared recorder's policy (raw data_fast.bin vs averaged
            // data_slow.bin, per the flags); the downlink below is unconditional and
            // full-rate, so the GS always sees live data regardless of recording mode.
            recorder_.recordSystemState(
                std::span<const uint8_t>(log_.data[h], detail::LOG_HALF_BYTES), bytes);

            downlink(BoardId::FillingStation,
                     static_cast<uint8_t>(logic::control::persistent_state.fill_state),
                     std::span<const uint8_t>(log_.data[h], bytes), sizeof(FcuSystemState), now_ms);
            log_.ready[h] = false;
        }
    }

    /** @brief Build + emit the low-rate ExtendedSystemState (~10 Hz): the slow/bulky
     *         state (the 4 thermocouples; later, event timestamps) the GS does not
     *         need thousands of times a second. ALWAYS downlinked to the GS;
     *         additionally logged to data_ext.bin while PersistingData is set
     *         (regardless of Fast/Slow). Foreground-driven; self-throttled. */
    void produceExtended(uint32_t now_ms)
    {
        if ((now_ms - last_extended_ms_) < detail::EXTENDED_INTERVAL_MS) {
            return;
        }
        last_extended_ms_ = now_ms;

        FcuExtendedSystemState ext = {};
        ext.creation_timestamp_ms   = now_ms;
        ext.control_flags           = logic::control::control_flags.raw();  // live recording config for the GS
        ext.last_refused_state_from = static_cast<uint8_t>(logic::control::last_refused_transition.from);
        ext.last_refused_state_to   = static_cast<uint8_t>(logic::control::last_refused_transition.to);
        const auto thermocouples = thermocouples_.info();
        for (std::size_t i = 0; i < THERMOCOUPLE_COUNT; ++i) {
            ext.thermocouple_info[i] = thermocouples[i];
        }

        const std::span<const uint8_t> bytes(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext));
        recorder_.recordExtended(bytes);   // -> data_ext.bin (gated by PersistingData)
        comm_.sendToGs(BoardId::FillingStation, PayloadType::Telemetry,
                       static_cast<uint8_t>(TelemetryType::ExtendedSystemState),
                       static_cast<uint8_t>(logic::control::persistent_state.fill_state),
                       /*seq=*/0, bytes, now_ms);
    }

    /** @brief Feed one inbound CAN frame to the ECU-telemetry reassembler; when a full
     *         EcuSystemState rebuilds, BATCH it for relay to the GS (tagged BoardId::Engine
     *         so the GS demuxes it from our own records). The batch is flushed by
     *         flushRelayedEcu() at the end of the tick — at the ECU's full 2 kHz this folds
     *         a tick's worth of records into one datagram instead of one UDP packet each.
     *         Non-telemetry frames are ignored by the reassembler. */
    void relayEcuFrame(const logic::communication::CanFrame& frame, uint32_t now_ms)
    {
        if (auto record = ecu_reassembler_.accept(frame)) {
            std::memcpy(ecu_relay_buf_.data() + ecu_relay_len_, &*record, sizeof(EcuSystemState));
            ecu_relay_len_ += sizeof(EcuSystemState);
            // Flush as soon as the batch can't take another whole record (one datagram's worth);
            // the rest of the tick's records start a fresh batch.
            if (ecu_relay_len_ + sizeof(EcuSystemState) > ecu_relay_buf_.size()) {
                flushRelayedEcu(now_ms);
            }
        }
    }

    /** @brief Relay any batched ECU records to the GS as a single datagram (tagged
     *         BoardId::Engine), then empty the batch. Called from the controller each tick
     *         after CAN ingress, so a burst drained in one tick rides one datagram while a
     *         lone record still goes out promptly (same tick). */
    void flushRelayedEcu(uint32_t now_ms)
    {
        if (ecu_relay_len_ == 0) {
            return;
        }
        downlink(BoardId::Engine, /*sourceState (ECU state not yet in the record)*/ 0,
                 std::span<const uint8_t>(ecu_relay_buf_.data(), ecu_relay_len_),
                 sizeof(EcuSystemState), now_ms);
        ecu_relay_len_ = 0;
    }

private:
    // Stream a run of fixed-size telemetry records to the GS, batched so each
    // datagram carries only whole records (the GS never sees a split one). The wire
    // framing (header + CRC + send) is the communication layer's job; the batching
    // is ours, because record size is a telemetry-stream concern.
    void downlink(BoardId sourceId, uint8_t sourceState,
                  std::span<const uint8_t> records, std::size_t record_size, uint32_t now_ms)
    {
        const std::size_t per_packet  = Comm::GS_PAYLOAD_CAPACITY / record_size;
        const std::size_t batch_bytes = per_packet * record_size;
        for (std::size_t off = 0; off < records.size(); off += batch_bytes) {
            const std::size_t chunk =
                records.size() - off < batch_bytes ? records.size() - off : batch_bytes;
            comm_.sendToGs(sourceId, PayloadType::Telemetry,
                           static_cast<uint8_t>(TelemetryType::SystemState),
                           sourceState, /*seq=*/0, records.subspan(off, chunk), now_ms);
        }
    }

    // Build an FcuSystemState from the ADC's info record (a fresh conversion on the
    // sample path, or a synthesized Faulted record on the watchdog fallback path).
    FcuSystemState buildSystemState(const AdcInfo& adc, uint32_t now_ms)
    {
        FcuSystemState state = {};
        state.base.creation_timestamp_ms = now_ms;

        // The ADC owns its record (state + status + channels) — the silent/fault
        // condition now lives in adc_info, not in sdCardFlags.
        state.base.adc_info = adc;

        // The SD card owns its record too: state (Init/Active/Error) + status bits
        // (incl. the last error cause). The recorder reports the WORST of its three
        // files, so a write failure on any stream surfaces — not just data_fast's.
        state.base.storage_info = recorder_.health();

        // The CAN bus and Ethernet link each report their own state + status + drop
        // count, read through the communication layer that owns them.
        state.base.can_info = comm_.canInfo();
        state.eth_info      = comm_.ethInfo();

        // The double-buffer overrun (a half filled before it could be flushed, so
        // records were dropped) is a logging-pipeline fault, not the card's own —
        // surface it on the SD card's status record (it owns its interface flags now).
        state.base.storage_info.status.write_overrun = log_.overrun ? 1 : 0;

        // The FCU's own valves report their own info, indexed by the FcuValves SSOT.
        state.base.valve_info[static_cast<std::size_t>(FcuValves::Fill)] = fill_valve_.info();
        state.base.valve_info[static_cast<std::size_t>(FcuValves::Dump)] = dump_valve_.info();

        // The thermocouples are NOT here — being ~10 Hz they ride the low-rate
        // FcuExtendedSystemState (produceExtended), not this 2 kHz record.
        return state;
    }

    // Append one record to the active half. Single-producer: every record comes from
    // produce() in the record-timer ISR. When a half fills it is marked ready for
    // drain(); if the other half is still unflushed the record is dropped and overrun
    // is flagged.
    void logAppend(const FcuSystemState& record)
    {
        uint8_t a = log_.active;
        if (log_.used[a] + sizeof(FcuSystemState) > detail::LOG_HALF_BYTES) {
            log_.ready[a] = true;        // finalize this half
            a ^= 1;
            if (log_.ready[a]) {         // consumer hasn't drained it yet
                log_.overrun = true;
                return;
            }
            log_.active  = a;
            log_.used[a] = 0;
        }
        std::memcpy(&log_.data[a][log_.used[a]], &record, sizeof(FcuSystemState));
        log_.used[a] = static_cast<uint16_t>(log_.used[a] + sizeof(FcuSystemState));
    }

    logic::telemetry::SdRecorder<S, FcuSystemState> recorder_;  // the 3-file SD recording policy
    V&    fill_valve_;   // injected Fill / Dump valves, read for telemetry
    V&    dump_valve_;
    A&    adc_;         // injected streaming ADC; produce() drains its ring
    Comm& comm_;        // injected communication layer; frames + downlinks records
    TC&   thermocouples_;  // injected MAX31856 bank; serviced off-ISR, read into the extended record
    detail::LogBuffer log_;            // .axisram in firmware; left uninitialised until init()
    volatile uint32_t  last_adc_ms_ = 0;  // last tick a conversion was drained; gates the silent-ADC filler
    uint32_t           last_extended_ms_ = 0;  // throttles produceExtended() to ~10 Hz
    logic::communication::can::SystemStateReassembler ecu_reassembler_;  // rebuilds ECU telemetry from CAN

    // Batches reassembled ECU records so they relay to the GS as datagrams (like our own
    // telemetry) instead of one UDP packet per record — at the ECU's full 2 kHz that would be
    // thousands of tiny packets a second. Holds up to one datagram's worth of whole records;
    // filled + flushed entirely from the main-loop tick (no double buffer needed).
    static constexpr std::size_t ECU_RELAY_BATCH_RECORDS =
        Comm::GS_PAYLOAD_CAPACITY / sizeof(EcuSystemState);
    std::array<uint8_t, ECU_RELAY_BATCH_RECORDS * sizeof(EcuSystemState)> ecu_relay_buf_;
    std::size_t ecu_relay_len_ = 0;   // bytes currently batched (a whole number of records)

    static_assert(std::extent_v<decltype(SystemStateBase::valve_info)> == 2,
                  "FcuSystemState expects exactly two valves (Fill, Dump)");
};

} // namespace logic::fcu
