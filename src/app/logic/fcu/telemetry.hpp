#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

#include "storage/interfaces/storage.hpp"        // logic::storage::Storage + StorageInfo
#include "actuation/interfaces/valve.hpp"        // logic::actuation::Valve
#include "actuation/interfaces/ematch.hpp"       // logic::actuation::Ematch (read into the extended record)
#include "actuation/interfaces/solenoid.hpp"     // logic::actuation::Solenoid (read into the extended record)
#include "actuation/interfaces/heater.hpp"       // logic::actuation::Heater (read into the extended record)
#include "communication/interfaces/adc.hpp"      // logic::communication::StreamingAdc + AdcInfo
#include "communication/interfaces/thermocouple.hpp"  // logic::communication::ThermocoupleBank + ThermocoupleInfo
#include "communication/interfaces/power_monitor.hpp"  // logic::communication::PowerMonitor + PowerMonitorInfo
#include "control/persistent_state.hpp"          // Backup-SRAM state snapshot (drain tags the source state)
#include "control/control_flags.hpp"             // fcu_control_flags — the FCU per-board flags byte
#include "telemetry/extended_base.hpp"           // logic::telemetry::fillExtendedBase (shared prefix)
#include "telemetry/sd_recorder.hpp"             // logic::telemetry::SdRecorder (shared 3-file SD policy)

#include "communication/protocol/framing/payload_type.hpp"        // PayloadType
#include "communication/protocol/framing/can_header.hpp"          // CanHeader (decode the relayed ECU state)
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
    volatile uint16_t overrun_count;  // halves dropped because the other was still unflushed (saturating)
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
 * @tparam PM logic::communication::PowerMonitor (the INA3221 on I2C4).
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, typename Comm,
          logic::communication::ThermocoupleBank TC, logic::communication::PowerMonitor PM,
          logic::actuation::Ematch EM, logic::actuation::Solenoid SOL, logic::actuation::Heater HTR>
class Telemetry {
public:
    /** @brief Construct over the held drivers + the communication layer. The three
     *         SD streams are the high-rate SystemState (fast/slow, picked by the
     *         FastRecording flag) and the low-rate ExtendedSystemState. The e-match,
     *         solenoid and heater are read-only here: their info() rides the extended record. */
    Telemetry(S& storage_fast, S& storage_slow, S& storage_ext,
              V& fill_valve, V& dump_valve, A& adc, Comm& comm, TC& thermocouples, PM& power_monitor,
              EM& ematch, SOL& solenoid, HTR& heater)
        : recorder_(storage_fast, storage_slow, storage_ext),
          fill_valve_(fill_valve), dump_valve_(dump_valve),
          adc_(adc), comm_(comm), thermocouples_(thermocouples), power_monitor_(power_monitor),
          ematch_(ematch), solenoid_(solenoid), heater_(heater) {}

    /** @brief Zero the double buffer and bring the three SD log files online. */
    void init()
    {
        // Both double buffers live in NOLOAD .axisram — clear them (and their indices)
        // before the producers start: our own records (log_) and the relayed ECU
        // records (ecu_log_).
        clearLog(log_);
        clearLog(ecu_log_);
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

    /** @brief Advance the power monitor's non-blocking acquisition one step. Driven from the
     *         foreground loop (NOT the record-timer ISR), since it talks to I2C4; it self-paces
     *         (~10 Hz) and never blocks. The latest reading is folded into the extended record
     *         by produceExtended. */
    void servicePowerMonitor(uint32_t now_ms) { power_monitor_.service(now_ms); }

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
        // Shared prefix: timestamp + base control flags + this board's per-board (FCU) flags +
        // the refused-command diagnostics (SetState + SetControlFlag, with counts).
        logic::telemetry::fillExtendedBase(ext.base, now_ms, logic::control::fcu_control_flags.raw());
        const auto thermocouples = thermocouples_.info();
        for (std::size_t i = 0; i < THERMOCOUPLE_COUNT; ++i) {
            ext.thermocouple_info[i] = thermocouples[i];
        }
        ext.power_monitor = power_monitor_.info();   // INA3221 (I2C4), polled at ~10 Hz
        ext.ematch_info   = ematch_.info();          // presence + firing-line state + last energise/deenergise ticks
        ext.solenoid_info = solenoid_.info();        // presence + open/closed state + last open/close ticks
        ext.heater_info   = heater_.info();          // on/off state + last on/off ticks

        const std::span<const uint8_t> bytes(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext));
        recorder_.recordExtended(bytes);   // -> data_ext.bin (gated by PersistingData)
        comm_.sendToGs(BoardId::FillingStation, PayloadType::Telemetry,
                       static_cast<uint8_t>(TelemetryType::ExtendedSystemState),
                       static_cast<uint8_t>(logic::control::persistent_state.fill_state),
                       /*seq=*/0, bytes, now_ms);
    }

    /** @brief Feed one inbound CAN frame to the ECU-telemetry reassemblers and relay whatever
     *         completes to the GS (tagged BoardId::Engine so the GS demuxes it from our own
     *         records, and carrying the ECU's state from the fragment header in
     *         EthernetHeader.sender_state — read the same way the GS reads ours). The two ECU
     *         record streams relay differently:
     *           - the high-rate EcuSystemState is BATCHED — appended to the relay double buffer
     *             and streamed to the GS only once a half FILLS (drainRelayedEcu), so the 2 kHz
     *             stream rides full datagrams instead of one tiny packet per record;
     *           - the low-rate EcuExtendedSystemState is relayed UNBATCHED — streamed to the GS
     *             the instant a record reassembles (at ~10 Hz a datagram per record is cheap, and
     *             the slow state reaches the ground without waiting on a relay half to fill).
     *         Each reassembler ignores the other's fragments (payload_id discriminates), and
     *         non-telemetry frames are ignored by both. */
    void relayEcuFrame(const logic::communication::CanFrame& frame, uint32_t now_ms)
    {
        if (auto record = ecu_reassembler_.accept(frame)) {
            // All fragments share the header, so the completing frame's carries the record's state.
            CanHeader header;
            header.code = frame.id;
            ecu_relay_state_ = static_cast<uint8_t>(header.frame.sender_state);

            ecuLogAppend(*record);
        }
        if (auto record = ecu_extended_reassembler_.accept(frame)) {
            CanHeader header;
            header.code = frame.id;
            const std::span<const uint8_t> bytes(
                reinterpret_cast<const uint8_t*>(&*record), sizeof(*record));
            comm_.sendToGs(BoardId::Engine, PayloadType::Telemetry,
                           static_cast<uint8_t>(TelemetryType::ExtendedSystemState),
                           static_cast<uint8_t>(header.frame.sender_state),
                           /*seq=*/0, bytes, now_ms);
        }
    }

    /** @brief Stream any FULL relay half to the GS (tagged BoardId::Engine), then release it —
     *         the relay counterpart of drain(). Called from the controller each tick; a half
     *         only goes out once filled, so the ECU's 2 kHz stream rides a few full datagrams
     *         instead of one tiny packet per record. */
    void drainRelayedEcu(uint32_t now_ms)
    {
        for (uint8_t h = 0; h < 2; ++h) {
            if (!ecu_log_.ready[h]) {
                continue;
            }
            downlink(BoardId::Engine, ecu_relay_state_,
                     std::span<const uint8_t>(ecu_log_.data[h], ecu_log_.used[h]),
                     sizeof(EcuSystemState), now_ms);
            ecu_log_.ready[h] = false;
        }
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

        // The double-buffer overrun count (halves that filled before they could be flushed,
        // so records were dropped) is a logging-pipeline fault, not the card's own — surface
        // it on the SD card's record (it owns its interface health now). A running count, not
        // a sticky flag, so a one-off boot stall reads differently from a sustained shortfall.
        state.base.storage_info.overrun_count = log_.overrun_count;

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
                if (log_.overrun_count != UINT16_MAX) {
                    log_.overrun_count = static_cast<uint16_t>(log_.overrun_count + 1);  // dropped half (saturating)
                }
                return;
            }
            log_.active  = a;
            log_.used[a] = 0;
        }
        std::memcpy(&log_.data[a][log_.used[a]], &record, sizeof(FcuSystemState));
        log_.used[a] = static_cast<uint16_t>(log_.used[a] + sizeof(FcuSystemState));
    }

    // Append one reassembled ECU record to the relay double buffer's active half (single
    // producer: relayEcuFrame on the main-loop CAN ingress). When a half fills it is marked
    // ready for drainRelayedEcu(); if the other half is still unsent the record is dropped and
    // overrun is flagged. Mirrors logAppend for the FCU's own records.
    void ecuLogAppend(const EcuSystemState& record)
    {
        uint8_t a = ecu_log_.active;
        if (ecu_log_.used[a] + sizeof(EcuSystemState) > detail::LOG_HALF_BYTES) {
            ecu_log_.ready[a] = true;        // finalize this half
            a ^= 1;
            if (ecu_log_.ready[a]) {         // consumer hasn't drained it yet
                if (ecu_log_.overrun_count != UINT16_MAX) {
                    ecu_log_.overrun_count = static_cast<uint16_t>(ecu_log_.overrun_count + 1);  // dropped half (saturating)
                }
                return;
            }
            ecu_log_.active  = a;
            ecu_log_.used[a] = 0;
        }
        std::memcpy(&ecu_log_.data[a][ecu_log_.used[a]], &record, sizeof(EcuSystemState));
        ecu_log_.used[a] = static_cast<uint16_t>(ecu_log_.used[a] + sizeof(EcuSystemState));
    }

    // Zero a telemetry double buffer and reset its indices — used at init() for both the
    // FCU's own log and the relayed-ECU log.
    static void clearLog(detail::LogBuffer& b)
    {
        std::memset(b.data, 0, sizeof(b.data));
        b.used[0]  = 0;
        b.used[1]  = 0;
        b.ready[0] = false;
        b.ready[1] = false;
        b.active   = 0;
        b.overrun_count = 0;
    }

    logic::telemetry::SdRecorder<S, FcuSystemState> recorder_;  // the 3-file SD recording policy
    V&    fill_valve_;   // injected Fill / Dump valves, read for telemetry
    V&    dump_valve_;
    A&    adc_;         // injected streaming ADC; produce() drains its ring
    Comm& comm_;        // injected communication layer; frames + downlinks records
    TC&   thermocouples_;  // injected MAX31856 bank; serviced off-ISR, read into the extended record
    PM&   power_monitor_;  // injected INA3221; serviced off-ISR, read into the extended record
    EM&   ematch_;         // injected e-match (read-only here); its info() rides the extended record
    SOL&  solenoid_;       // injected solenoid valve (read-only here); its info() rides the extended record
    HTR&  heater_;         // injected heater (read-only here); its info() rides the extended record
    detail::LogBuffer log_;            // .axisram in firmware; left uninitialised until init()
    volatile uint32_t  last_adc_ms_ = 0;  // last tick a conversion was drained; gates the silent-ADC filler
    uint32_t           last_extended_ms_ = 0;  // throttles produceExtended() to ~10 Hz
    logic::communication::can::SystemStateReassembler   ecu_reassembler_;           // rebuilds ECU SystemState from CAN (batched relay)
    logic::communication::can::ExtendedStateReassembler ecu_extended_reassembler_;  // rebuilds ECU ExtendedSystemState from CAN (relayed straight to the GS)

    // Relay double buffer: reassembled ECU records are appended to the active half, and a half
    // is streamed to the GS only once it FILLS (drainRelayedEcu) — the same batching as our own
    // telemetry (log_), so the ECU's 2 kHz stream rides a few full datagrams instead of one
    // tiny UDP packet per record. Filled + drained from the main-loop tick. Rides the
    // controller's .axisram placement like log_ (it does not need DMA-reachable memory).
    detail::LogBuffer ecu_log_;
    uint8_t           ecu_relay_state_ = 0;  // ECU state from the last reassembled record; tags the relay datagram

    static_assert(std::extent_v<decltype(SystemStateBase::valve_info)> == 2,
                  "FcuSystemState expects exactly two valves (Fill, Dump)");
};

} // namespace logic::fcu
