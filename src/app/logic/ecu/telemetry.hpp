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
#include "communication/interfaces/can.hpp"      // logic::communication::CanFrame
#include "communication/interfaces/power_monitor.hpp"  // logic::communication::PowerMonitor + PowerMonitorInfo
#include "control/persistent_state.hpp"          // fill_state — tags each downlinked record with the ECU's state
#include "telemetry/sd_recorder.hpp"             // logic::telemetry::SdRecorder (shared 3-file SD policy)
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
 * It owns the telemetry double buffer (log_), turns each ADC conversion into an
 * EcuSystemState record (produce), and flushes full halves to SD AND downlinks every
 * record in the half onto the CAN bus (drain): the SD and the CAN downlink both carry
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

/* Telemetry double buffer half size — shared with the SD recorder, which writes a
   whole half verbatim to data_fast.bin. */
inline constexpr std::size_t LOG_HALF_BYTES = logic::telemetry::SD_LOG_BLOCK_BYTES;

/* If the ADC ring stays empty this long, the ADC is presumed silent and the record
   timer emits filler records (flagged invalid) so the downlink rate holds. */
inline constexpr uint32_t ADC_TIMEOUT_MS = 10;

/* ExtendedSystemState cadence (low-rate; for now control flags + refused transition +
   power monitor, later event timestamps). Logged to data_ext.bin from the foreground AND
   downlinked to the FCU over CAN once per record (unbatched), so the FCU can relay the
   ECU's slow state to the GS the moment it lands. */
inline constexpr uint32_t EXTENDED_INTERVAL_MS = 100;   // ~10 Hz

/* The telemetry double buffer — a plain aggregate (no member initializers) so the
   constructor does not touch its 8 KB at static-init; init() zeroes it. Pinned in
   D1 AXI-SRAM via the controller instance's placement in firmware. */
struct LogBuffer {
    uint8_t           data[2][LOG_HALF_BYTES];
    volatile uint16_t used[2];   // bytes filled in each half
    volatile bool     ready[2];  // half full, awaiting drain
    uint8_t           active;    // half the producer is filling (0/1)
    volatile uint16_t overrun_count;  // halves dropped because the other was still unflushed (saturating)
};

} // namespace detail

/**
 * @brief The ECU telemetry pipeline, parameterised on its held drivers + the
 *        communication layer it downlinks through.
 * @tparam S logic::storage::Storage (the SD card in firmware).
 * @tparam V logic::actuation::Valve (read for telemetry; both IPA and NOS).
 * @tparam A logic::communication::StreamingAdc (the ADS131M08).
 * @tparam Comm The ECU Communication layer (frames + sends to the FCU).
 * @tparam PM logic::communication::PowerMonitor (the INA3221 on I2C4; stubbed on the current PCB).
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

    /** @brief Zero the double buffer and bring the three SD log files online. */
    void init()
    {
        std::memset(log_.data, 0, sizeof(log_.data));
        log_.used[0]  = 0;
        log_.used[1]  = 0;
        log_.ready[0] = false;
        log_.ready[1] = false;
        log_.active   = 0;
        log_.overrun_count = 0;
        last_adc_ms_  = 0;

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

    /** @brief Flush any full half: SD-log the SystemState through the shared recorder
     *         (raw data_fast.bin vs 125 Hz averaged data_slow.bin, per the flags), and
     *         downlink EVERY record in the half to the FCU over CAN (always, full-rate —
     *         the CAN stream is the ECU's live link, independent of SD recording mode, and
     *         now carries the same 2 kHz the SD log does). This only fills the driver's
     *         software TX ring and returns; the ring paces the burst onto the bus in the
     *         background (TX-FIFO-empty ISR), so the loop never waits on the wire. */
    void drain(uint32_t /*now_ms*/)
    {
        for (uint8_t h = 0; h < 2; ++h) {
            if (!log_.ready[h]) {
                continue;
            }
            const uint16_t bytes = log_.used[h];

            recorder_.recordSystemState(
                std::span<const uint8_t>(log_.data[h], detail::LOG_HALF_BYTES), bytes);

            // Hand off every whole record in the half (one FD frame each). The platform
            // CAN driver queues them in its software TX ring, so this is a run of cheap
            // RAM copies, not a blocking wait on the ~24 ms it takes to clock the burst out.
            for (std::size_t off = 0; off + sizeof(EcuSystemState) <= bytes;
                 off += sizeof(EcuSystemState)) {
                EcuSystemState rec;
                std::memcpy(&rec, &log_.data[h][off], sizeof(EcuSystemState));
                sendRecordCan(rec);
            }
            log_.ready[h] = false;
        }
    }

    /** @brief Build the low-rate ExtendedSystemState (~10 Hz), log it to data_ext.bin
     *         (gated by PersistingData), AND downlink it to the FCU over CAN — one record
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
        ext.power_monitor = power_monitor_.info();  // INA3221 (stubbed on the current PCB)
        recorder_.recordExtended(
            std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(&ext), sizeof(ext)));
        sendExtendedCan(ext);   // downlink to the FCU (unbatched), which relays it to the GS
    }

    /** @brief Advance the power monitor's non-blocking acquisition one step (off the record-timer
     *         ISR). A no-op while the driver is stubbed; the latest reading is folded into the
     *         extended record by produceExtended. */
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

    // Append one record to the active half (single-producer: produce() in the record-
    // timer ISR). When a half fills it is marked ready for drain(); if the other half
    // is still unflushed the record is dropped and overrun is flagged.
    void logAppend(const EcuSystemState& record)
    {
        uint8_t a = log_.active;
        if (log_.used[a] + sizeof(EcuSystemState) > detail::LOG_HALF_BYTES) {
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
        std::memcpy(&log_.data[a][log_.used[a]], &record, sizeof(EcuSystemState));
        log_.used[a] = static_cast<uint16_t>(log_.used[a] + sizeof(EcuSystemState));
    }

    // Fragment one EcuSystemState into CAN frames (shared codec) and send them to the
    // FCU through the communication layer.
    void sendRecordCan(const EcuSystemState& record)
    {
        namespace codec = logic::communication::can;
        std::array<logic::communication::CanFrame, codec::SYSTEM_STATE_FRAGMENTS> frames;
        codec::packSystemState(
            record, BoardId::Engine, BoardId::FillingStation,
            static_cast<uint8_t>(logic::control::persistent_state.fill_state), telemetry_seq_,
            std::span<logic::communication::CanFrame, codec::SYSTEM_STATE_FRAGMENTS>(frames));
        for (const auto& f : frames) {
            comm_.sendFrame(f);
        }
        ++telemetry_seq_;
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
    PM&   power_monitor_;  // injected INA3221 (stubbed on the current PCB); read into the extended record
    detail::LogBuffer log_;            // .axisram in firmware; left uninitialised until init()
    volatile uint32_t  last_adc_ms_      = 0;  // last tick a conversion was drained; gates the silent-ADC filler
    uint32_t           last_extended_ms_ = 0;  // throttles produceExtended() to ~10 Hz
    uint8_t            telemetry_seq_     = 0;  // 4-bit CAN SystemState record sequence (wraps)
    uint8_t            extended_seq_      = 0;  // 4-bit CAN ExtendedSystemState record sequence (wraps)

    static_assert(std::extent_v<decltype(SystemStateBase::valve_info)> == 2,
                  "EcuSystemState expects exactly two valves (IPA, NOS)");
};

} // namespace logic::ecu
