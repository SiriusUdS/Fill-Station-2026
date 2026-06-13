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
#include "control/persistent_state.hpp"          // Backup-SRAM state snapshot (drain tags the source state)

#include "communication/protocol/framing/payload_type.hpp"        // PayloadType
#include "communication/protocol/telemetry/fcu_system_state.hpp"  // FcuSystemState
#include "communication/protocol/framing/system_state_codec.hpp"  // EcuSystemState + SystemStateReassembler
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
   the active 4096-byte half; when a half fills it is flushed to SD and streamed
   to the GS while the producer fills the other half. */
inline constexpr std::size_t LOG_HALF_BYTES = 4096;

/* If the ADC ring stays empty this long, the ADC is presumed silent and the
   record timer emits filler records (flagged invalid) so the packet rate holds. */
inline constexpr uint32_t ADC_TIMEOUT_MS = 10;   // > worst-case stall before declaring the ADC silent

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
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, typename Comm>
class Telemetry {
public:
    /** @brief Construct over the held drivers + the communication layer. */
    Telemetry(S& storage, V& fill_valve, V& dump_valve, A& adc, Comm& comm)
        : storage_(storage), fill_valve_(fill_valve), dump_valve_(dump_valve),
          adc_(adc), comm_(comm) {}

    /** @brief Zero the double buffer and bring the backing store online. */
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

        storage_.init();
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

    /** @brief Flush any full half: write the 4096-byte block to SD, stream its
     *         records to the GS, then release the half. */
    void drain(uint32_t now_ms)
    {
        for (uint8_t h = 0; h < 2; ++h) {
            if (!log_.ready[h]) {
                continue;
            }
            const uint16_t bytes = log_.used[h];

            storage_.write(std::span<const uint8_t>(log_.data[h], detail::LOG_HALF_BYTES));
            downlink(BoardId::FillingStation,
                     static_cast<uint8_t>(logic::control::persistent_state.fill_state),
                     std::span<const uint8_t>(log_.data[h], bytes), sizeof(FcuSystemState), now_ms);
            log_.ready[h] = false;
        }
    }

    /** @brief Feed one inbound CAN frame to the ECU-telemetry reassembler; when a
     *         full EcuSystemState rebuilds, relay it to the GS tagged BoardId::Engine
     *         so the GS demuxes it from our own records. Non-telemetry frames are
     *         ignored by the reassembler. */
    void relayEcuFrame(const logic::communication::CanFrame& frame, uint32_t now_ms)
    {
        if (auto record = ecu_reassembler_.accept(frame)) {
            const EcuSystemState ecu = *record;
            const std::span<const uint8_t> one(reinterpret_cast<const uint8_t*>(&ecu), sizeof(ecu));
            downlink(BoardId::Engine, /*sourceState (ECU state not yet in the record)*/ 0,
                     one, one.size(), now_ms);
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
                           sourceState, records.subspan(off, chunk), now_ms);
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
        // (incl. the last error cause). The GS reads health straight from here.
        state.base.storage_info = storage_.info();

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

    S&    storage_;     // injected backing store, used as a Storage explicitly
    V&    fill_valve_;   // injected Fill / Dump valves, read for telemetry
    V&    dump_valve_;
    A&    adc_;         // injected streaming ADC; produce() drains its ring
    Comm& comm_;        // injected communication layer; frames + downlinks records
    detail::LogBuffer log_;            // .axisram in firmware; left uninitialised until init()
    volatile uint32_t  last_adc_ms_ = 0;  // last tick a conversion was drained; gates the silent-ADC filler
    logic::communication::can::SystemStateReassembler ecu_reassembler_;  // rebuilds ECU telemetry from CAN

    static_assert(std::extent_v<decltype(SystemStateBase::valve_info)> == 2,
                  "FcuSystemState expects exactly two valves (Fill, Dump)");
};

} // namespace logic::fcu
