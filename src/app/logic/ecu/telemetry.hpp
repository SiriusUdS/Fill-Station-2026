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
#include "control/control_flags.hpp"             // control_flags — PersistingData gates the SD write

#include "communication/protocol/telemetry/ecu_system_state.hpp"  // EcuSystemState (+ SystemStateBase)
#include "communication/system_state_codec.hpp"  // packSystemState (CAN fragments)
#include "system/valves/ecu.hpp"                                  // EcuValves (valve identity / array index SSOT)
#include "system/board_id.hpp"

/* ------------------------------------------------------------------------- *
 * ECU telemetry pipeline (HAL-free) — the record-production + downlink half of
 * the ECU: the ADC and the SD card, plus the live downlink to the FCU over CAN.
 * The CAN-only sibling of logic::fcu::Telemetry (no Ethernet egress).
 *
 * It owns the telemetry double buffer (log_), turns each ADC conversion into an
 * EcuSystemState record (produce), and flushes full halves to SD while downsampling
 * one record per drained half onto the CAN bus (drain): the SD gets the full-rate
 * log, the bus cannot (a record is many frames), so the CAN downlink is a downsampled
 * live stream the FCU relays to the ground station. It speaks through the injected
 * Communication layer; the shared SystemState codec does the CAN fragmentation.
 *
 * In firmware the owning controller instance is placed in D1 AXI-SRAM (see main.cpp)
 * so the SD write can hand a buffer half straight to the SDMMC DMA.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

namespace detail {

/* Telemetry double buffer half size (sector-aligned SD writes). */
inline constexpr std::size_t LOG_HALF_BYTES = 4096;

/* If the ADC ring stays empty this long, the ADC is presumed silent and the record
   timer emits filler records (flagged invalid) so the downlink rate holds. */
inline constexpr uint32_t ADC_TIMEOUT_MS = 10;

/* The telemetry double buffer — a plain aggregate (no member initializers) so the
   constructor does not touch its 8 KB at static-init; init() zeroes it. Pinned in
   D1 AXI-SRAM via the controller instance's placement in firmware. */
struct LogBuffer {
    uint8_t           data[2][LOG_HALF_BYTES];
    volatile uint16_t used[2];   // bytes filled in each half
    volatile bool     ready[2];  // half full, awaiting drain
    uint8_t           active;    // half the producer is filling (0/1)
    volatile bool     overrun;   // a half filled before the other was drained
};

} // namespace detail

/**
 * @brief The ECU telemetry pipeline, parameterised on its held drivers + the
 *        communication layer it downlinks through.
 * @tparam S logic::storage::Storage (the SD card in firmware).
 * @tparam V logic::actuation::Valve (read for telemetry; both IPA and NOS).
 * @tparam A logic::communication::StreamingAdc (the ADS131M08).
 * @tparam Comm The ECU Communication layer (frames + sends to the FCU).
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, typename Comm>
class Telemetry {
public:
    /** @brief Construct over the held drivers + the communication layer. */
    Telemetry(S& storage, V& ipa_valve, V& nos_valve, A& adc, Comm& comm)
        : storage_(storage), ipa_valve_(ipa_valve), nos_valve_(nos_valve),
          adc_(adc), comm_(comm) {}

    /** @brief Zero the double buffer and bring the backing store online. */
    void init()
    {
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

    /** @brief Flush any full half: persist the 4096-byte block to SD (only while the
     *         PersistingData control flag is set — otherwise the half drains unwritten
     *         to spare the card), and downlink the half's most recent record to the
     *         FCU over CAN. */
    void drain(uint32_t /*now_ms*/)
    {
        for (uint8_t h = 0; h < 2; ++h) {
            if (!log_.ready[h]) {
                continue;
            }
            const uint16_t bytes = log_.used[h];

            // Save only when told to: the buffer always drains (the half is released
            // below regardless), but it reaches the card only while PersistingData is on.
            if (logic::control::control_flags.get(ControlFlag::PersistingData)) {
                storage_.write(std::span<const uint8_t>(log_.data[h], detail::LOG_HALF_BYTES));
            }
            if (bytes >= sizeof(EcuSystemState)) {
                EcuSystemState latest;
                std::memcpy(&latest, &log_.data[h][bytes - sizeof(EcuSystemState)], sizeof(EcuSystemState));
                sendRecordCan(latest);
            }
            log_.ready[h] = false;
        }
    }

private:
    // Build an EcuSystemState from the ADC's info record (a fresh conversion on the
    // sample path, or a synthesized Faulted record on the silent-ADC fallback path).
    EcuSystemState buildSystemState(const AdcInfo& adc, uint32_t now_ms)
    {
        EcuSystemState state = {};
        state.base.creation_timestamp_ms = now_ms;

        // Each peripheral OWNS its telemetry record; the pipeline only reads it.
        state.base.adc_info     = adc;
        state.base.storage_info = storage_.info();
        state.base.can_info     = comm_.canInfo();
        // The ECU has no Ethernet peripheral, so EcuSystemState carries no eth_info.

        // The double-buffer overrun (records dropped because a half could not be
        // flushed in time) is a logging-pipeline fault — surface it on the SD card's
        // own status record (it owns its interface flags).
        state.base.storage_info.status.write_overrun = log_.overrun ? 1 : 0;

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
                log_.overrun = true;
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
            record, BoardId::Engine, BoardId::FillingStation, telemetry_seq_,
            std::span<logic::communication::CanFrame, codec::SYSTEM_STATE_FRAGMENTS>(frames));
        for (const auto& f : frames) {
            comm_.sendFrame(f);
        }
        ++telemetry_seq_;
    }

    S&    storage_;     // injected backing store, used as a Storage explicitly
    V&    ipa_valve_;    // injected IPA / NOS valves, read for telemetry
    V&    nos_valve_;
    A&    adc_;         // injected streaming ADC; produce() drains its ring
    Comm& comm_;        // injected communication layer; frames + downlinks records
    detail::LogBuffer log_;            // .axisram in firmware; left uninitialised until init()
    volatile uint32_t  last_adc_ms_   = 0;  // last tick a conversion was drained; gates the silent-ADC filler
    uint8_t            telemetry_seq_ = 0;  // 4-bit CAN telemetry record sequence (wraps)

    static_assert(std::extent_v<decltype(SystemStateBase::valve_info)> == 2,
                  "EcuSystemState expects exactly two valves (IPA, NOS)");
};

} // namespace logic::ecu
