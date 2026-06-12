#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

#include "storage/interfaces/storage.hpp"        // logic::storage::Storage + StorageInfo (injected)
#include "actuation/interfaces/valve.hpp"        // logic::actuation::Valve + ValveState (injected)
#include "communication/interfaces/adc.hpp"      // logic::communication::StreamingAdc (injected)
#include "communication/interfaces/can.hpp"      // logic::communication::Can + CanFrame (injected)
#include "control/persistent_state.hpp"          // Backup-SRAM state snapshot (shared with the FCU)
#include "dil/can_types.h"                        // HAL-free CAN protocol (CANHeader, enums)

#include "communication/protocol/telemetry/system_state.hpp"  // SystemState, ValveInfo, InterfaceFieldFlags
#include "communication/protocol/can/system_state_codec.hpp"  // SystemState <-> CAN fragment codec (shared)
#include "ecu_valves.hpp"                          // EcuValves (valve identity / array index SSOT)

#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"  // ENGINE_BOARD_ID, board ids

/* ------------------------------------------------------------------------- *
 * ECU engine controller (HAL-free), as a class template — the CAN-side sibling
 * of logic::fcu::Controller.
 *
 * Controller<S, V, A, C> is parameterised on its held drivers: the backing store
 * S (logic::storage::Storage, the SD card), the valve type V (logic::actuation::
 * Valve, both IPA and NOS), the streaming ADC A (logic::communication::Streaming
 * Adc, the ADS131M08), and the CAN bus C (logic::communication::Can). It holds
 * each by reference and calls it directly.
 *
 * Unlike the FCU there is NO Ethernet: the ECU receives commands from the FCU over
 * CAN and downlinks its telemetry back over CAN (the FCU relays it to the GS). The
 * ADC's DRDY ISR fills its ring; the controller drains it on its own cadence in
 * produceRecord() (driven by a dedicated record timer), decoupled from capture.
 *
 * State for now is just Init -> Safe (shared logic::control::persistent_state); the
 * engine-specific state machine is added later. The telemetry double buffer is a
 * member (log_); in firmware the whole instance is placed in D1 AXI-SRAM (see
 * main.cpp) because the SD write hands a half straight to the SDMMC DMA.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

namespace detail {

/* Telemetry double buffer half size (sector-aligned SD writes). */
inline constexpr std::size_t LOG_HALF_BYTES = 4096;

/* If the ADC ring stays empty this long, the ADC is presumed silent and the record
   timer emits filler records (flagged invalid) so the downlink rate holds. */
inline constexpr uint32_t ADC_TIMEOUT_MS = 10;

/* Byte offset of the valve index in a CAN_ID_CMD_VALVE frame (mirrors the FCU's
   sendValveCmd: data[0..3] = timestamp, data[4] = valve index). */
inline constexpr std::size_t CMD_VALVE_INDEX_OFFSET = sizeof(uint32_t);

/* Volatile per-boot bookkeeping (the state itself lives in Backup SRAM via
   logic::control::persistent_state, so it survives a reset). */
struct Engine {
    uint32_t current_tick_ms = 0;
    uint32_t last_cmd_ms     = 0;   // last CAN command received from the FCU
};

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
 * @brief The ECU engine controller, parameterised on its held drivers.
 * @tparam S A type modelling logic::storage::Storage (the SD card in firmware).
 * @tparam V A type modelling logic::actuation::Valve (a BallValve in firmware);
 *           both the IPA and NOS valves are of this type.
 * @tparam A A type modelling logic::communication::StreamingAdc (the ADS131M08).
 * @tparam C A type modelling logic::communication::Can (the FDCAN bus to the FCU).
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C>
class Controller {
public:
    /** @brief Construct over the held drivers; does not touch hardware. Call init() next. */
    Controller(S& storage, V& ipa_valve, V& nos_valve, A& adc, C& can)
        : storage_(storage), ipa_valve_(ipa_valve), nos_valve_(nos_valve),
          adc_(adc), can_(can) {}

    /**
     * @brief  Initialise the ECU logic: starting state (Init -> Safe), telemetry
     *         buffer, and bring the backing store online. Call once after the
     *         platform peripherals (CAN, SD) are up.
     */
    void init();

    /**
     * @brief  Advance the ECU one step: drain CAN (commands from the FCU), drive
     *         the state machine, and flush full telemetry halves to SD + CAN.
     *         Record production is on its own timer (produceRecord), not here.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void tick(uint32_t now_ms);

    /**
     * @brief  Produce telemetry record(s) — the comms/save cadence, driven by a
     *         dedicated timer (NOT the ADC). Drains every conversion queued in the
     *         ADC's ring (each a fresh record); on a silent ADC emits one filler
     *         flagged invalid. Runs in the timer ISR — keep it off the SD/CAN paths.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void produceRecord(uint32_t now_ms);

private:
    void        canTick();                                   // drain CAN, dispatch FCU commands
    void        handleCanFrame(const logic::communication::CanFrame& frame);
    void        handleValveCmd(const logic::communication::CanFrame& frame, const CANHeader& header);
    void        handlePing(const logic::communication::CanFrame& frame, const CANHeader& header);
    SystemState buildSystemState(const AdcInfo& adc, uint32_t now_ms);
    void        logAppend(const SystemState& record);
    void        drainTick();                                 // flush full halves to SD + downlink over CAN
    void        sendRecordCan(const SystemState& record);    // fragment one record over CAN to the FCU

    S&                storage_;     // injected backing store, used as a Storage explicitly
    V&                ipa_valve_;   // injected IPA / NOS valves, commanded over CAN + read for telemetry
    V&                nos_valve_;
    A&                adc_;         // injected streaming ADC; produceRecord() drains its ring
    C&                can_;         // injected CAN bus; rx in canTick(), telemetry tx in drainTick()
    detail::Engine    engine_{};
    detail::LogBuffer log_;         // .axisram in firmware; left uninitialised until init()
    volatile uint32_t last_adc_ms_   = 0;  // last tick a conversion was drained; gates the silent-ADC filler
    uint8_t           telemetry_seq_ = 0;  // 4-bit CAN telemetry record sequence (wraps)

    static_assert(std::extent_v<decltype(SystemState::valve_info)> == 2,
                  "SystemState expects exactly two valves (IPA, NOS)");
};

} // namespace logic::ecu

#include "ecu_controller.tpp"
