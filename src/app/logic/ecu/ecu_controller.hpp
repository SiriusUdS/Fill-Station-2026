#pragma once

#include <cstdint>

#include "storage/interfaces/storage.hpp"        // logic::storage::Storage
#include "actuation/interfaces/valve.hpp"        // logic::actuation::Valve
#include "communication/interfaces/adc.hpp"      // logic::communication::StreamingAdc
#include "communication/interfaces/can.hpp"      // logic::communication::Can
#include "communication/interfaces/power_monitor.hpp"  // logic::communication::PowerMonitor
#include "control/persistent_state.hpp"          // Backup-SRAM state snapshot (shared with the FCU)
#include "system/state.hpp"

#include "communication.hpp"   // logic::ecu::Communication<C>
#include "telemetry.hpp"       // logic::ecu::Telemetry<S,V,A,Comm>
#include "control.hpp"         // logic::ecu::Control<V,Comm>

/* ------------------------------------------------------------------------- *
 * ECU engine controller (HAL-free) — the orchestrator. The CAN-side sibling of
 * logic::fcu::Controller: it holds the three concerns and wires them together,
 * owning no peripheral logic of its own.
 *
 *   Communication<C>  — the only owner of the CAN bus (to/from the FCU): framing,
 *                       routing, raw transport. Consumed by the other two.
 *   Telemetry<S,V,A>  — ADC + SD: produce records, flush to SD, downlink (fragmented)
 *                       to the FCU over CAN.
 *   Control<V>        — command handling + execution: decode, dispatch, actuate the
 *                       propellant valves, answer pings.
 *
 * Unlike the FCU there is NO Ethernet: the ECU receives commands from the FCU over
 * CAN and downlinks its telemetry back over CAN (the FCU relays it to the ground
 * station). The orchestrator's only real work is draining CAN and handing each frame
 * to Control (every inbound frame is a command — the ECU never receives telemetry).
 *
 * Public surface is unchanged from the former monolithic controller (init / tick /
 * produceRecord), so bring-up (Controller<SdCard, BallValve, Ads131m08, Can>) builds
 * it exactly as before. State for now is just Init -> Safe (shared logic::control::
 * persistent_state); the engine-specific state machine is added later. In firmware the
 * whole instance is placed in D1 AXI-SRAM (see main.cpp) because Telemetry's SD write
 * hands a buffer half straight to the SDMMC DMA, which cannot reach DTCM.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

/**
 * @brief The ECU engine controller, parameterised on its held drivers.
 * @tparam S A type modelling logic::storage::Storage (the SD card in firmware).
 * @tparam V A type modelling logic::actuation::Valve (a BallValve in firmware);
 *           both the IPA and NOS valves are of this type.
 * @tparam A A type modelling logic::communication::StreamingAdc (the ADS131M08).
 * @tparam C A type modelling logic::communication::Can (the FDCAN bus to the FCU).
 * @tparam PM A type modelling logic::communication::PowerMonitor (the INA3221 on I2C4).
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Can C,
          logic::communication::PowerMonitor PM>
class Controller {
public:
    /** @brief Construct over the held drivers; does not touch hardware. Call init() next.
     *         The three SD streams (data_fast/slow/ext.bin) use the same shared recording
     *         policy as the FCU; which SystemState file is written is the FastRecording flag's. */
    Controller(S& storage_fast, S& storage_slow, S& storage_ext,
               V& ipa_valve, V& nos_valve, A& adc, C& can, PM& power_monitor)
        : comm_(can),
          telemetry_(storage_fast, storage_slow, storage_ext, ipa_valve, nos_valve, adc, comm_,
                     power_monitor),
          control_(ipa_valve, nos_valve, comm_) {}

    /**
     * @brief  Initialise the ECU logic: resume the persisted state, then the telemetry
     *         buffer + backing store and the control layer. Call once after the platform
     *         peripherals (CAN, SD) are up.
     */
    void init()
    {
        // Resume the persisted state first (shared mechanism + policy with the FCU):
        // resumeBootState() resumes ONLY the in-progress states a reset must not silently
        // restart (Abort / Launch / Ignite); any other saved state, and a cold or corrupt
        // boot, commits a fresh INIT (advanced to Safe below).
        (void)logic::control::persistent_state.resumeBootState();

        telemetry_.init();   // zeroes the double buffer and mounts the SD card
        control_.init();

        // As soon as init is done, leave Init for Safe (cold boot), routed through the single
        // transition point so onTransition runs (both propellant valves driven closed). A
        // resumed engine/armed state from Backup SRAM is left as-is — only a fresh Init advances.
        if (logic::control::persistent_state.fill_state == logic::control::State::Init) {
            (void)control_.transitionTo(logic::control::State::Safe, 0);  // boot transition; t=0
        }
    }

    /**
     * @brief  Advance the ECU one step: drain CAN (commands from the FCU) and dispatch
     *         each, flush full telemetry halves to SD + CAN, and drive the state machine.
     *         Record production is on its own timer (produceRecord), not here.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void tick(uint32_t now_ms)
    {
        while (auto frame = comm_.receive()) {   // every inbound frame is an FCU command
            control_.onCommand(*frame, now_ms);
        }

        telemetry_.servicePowerMonitor(now_ms);  // advance the non-blocking INA3221 round-robin
        telemetry_.produceExtended(now_ms);  // ~10 Hz ExtendedSystemState -> data_ext.bin
        telemetry_.drain(now_ms);   // flush full halves to SD + downlink over CAN
    }

    /**
     * @brief  Produce telemetry record(s) — the comms/save cadence, driven by a
     *         dedicated timer (NOT the ADC). Runs in the timer ISR; delegates to the
     *         telemetry pipeline. Keep it off the SD and CAN paths.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void produceRecord(uint32_t now_ms) { telemetry_.produce(now_ms); }

private:
    Communication<C>                         comm_;   // declared first: the others hold a ref to it
    Telemetry<S, V, A, Communication<C>, PM> telemetry_;
    Control<V, Communication<C>>             control_;
};

} // namespace logic::ecu
