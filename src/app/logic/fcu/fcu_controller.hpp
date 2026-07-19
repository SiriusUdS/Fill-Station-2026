#pragma once

#include <cstdint>

#include "storage/interfaces/storage.hpp"        // logic::storage::Storage
#include "actuation/interfaces/valve.hpp"        // logic::actuation::Valve
#include "communication/interfaces/adc.hpp"      // logic::communication::StreamingAdc
#include "communication/interfaces/thermocouple.hpp"  // logic::communication::ThermocoupleBank
#include "communication/interfaces/power_monitor.hpp"  // logic::communication::PowerMonitor
#include "communication/interfaces/ethernet.hpp" // logic::communication::Ethernet
#include "communication/interfaces/can.hpp"      // logic::communication::Can
#include "actuation/interfaces/ematch.hpp"       // logic::actuation::Ematch (the FCU igniter seam)
#include "actuation/interfaces/solenoid.hpp"     // logic::actuation::Solenoid (the FCU solenoid-valve seam)
#include "actuation/interfaces/heater.hpp"       // logic::actuation::Heater (the FCU heater seam)
#include "control/persistent_state.hpp"          // Backup-SRAM state snapshot

#include "communication/protocol/framing/can_header.hpp"      // CanHeader (inbound demux)
#include "communication/protocol/framing/payload_type.hpp"    // PayloadType
#include "communication/protocol/response/response_type.hpp"  // ResponseType (Pong)
#include "system/state.hpp"

#include "communication.hpp"   // logic::fcu::Communication<E,C>
#include "telemetry.hpp"       // logic::fcu::Telemetry<S,V,A,Comm>
#include "control.hpp"         // logic::fcu::Control<V,Comm>

/* ------------------------------------------------------------------------- *
 * FCU controller (HAL-free) — the orchestrator. It holds the three concerns and
 * wires them together; it owns no peripheral logic of its own:
 *
 *   Communication<E,C>  — the only owner of Ethernet (GS) + CAN (ECU): framing,
 *                         routing, CRC, raw transport. Consumed by the other two.
 *   Telemetry<S,V,A>    — ADC + SD: produce records, flush to SD, downlink to GS,
 *                         relay the ECU's telemetry on.
 *   Control<V>          — command handling + execution: parse, gate, dispatch,
 *                         actuate local valves, forward to the ECU.
 *
 * The orchestrator's one piece of real work is ingress routing: it drains the
 * transports and routes each inbound frame to the right consumer (a Pong from the
 * ECU -> Control; ECU telemetry -> Telemetry; GS commands -> Control). Routing is
 * here, not in Communication, because it means knowing about both consumers — and
 * Communication must not depend on them.
 *
 * Public surface is unchanged from the former monolithic controller (init / tick /
 * produceRecord), so bring-up (Controller<SdCard, BallValve, Ads131m08, ...>) and
 * the tests (Controller<Fake...>) construct it exactly as before. In firmware the
 * whole instance is placed in D1 AXI-SRAM (see main.cpp) because Telemetry's SD
 * write hands a buffer half straight to the SDMMC DMA, which cannot reach DTCM.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

/**
 * @brief The FCU filling-station controller, parameterised on its held drivers.
 * @tparam S logic::storage::Storage (the SD card in firmware).
 * @tparam V logic::actuation::Valve (a BallValve in firmware; both Fill and Dump).
 * @tparam A logic::communication::StreamingAdc (the ADS131M08).
 * @tparam E logic::communication::Ethernet (the UDP link to the GS).
 * @tparam C logic::communication::Can (the FDCAN bus to the ECU).
 * @tparam TC logic::communication::ThermocoupleBank (the 2 MAX31856 on SPI6).
 * @tparam PM logic::communication::PowerMonitor (the INA3221 on I2C4).
 * @tparam EM logic::actuation::Ematch (the e-match igniter line; 3 GPIOs on GPIOD).
 * @tparam SOL logic::actuation::Solenoid (the solenoid valve; 1 GPIO).
 * @tparam HTR logic::actuation::Heater (the main and tank heaters; 1 GPIO each).
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C, logic::communication::ThermocoupleBank TC,
          logic::communication::PowerMonitor PM, logic::actuation::Ematch EM,
          logic::actuation::Solenoid SOL, logic::actuation::Heater HTR>
class Controller {
public:
    /** @brief Construct over the held drivers; does not touch hardware. Call init() next.
     *         The three SD streams are the raw (data_fast.bin) and 125 Hz averaged
     *         (data_slow.bin) SystemState plus the low-rate ExtendedSystemState
     *         (data_ext.bin); which SystemState file is written is the FastRecording flag's. */
    Controller(S& storage_fast, S& storage_slow, S& storage_ext,
               V& fill_valve, V& dump_valve, A& adc, E& eth, C& can, TC& thermocouples,
               PM& power_monitor, EM& ematch, SOL& solenoid, HTR& heater, HTR& heater_tank)
        : comm_(eth, can),
          telemetry_(storage_fast, storage_slow, storage_ext, fill_valve, dump_valve, adc, comm_,
                     thermocouples, power_monitor, ematch, solenoid, heater, heater_tank),
          control_(fill_valve, dump_valve, comm_, ematch, solenoid, heater, heater_tank) {}

    /**
     * @brief  Initialise the FCU logic: communication endpoint, telemetry buffer +
     *         backing store, control liveness clock, and resume the persisted state.
     *         Call once after the platform peripherals (Ethernet, CAN) are up.
     */
    void init()
    {
        // Resume the persisted state FIRST — it lives in Backup SRAM and must be valid before
        // Control's init runs, because Control safes the local valves against it. Policy
        // (resumeBootState): resume ONLY the in-progress states a reset must not silently
        // restart (Abort / Launch / Ignite); any other saved state, and a cold or corrupt
        // boot, commits a fresh INIT (advanced to Safe below).
        (void)logic::control::persistent_state.resumeBootState();

        // Control next, so the actuators reach their boot position before the slower link / SD
        // bring-up below. Control::init drives the state machine into the boot state: a cold
        // boot safes every actuator and enters Safe; a resumed Abort/Launch/Ignite RE-EXECUTES
        // its entry transition (rules bypassed) so the actuators are driven to match the resumed
        // state. t=0 at boot.
        control_.init(0);

        comm_.init();
        telemetry_.init();   // zeroes the double buffer and mounts the SD card
    }

    /**
     * @brief  Advance the FCU one step: drain CAN and UDP and route each frame, and
     *         flush full telemetry halves. Record production is on its own timer
     *         (produceRecord), not here.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void tick(uint32_t now_ms)
    {
        // CAN ingress: an ECU response (Pong answering a forwarded ping, or Ack answering any
        // bridged command) -> Control to match + relay; everything else is ECU telemetry ->
        // Telemetry to reassemble + relay.
        while (auto in = comm_.receiveFrame()) {
            if (isResponse(in->header)) {
                control_.onResponse(static_cast<uint8_t>(in->header.frame.payload_id),
                                    static_cast<uint8_t>(in->header.frame.seq), now_ms);
            } else {
                telemetry_.relayEcuFrame(in->frame, now_ms);
            }
        }
        // Stream any relay half that filled this tick to the GS — drained like our own
        // telemetry, so only full halves go out and the ECU stream rides full datagrams
        // instead of one tiny packet per record.
        telemetry_.drainRelayedEcu(now_ms);

        // UDP ingress: ground-station commands.
        comm_.tick();  // service the link so receiveDatagram() can return inbound traffic
        while (auto datagram = comm_.receiveDatagram()) {
            control_.onDatagram(datagram->payload, now_ms);
        }

        telemetry_.serviceThermocouples(now_ms);  // advance the non-blocking MAX31856 round-robin
        telemetry_.servicePowerMonitor(now_ms);   // advance the non-blocking INA3221 round-robin
        control_.serviceEmatch();        // sample EMATCH_DET -> continuity LED (and the extended record)
        control_.serviceSolenoid(now_ms);  // enforce open-only-in-Unsafe on the coil line
        control_.serviceHeaters(now_ms);    // drive each heater on/off straight from its own flag
        control_.serviceWatchdog();      // feed the IWDG while in Safe (comm loss is not a fault there)
        telemetry_.produceExtended(now_ms);  // ~10 Hz ExtendedSystemState -> GS + data_ext.bin
        telemetry_.drain(now_ms);        // flush full halves to SD + the GS
        control_.servicePending(now_ms); // resend / time out the in-flight reliable command
    }

    /**
     * @brief  Produce telemetry record(s) — the comms/save cadence, driven by a
     *         dedicated timer (NOT the ADC). Runs in the timer ISR; delegates to the
     *         telemetry pipeline. Keep it off the SD and Ethernet paths.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void produceRecord(uint32_t now_ms) { telemetry_.produce(now_ms); }

private:
    // A response from the ECU on CAN (Response payload type): a Pong answering a forwarded
    // ping, or an Ack answering any bridged command. Control matches it to the in-flight
    // command by seq and relays it to the GS.
    [[nodiscard]] static bool isResponse(const CanHeader& header)
    {
        return static_cast<PayloadType>(header.frame.payload_type) == PayloadType::Response;
    }

    Communication<E, C>                                           comm_;   // declared first: the others hold a ref to it
    Telemetry<S, V, A, Communication<E, C>, TC, PM, EM, SOL, HTR> telemetry_;
    Control<V, Communication<E, C>, EM, SOL, HTR>                 control_;
};

} // namespace logic::fcu
