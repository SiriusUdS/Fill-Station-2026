#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

#include "storage/interfaces/storage.hpp"            // logic::storage::Storage + StorageInfo (injected dependency)
#include "actuation/interfaces/valve.hpp"            // logic::actuation::Valve + ValveState (injected)
#include "communication/interfaces/adc.hpp"          // logic::communication::StreamingAdc (injected)
#include "communication/interfaces/ethernet.hpp"     // logic::communication::udp + Endpoint
#include "communication/interfaces/can.hpp"          // logic::communication::can + CanFrame
#include "control/persistent_state.hpp"              // Backup-SRAM state snapshot
#include "dil/can_types.h"                           // HAL-free CAN protocol (CANHeader, enums)

#include "communication/protocol/ethernet/ethernet_header.hpp"  // EthernetHeader (downlink header)
#include "communication/protocol/telemetry/system_state.hpp"    // SystemState, ValveInfo, InterfaceFieldFlags
#include "communication/protocol/can/system_state_codec.hpp"    // SystemStateReassembler (ECU telemetry relay)
#include "fcu_valves.hpp"                  // FcuValves (valve identity / array index SSOT)
#include "command/command.hpp"            // CommandType (Ethernet payloadID)
#include "command/set_valve_position.hpp" // SetValvePositionFrame, ValveCommand

#include "sirius-headers-common/Ethernet/UDPFrame.h"
#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"
#include "sirius-headers-common/Telecommunication/BoardCommandV2.h"
#include "sirius-headers-common/FillingStation/FillingStationState.h"

/* ------------------------------------------------------------------------- *
 * FCU filling-station state machine (HAL-free), as a class template.
 *
 * Controller<S, V, A> is parameterised on its held drivers: the backing store S
 * (any logic::storage::Storage), the valve type V (logic::actuation::Valve, both
 * Fill and Dump), and the streaming ADC A (logic::communication::StreamingAdc).
 * It holds each by reference and calls it directly, so error handling is inline
 * and the contract is explicit. The ADC's DRDY ISR fills its ring; the controller
 * drains it on its own cadence in produceRecord() (driven by a dedicated record
 * timer), so capture is decoupled from the save/comms rate.
 *
 * Bring-up owns the one instance (Controller<SdCard, BallValve, Ads131m08>) and
 * the test owns Controller<Fake...>; the implementation lives in fcu_controller.tpp.
 *
 * The telemetry double buffer is a member (log_); in firmware the whole instance
 * is placed in D1 AXI-SRAM (see main.cpp) because the SD write hands a half
 * straight to the SDMMC DMA, which cannot reach DTCM.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

namespace detail {

using logic::communication::Endpoint;

inline constexpr uint32_t    RX_WATCHDOG_MS            = 500;
inline constexpr std::size_t REQUEST_STATE_OFFSET_BYTES = 15;  // requested state byte in the packet

/* Telemetry double buffer: each SystemState (one per ADC sample) is appended to
   the active 4096-byte half; when a half fills it is flushed to SD and streamed
   to the GS while the producer fills the other half. */
inline constexpr std::size_t LOG_HALF_BYTES = 4096;

/* If the ADC ring stays empty this long, the ADC is presumed silent and the
   record timer emits filler records (flagged invalid) so the packet rate holds. */
inline constexpr uint32_t ADC_TIMEOUT_MS = 10;   // > worst-case stall before declaring the ADC silent

/* Records per UDP datagram (EthernetHeader + records + CRC <= the link's UDP
   payload limit; keep UDP_MAX_PAYLOAD_BYTES in sync with the platform stack). */
inline constexpr std::size_t UDP_MAX_PAYLOAD_BYTES = 1432;
inline constexpr std::size_t ETH_RECORDS_PER_PACKET =
    (UDP_MAX_PAYLOAD_BYTES - sizeof(EthernetHeader) - sizeof(uint32_t)) / sizeof(SystemState);

/* Ground-station endpoint (heartbeat / telemetry destination). */
inline constexpr std::array<uint8_t, 6> GS_MAC  = {0x00, 0xE0, 0x4C, 0x33, 0x0F, 0x98};
inline constexpr uint16_t               GS_PORT = 7520;

inline constexpr uint32_t make_ipv4(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return (static_cast<uint32_t>(b1) << 24) | (static_cast<uint32_t>(b2) << 16) |
           (static_cast<uint32_t>(b3) << 8) | static_cast<uint32_t>(b4);
}

/* CRC32 (HAL-free, reflected poly 0xEDB88320 / zlib variant).
   TODO: confirm this matches the ground station's CRC32 variant. */
inline uint32_t crc32(const uint8_t* data, std::size_t length_bytes)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < length_bytes; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = ~(crc & 1U) + 1U;
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

/* The state-machine state itself is NOT held here: it lives in battery-backed
   Backup SRAM via logic::control::persistent_state, so it survives a reset.
   This struct holds only the volatile per-boot bookkeeping. */
struct FillStation {
    uint32_t current_tick_ms  = 0;
    uint32_t last_rx_ms       = 0;
    Endpoint gs;
};

/* The telemetry double buffer. A plain aggregate (no member initializers) so the
   controller's constructor does not touch its 8 KB at static-init; init() zeroes
   it. Pinned in D1 AXI-SRAM via the controller instance's placement in firmware. */
struct LogBuffer {
    uint8_t           data[2][LOG_HALF_BYTES];
    volatile uint16_t used[2];   // bytes filled in each half
    volatile bool     ready[2];  // half full, awaiting drain
    uint8_t           active;    // half the producer is filling (0/1)
    volatile bool     overrun;   // a half filled before the other was drained
};

} // namespace detail

/**
 * @brief The FCU filling-station controller, parameterised on its held drivers.
 * @tparam S A type modelling logic::storage::Storage (the SD card in firmware).
 * @tparam V A type modelling logic::actuation::Valve (a BallValve in firmware);
 *           both the Fill and Dump valves are of this type.
 * @tparam A A type modelling logic::communication::StreamingAdc (the ADS131M08).
 * @tparam E A type modelling logic::communication::Ethernet (the UDP link).
 * @tparam C A type modelling logic::communication::Can (the FDCAN bus).
 */
template <logic::storage::Storage S, logic::actuation::Valve V,
          logic::communication::StreamingAdc A, logic::communication::Ethernet E,
          logic::communication::Can C>
class Controller {
public:
    /** @brief Construct over the held drivers; does not touch hardware. Call init() next. */
    Controller(S& storage, V& fill_valve, V& dump_valve, A& adc, E& eth, C& can)
        : storage_(storage), fill_valve_(fill_valve), dump_valve_(dump_valve),
          adc_(adc), eth_(eth), can_(can) {}

    /**
     * @brief  Initialise the FCU logic: starting state, ground-station endpoint,
     *         telemetry buffer, and bring the backing store online. Call once
     *         after the platform peripherals (Ethernet, CAN) are up.
     */
    void init();

    /**
     * @brief  Advance the FCU one step: drain CAN and UDP, run the state machine,
     *         emit the heartbeat and service the receive watchdog. Record
     *         production is on its own timer (produceRecord), not here.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void tick(uint32_t now_ms);

    /**
     * @brief  Produce telemetry record(s) — the comms/save cadence, driven by a
     *         dedicated timer (NOT the ADC). Drains every conversion queued in the
     *         ADC's ring (each a fresh record); if the ring is empty and the ADC
     *         has timed out, emits one filler with the last-known data flagged
     *         invalid (Faulted / !data_valid). Runs in the timer ISR — keep it off
     *         the SD and Ethernet paths.
     * @param  now_ms  Current millisecond tick (e.g. HAL_GetTick()).
     */
    void produceRecord(uint32_t now_ms);

private:
    void        sendValveCmd(uint8_t valve, uint8_t cmd);
    void        canTick();
    void        handleStateRequest(std::span<const uint8_t> payload);
    void        handleSetValvePosition(std::span<const uint8_t> payload);
    void        handleDatagram(std::span<const uint8_t> payload);
    void        rxTick();
    SystemState buildSystemState(const AdcInfo& adc, uint32_t now_ms);
    void        logAppend(const SystemState& record);
    void        watchdogTick();
    // Single GS egress: batch a run of SystemState records into UDP datagrams tagged
    // with their source board id + state. Used for both the FCU's own records and the
    // reassembled ECU records relayed off the CAN bus.
    void        sendToGs(uint8_t sourceId, uint8_t sourceState, std::span<const uint8_t> records);
    void        drainTick();

    S&                  storage_;        // injected backing store, used as a Storage explicitly
    V&                  fill_valve_;      // injected Fill / Dump valves, read for telemetry
    V&                  dump_valve_;
    A&                  adc_;            // injected streaming ADC; produceRecord() drains its ring
    E&                  eth_;            // injected UDP link; rx in rxTick(), tx in sendBatched()
    C&                  can_;            // injected CAN bus; drained in canTick(), tx in sendValveCmd()
    detail::FillStation fill_{};
    detail::LogBuffer   log_;            // .axisram in firmware; left uninitialised until init()
    volatile uint32_t   last_adc_ms_ = 0;  // last tick a conversion was drained; gates the silent-ADC filler
    logic::communication::can::SystemStateReassembler ecu_reassembler_;  // rebuilds ECU telemetry from CAN

    static_assert(std::extent_v<decltype(SystemState::valve_info)> == 2,
                  "SystemState expects exactly two valves (Fill, Dump)");
};

} // namespace logic::fcu

#include "fcu_controller.tpp"
