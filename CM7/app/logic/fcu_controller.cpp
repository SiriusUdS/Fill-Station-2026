/**
  ******************************************************************************
  * @file    fcu_controller.cpp
  * @brief   FCU filling-station state machine. HAL-free logic built on the udp::,
  *          can::, adc:: and storage:: interfaces: produces one SystemState per
  *          ADC sample into a double buffer, flushes full halves to the SD card
  *          and streams them to the ground station, runs the state machine, and
  *          enforces the receive / ADC watchdogs.
  ******************************************************************************
  */

#include "fcu_controller.hpp"

#include "communication/interfaces/ethernet.hpp"   // logic::communication::udp + Endpoint
#include "communication/interfaces/can.hpp"          // logic::communication::can + CanFrame
#include "communication/interfaces/adc.hpp"          // logic::communication::adc (system telemetry)
#include "storage/storage.hpp"                        // logic::storage::Storage (SD logging)
#include "control/persistent_state.hpp"              // Backup-SRAM state snapshot
#include "dil/can_types.h"                            // HAL-free CAN protocol (CANHeader, enums) — used by sendValveCmd (outbound, TBD)

#include "communication/protocol/ethernet/ethernet_header.hpp"  // EthernetHeader (downlink header)
#include "communication/protocol/telemetry/system_state.hpp"    // SystemState (GET_SYSTEM payload)

#include "sirius-headers-common/Ethernet/UDPFrame.h"
#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"
#include "sirius-headers-common/Telecommunication/BoardCommandV2.h"
#include "sirius-headers-common/FillingStation/FillingStationState.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

namespace udp     = logic::communication::udp;
namespace can     = logic::communication::can;
namespace adc     = logic::communication::adc;
namespace storage = logic::storage;
namespace control = logic::control;
using logic::communication::CanFrame;
using logic::communication::Endpoint;

namespace {

constexpr uint32_t RX_WATCHDOG_MS = 500;
constexpr std::size_t REQUEST_STATE_OFFSET_BYTES = 15;  // requested state byte in the packet

/* The FCU's own two valves, in telemetry order (s_fill.valves / valve_states). */
constexpr std::size_t VALVE_FILL = 0;
constexpr std::size_t VALVE_DUMP = 1;

/* Telemetry double buffer: each SystemState (one per ADC sample) is appended to
   the active 4096-byte half; when a half fills it is flushed to SD and streamed
   to the GS while the producer fills the other half. */
constexpr std::size_t LOG_HALF_BYTES = 4096;

/* If no ADC sample arrives for this long, the ADC is presumed dead and the main
   loop produces fallback records (flagged) so logging/telemetry keep flowing. */
constexpr uint32_t ADC_TIMEOUT_MS     = 10;   // > worst-case main-loop stall (SD flush)
constexpr uint32_t FALLBACK_PERIOD_MS = 1;    // fallback record cadence while ADC is down

/* Records per UDP datagram (EthernetHeader + records + CRC <= the link's UDP
   payload limit; keep UDP_MAX_PAYLOAD_BYTES in sync with the platform stack). */
constexpr std::size_t UDP_MAX_PAYLOAD_BYTES = 1432;
constexpr std::size_t ETH_RECORDS_PER_PACKET =
    (UDP_MAX_PAYLOAD_BYTES - sizeof(EthernetHeader) - sizeof(uint32_t)) / sizeof(SystemState);

/* Ground-station endpoint (heartbeat / telemetry destination). */
constexpr std::array<uint8_t, 6> GS_MAC      = {0x00, 0xE0, 0x4C, 0x33, 0x0F, 0x98};
constexpr uint16_t               GS_PORT     = 7520;

constexpr uint32_t make_ipv4(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return (static_cast<uint32_t>(b1) << 24) | (static_cast<uint32_t>(b2) << 16) |
           (static_cast<uint32_t>(b3) << 8) | static_cast<uint32_t>(b4);
}

/* The state-machine state itself is NOT held here: it lives in battery-backed
   Backup SRAM via logic::control::persistentState(), so it survives a reset.
   This struct holds only the volatile per-boot bookkeeping. */
struct FillStation {
    uint32_t current_tick_ms  = 0;
    uint32_t last_rx_ms       = 0;
    uint32_t last_fallback_ms = 0;
    Endpoint gs;
    // The FCU's two valves (index VALVE_FILL / VALVE_DUMP) in telemetry form.
    // Source of truth, copied into every record. Populated by the valve command
    // handling (TBD); zero (status/value 0) until then.
    ValveState valves[2] = {};
};

FillStation s_fill;

/* Storage seam handle (single SD card on SDMMC2). */
storage::Storage s_storage;

static_assert(adc::CHANNEL_COUNT <= std::extent_v<decltype(SystemState::raw_adc_values)>,
              "more ADC channels than SystemState::raw_adc_values slots");
static_assert(std::extent_v<decltype(SystemState::valve_states)> == 2,
              "SystemState expects exactly two valves (Fill, Dump)");

/* Set to s_fill.current_tick_ms on every ADC sample (ISR); the ADC watchdog uses
   it to detect a silent ADC. */
volatile uint32_t s_last_adc_ms = 0;

/* The telemetry double buffer. Pinned in D1 AXI-SRAM because drainTick() hands a
   half straight to the SD write, whose SDMMC DMA cannot reach DTCM. NOLOAD, so
   init() zeroes it. */
struct LogBuffer {
    uint8_t           data[2][LOG_HALF_BYTES];
    volatile uint16_t used[2];   // bytes filled in each half
    volatile bool     ready[2];  // half full, awaiting drain
    uint8_t           active;    // half the producer is filling (0/1)
    volatile bool     overrun;   // a half filled before the other was drained
};
__attribute__((section(".axisram"))) LogBuffer s_log;

/* CRC32 (HAL-free, reflected poly 0xEDB88320 / zlib variant).
   TODO: confirm this matches the ground station's CRC32 variant. */
uint32_t crc32(const uint8_t* data, std::size_t length_bytes)
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

/* ---- CAN ----------------------------------------------------------------- */

/* Frame and queue a valve command to the ECU. Kept internal: the state logic
   calls it when a valve transition is commanded. */
[[maybe_unused]] void sendValveCmd(uint8_t valve, uint8_t cmd)
{
    CANHeader header = {};
    header.frame.senderID    = FILLING_STATION_BOARD_ID;
    header.frame.targetID    = ENGINE_BOARD_ID;
    header.frame.deviceState = cmd;
    header.frame.messageID   = CAN_ID_CMD_VALVE;

    CanFrame frame;
    frame.id     = header.code;
    frame.length = static_cast<uint8_t>(frame.data.size());
    std::memcpy(frame.data.data(), &s_fill.current_tick_ms, sizeof(uint32_t));  // ValveCmd timestamp
    frame.data[sizeof(uint32_t)] = valve;                                        // ValveCmd valveIndex

    (void)can::send(frame);
}

/* The FCU receives only status/telemetry from the ECU over CAN — never commands,
   which always arrive over Ethernet. Drain the RX ring each tick so it cannot
   back up; feeding ECU valve status into the state machine is a later step. */
void canTick()
{
    while (auto frame = can::receive()) {
        (void)frame;  // TODO: consume ECU valve status / telemetry
    }
}

/* ---- UDP (ground-station commands) --------------------------------------- */

void handleStateRequest(const UDPPacketHeader& header, std::span<const uint8_t> payload)
{
    if (header.frame.payloadID != REQUEST_STATE) {
        return;
    }
    if (payload.size() <= REQUEST_STATE_OFFSET_BYTES) {
        return;
    }
    const uint8_t requested = payload[REQUEST_STATE_OFFSET_BYTES];

    switch (static_cast<uint8_t>(control::persistent_state.fill_state)) {
        case FILLING_STATION_STATE_SAFE:
            if (requested != FILLING_STATION_STATE_TEST &&
                requested != FILLING_STATION_STATE_UNSAFE) return;
            break;
        case FILLING_STATION_STATE_TEST:
            if (requested != FILLING_STATION_STATE_SAFE) return;
            break;
        case FILLING_STATION_STATE_UNSAFE:
            if (requested != FILLING_STATION_STATE_SAFE &&
                requested != FILLING_STATION_STATE_IGNITE &&
                requested != FILLING_STATION_STATE_ABORT) return;
            break;
        case FILLING_STATION_STATE_IGNITE:
            if (requested != FILLING_STATION_STATE_SAFE &&
                requested != FILLING_STATION_STATE_ABORT) return;
            break;
        case FILLING_STATION_STATE_ABORT:
            if (requested != FILLING_STATION_STATE_SAFE) return;
            break;
        default:
            return;
    }
    control::persistent_state.saveState(static_cast<control::State>(requested));
}

void handleDatagram(std::span<const uint8_t> payload)
{
    if (payload.size() < sizeof(UDPPacketHeader)) {
        return;
    }

    UDPPacketHeader header;
    std::memcpy(header.bytes, payload.data(), sizeof(UDPPacketHeader));
    s_fill.last_rx_ms = s_fill.current_tick_ms;

    const uint8_t device = header.frame.deviceID;
    if (device != FILLING_STATION_BOARD_ID && device != BOARD_BROADCAST_ID) {
        return;  /* not addressed to us (CAN-bridge routing TODO) */
    }

    /* The per-state command handlers are currently empty; only the default
       REQUEST_STATE transition handler acts on the packet. */
    handleStateRequest(header, payload);
}

void rxTick()
{
    udp::tick();  // service the link so receive() can return inbound datagrams
    while (auto datagram = udp::receive()) {
        handleDatagram(datagram->payload);
    }
}

/* ---- Telemetry production (per ADC sample) -------------------------------- */

// Build a SystemState from a set of channel counts. adc_ok == false (the
// fallback path) leaves the channels zero and flags the ADC fault in erno.
SystemState buildSystemState(std::span<const int32_t> channels, bool adc_ok)
{
    constexpr std::size_t SLOTS = std::extent_v<decltype(SystemState::raw_adc_values)>;

    SystemState state = {};
    state.frameTs_MS         = s_fill.current_tick_ms;
    state.lastHandshakeTs_MS = s_fill.last_rx_ms;

    const std::size_t n = channels.size() < SLOTS ? channels.size() : SLOTS;
    for (std::size_t i = 0; i < n; ++i) {
        state.raw_adc_values[i] = static_cast<uint32_t>(channels[i]);  // bit-for-bit
    }

    // Logging / data health into sdCardFlags so the GS sees it per record:
    //   initialized  — SD mounted and ready
    //   writingError — the double buffer overran, so some records were dropped
    //   readingError — this record is a fallback (ADC silent; channels stale/zero)
    // TODO: readingError is the ADC fault parked here for now; move it to a
    // dedicated ADC/device flag set when InterfaceField gets per-device flags.
    InterfaceFieldFlags sd = {};
    sd.bits.initialized  = (s_storage.status().bits.state == STORAGE_STATE_ACTIVE) ? 1 : 0;
    sd.bits.writingError = s_log.overrun ? 1 : 0;
    sd.bits.readingError = adc_ok ? 0 : 1;
    state.interfaces.frame.sdCardFlags = sd;

    // The FCU's own valves, reported in telemetry order (Fill, Dump).
    state.valve_states[VALVE_FILL] = s_fill.valves[VALVE_FILL];
    state.valve_states[VALVE_DUMP] = s_fill.valves[VALVE_DUMP];

    return state;
}

// Append one record to the active half. Single-producer: the ADC ISR, or the
// fallback while the ADC is down — never both at once. When a half fills it is
// marked ready for drainTick(); if the other half is still unflushed the record
// is dropped and overrun is flagged.
void logAppend(const SystemState& record)
{
    uint8_t a = s_log.active;
    if (s_log.used[a] + sizeof(SystemState) > LOG_HALF_BYTES) {
        s_log.ready[a] = true;        // finalize this half
        a ^= 1;
        if (s_log.ready[a]) {         // consumer hasn't drained it yet
            s_log.overrun = true;
            return;
        }
        s_log.active  = a;
        s_log.used[a] = 0;
    }
    std::memcpy(&s_log.data[a][s_log.used[a]], &record, sizeof(SystemState));
    s_log.used[a] = static_cast<uint16_t>(s_log.used[a] + sizeof(SystemState));
}

// ADC per-sample callback (ISR context, ~2 kHz): produce a record from the fresh
// channels and append it. Kept short — no SD/Ethernet here.
void onAdcSample(std::span<const int32_t> channels)
{
    s_last_adc_ms = s_fill.current_tick_ms;
    logAppend(buildSystemState(channels, /*adc_ok=*/true));
}

/* ---- Watchdogs ------------------------------------------------------------ */

// Abort if the ground-station link goes quiet while armed.
void watchdogTick()
{
    const uint8_t state = static_cast<uint8_t>(control::persistent_state.fill_state);
    if (state == FILLING_STATION_STATE_UNSAFE || state == FILLING_STATION_STATE_IGNITE) {
        if ((s_fill.current_tick_ms - s_fill.last_rx_ms) >= RX_WATCHDOG_MS) {
            control::persistent_state.saveState(control::State::Abort);
        }
    }
}

// If the ADC has gone silent, keep the pipeline alive: produce fallback records
// (no fresh ADC data, ADC fault flagged) at a gated cadence from the main loop.
void adcWatchdogTick()
{
    if ((s_fill.current_tick_ms - s_last_adc_ms) <= ADC_TIMEOUT_MS) {
        return;  // ADC producing normally
    }
    if ((s_fill.current_tick_ms - s_fill.last_fallback_ms) < FALLBACK_PERIOD_MS) {
        return;
    }
    s_fill.last_fallback_ms = s_fill.current_tick_ms;
    logAppend(buildSystemState({}, /*adc_ok=*/false));
}

/* ---- Telemetry drain (SD + Ethernet) -------------------------------------- */

// Split a run of records into UDP datagrams (EthernetHeader + records + CRC) and
// send them to the GS.
void sendBatched(std::span<const uint8_t> records)
{
    static std::array<uint8_t,
        sizeof(EthernetHeader) + ETH_RECORDS_PER_PACKET * sizeof(SystemState) + sizeof(uint32_t)>
        packet;

    constexpr std::size_t batch_bytes = ETH_RECORDS_PER_PACKET * sizeof(SystemState);
    for (std::size_t off = 0; off < records.size(); off += batch_bytes) {
        const std::size_t chunk =
            records.size() - off < batch_bytes ? records.size() - off : batch_bytes;

        EthernetHeader header = {};
        header.deviceID      = FILLING_STATION_BOARD_ID;
        header.payloadID     = GET_SYSTEM;
        header.payloadLenght = static_cast<uint16_t>(chunk);
        header.deviceState   = static_cast<uint8_t>(control::persistent_state.fill_state);
        header.deviceTS_MS   = s_fill.current_tick_ms;

        const uint32_t crc = crc32(records.data() + off, chunk);
        std::memcpy(packet.data(), &header, sizeof(header));
        std::memcpy(packet.data() + sizeof(header), records.data() + off, chunk);
        std::memcpy(packet.data() + sizeof(header) + chunk, &crc, sizeof(crc));

        (void)udp::send(s_fill.gs,
            std::span<const uint8_t>(packet.data(), sizeof(header) + chunk + sizeof(crc)));
    }
}

// Flush any full half: write the 4096-byte block to SD (sector-aligned) and
// stream its records to the GS, then release the half.
void drainTick()
{
    for (uint8_t h = 0; h < 2; ++h) {
        if (!s_log.ready[h]) {
            continue;
        }
        const uint16_t bytes = s_log.used[h];

        // TEMPORARY (remove later): stamp a marker in the tail of the 4096-byte
        // block (past the records, so it is not in the Ethernet send) to make SD
        // chunk boundaries easy to spot when reading runtime.bin.
        static constexpr char SD_CHUNK_MARKER[] = "END_OF_SD_CHUNK";
        constexpr std::size_t MARKER_LEN = sizeof(SD_CHUNK_MARKER) - 1;  // drop the NUL
        std::memcpy(&s_log.data[h][LOG_HALF_BYTES - MARKER_LEN], SD_CHUNK_MARKER, MARKER_LEN);

        s_storage.write(std::span<const uint8_t>(s_log.data[h], LOG_HALF_BYTES));
        sendBatched(std::span<const uint8_t>(s_log.data[h], bytes));
        s_log.ready[h] = false;
    }
}

} // namespace

namespace logic::fcu {

void init()
{
    s_fill = FillStation{};
    s_fill.gs.mac  = GS_MAC;
    s_fill.gs.ipv4 = make_ipv4(192, 168, 0, 111);
    s_fill.gs.port = GS_PORT;

    // The telemetry double buffer lives in NOLOAD .axisram — clear it and its
    // indices before the producer starts.
    std::memset(s_log.data, 0, sizeof(s_log.data));
    s_log.used[0]  = 0;
    s_log.used[1]  = 0;
    s_log.ready[0] = false;
    s_log.ready[1] = false;
    s_log.active   = 0;
    s_log.overrun  = false;
    s_last_adc_ms  = 0;

    // The state machine state lives in Backup SRAM. Resume it across a reset; on
    // a cold or corrupt boot, commit a fresh INIT so the blob is valid from here
    // on. (The platform inspects the same persistent_state before bringing the
    // valves up, to decide whether to skip the normal valve-moving init.)
    control::persistent_state.saveState(
        control::persistent_state.loadState().value_or(control::State::Init));

    // Bring the SD card online, then start producing one SystemState per ADC
    // sample into the double buffer (the callback runs in the ADC ISR).
    s_storage.init();
    adc::set_sample_callback(&onAdcSample);
}

void tick(uint32_t now_ms)
{
    s_fill.current_tick_ms = now_ms;

    canTick();
    rxTick();
    adcWatchdogTick();   // fallback production if the ADC is silent
    drainTick();         // flush full halves to SD + the GS
    watchdogTick();      // GS-link abort watchdog

    if (control::persistent_state.fill_state == control::State::Init) {
        control::persistent_state.saveState(control::State::Safe);
    }
}

} // namespace logic::fcu
