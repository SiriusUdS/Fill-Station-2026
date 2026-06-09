/**
  ******************************************************************************
  * @file    fcu_controller.cpp
  * @brief   FCU filling-station state machine. HAL-free logic built on the
  *          udp:: and can:: interfaces: polls received datagrams and CAN frames,
  *          runs the state machine, emits the heartbeat and the receive
  *          watchdog, and answers CAN ping/valve traffic.
  ******************************************************************************
  */

#include "fcu_controller.hpp"

#include "communication/interfaces/ethernet.hpp"   // logic::communication::udp + Endpoint
#include "communication/interfaces/can.hpp"          // logic::communication::can + CanFrame
#include "control/persistent_state.hpp"              // Backup-SRAM state snapshot
#include "dil/can_types.h"                            // HAL-free CAN protocol (CANHeader, enums)

#include "sirius-headers-common/Ethernet/UDPFrame.h"
#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"
#include "sirius-headers-common/Telecommunication/BoardCommandV2.h"
#include "sirius-headers-common/FillingStation/FillingStationState.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

namespace udp     = logic::communication::udp;
namespace can     = logic::communication::can;
namespace control = logic::control;
using logic::communication::CanFrame;
using logic::communication::Endpoint;

namespace {

constexpr uint32_t RX_WATCHDOG_MS      = 500;
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1;
constexpr std::size_t REQUEST_STATE_OFFSET_BYTES = 15;  // requested state byte in the packet

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
    uint32_t current_tick_ms = 0;
    uint32_t last_tx_ms     = 0;
    uint32_t last_rx_ms     = 0;
    Endpoint gs;
    uint32_t can_rx_count   = 0;
};

FillStation s_fill;

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

void handleCanFrame(const CanFrame& frame)
{
    CANHeader header;
    header.code = frame.id;

    switch (header.frame.messageID) {
        case CAN_ID_COMM_PING: {
            /* Echo the payload back to the sender as a PONG. */
            CANHeader resp = {};
            resp.frame.senderID  = FILLING_STATION_BOARD_ID;
            resp.frame.targetID  = header.frame.senderID;
            resp.frame.messageID = CAN_ID_COMM_PONG;

            CanFrame pong = frame;
            pong.id = resp.code;
            (void)can::send(pong);
            break;
        }
        case CAN_ID_STATUS_VALVE: {
            /* ValveStatus: valveIndex at byte 4, status in the header. */
            s_fill.can_rx_count++;
            /* TODO: feed the valve index/status into the state machine. */
            break;
        }
        default:
            break;
    }
}

void canTick()
{
    while (auto frame = can::receive()) {
        handleCanFrame(*frame);
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

/* ---- Heartbeat + watchdog ------------------------------------------------ */

void messageTick()
{
    const uint8_t state = static_cast<uint8_t>(control::persistent_state.fill_state);
    if (state == FILLING_STATION_STATE_UNSAFE ||
        state == FILLING_STATION_STATE_IGNITE) {
        if ((s_fill.current_tick_ms - s_fill.last_rx_ms) >= RX_WATCHDOG_MS) {
            control::persistent_state.saveState(control::State::Abort);
        }
    }

    if ((s_fill.current_tick_ms - s_fill.last_tx_ms) < HEARTBEAT_PERIOD_MS) {
        return;
    }

    UDPPacketHeader header = {};
    header.frame.deviceID      = FILLING_STATION_BOARD_ID;
    header.frame.deviceState   = static_cast<uint8_t>(control::persistent_state.fill_state);
    header.frame.deviceTS_MS   = s_fill.current_tick_ms;
    header.frame.payloadID     = GET_SYSTEM;
    header.frame.payloadLenght = 4;

    const std::array<uint8_t, 4> values = {0xDE, 0xAD, 0xBE, 0xEF};
    const uint32_t crc = crc32(values.data(), values.size());

    std::array<uint8_t, sizeof(UDPPacketHeader) + 8> buffer = {};
    std::memcpy(buffer.data(), header.bytes, sizeof(UDPPacketHeader));
    std::memcpy(buffer.data() + sizeof(UDPPacketHeader), values.data(), values.size());
    std::memcpy(buffer.data() + sizeof(UDPPacketHeader) + 4, &crc, sizeof(crc));

    (void)udp::send(s_fill.gs, buffer);
    s_fill.last_tx_ms = s_fill.current_tick_ms;
}

} // namespace

namespace logic::fcu {

void init()
{
    s_fill = FillStation{};
    s_fill.gs.mac  = GS_MAC;
    s_fill.gs.ipv4 = make_ipv4(192, 168, 0, 111);
    s_fill.gs.port = GS_PORT;

    // The state machine state lives in Backup SRAM. Resume it across a reset; on
    // a cold or corrupt boot, commit a fresh INIT so the blob is valid from here
    // on. (The platform inspects the same persistent_state before bringing the
    // valves up, to decide whether to skip the normal valve-moving init.)
    control::persistent_state.saveState(
        control::persistent_state.loadState().value_or(control::State::Init));
}

void tick(uint32_t now_ms)
{
    s_fill.current_tick_ms = now_ms;

    canTick();
    rxTick();
    messageTick();

    if (control::persistent_state.fill_state == control::State::Init) {
        control::persistent_state.saveState(control::State::Safe);
    }
}

} // namespace logic::fcu
