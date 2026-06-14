#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "communication/interfaces/ethernet.hpp"   // logic::communication::Ethernet + Endpoint + Datagram
#include "communication/interfaces/can.hpp"         // logic::communication::Can + CanFrame

#include "communication/protocol/framing/ethernet_header.hpp"  // EthernetHeader
#include "communication/protocol/framing/can_header.hpp"       // CanHeader
#include "communication/protocol/framing/payload_type.hpp"     // PayloadType
#include "system/board_id.hpp"                                 // BoardId
#include "data_integrity/crc32.hpp"                            // logic::data_integrity::crc32 (GS frame CRC)

/* ------------------------------------------------------------------------- *
 * FCU communication layer (HAL-free) — the single owner of the two transports
 * (Ethernet to the ground station, CAN to the ECU). Telemetry and Control hold
 * a reference to one of these and speak through it; nobody else touches eth_ or
 * can_ directly.
 *
 * It owns the wire MECHANICS — header construction, the GS endpoint, board-id
 * routing, the frame CRC, and the raw transport calls — but NOT payload meaning.
 * `payload_id` is opaque routing bytes here: this layer never switches on what a
 * payload *is* (that is Telemetry's / Control's job). `payload_type` it does
 * know, because Command/Telemetry/Response is a transport framing class, not a
 * payload semantic.
 *
 * Inbound is deliberately minimal: it hands datagrams/frames up (decoding the CAN
 * header once) and lets the orchestrator route them — routing means knowing about
 * both consumers, and this layer must not depend on Telemetry or Control.
 * ------------------------------------------------------------------------- */

namespace logic::fcu {

namespace detail {

/* The link's UDP payload limit (EthernetHeader + payload + CRC must fit; keep in
   sync with the platform stack). */
inline constexpr std::size_t UDP_MAX_PAYLOAD_BYTES = 1432;

inline constexpr uint32_t make_ipv4(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return (static_cast<uint32_t>(b1) << 24) | (static_cast<uint32_t>(b2) << 16) |
           (static_cast<uint32_t>(b3) << 8) | static_cast<uint32_t>(b4);
}

/* The two ground-station hosts the FCU talks to — DIFFERENT machines:
 *   - the telemetry sink receives all telemetry (our own FcuSystemState / Extended AND the
 *     ECU records we relay);
 *   - the commander sends us commands, and is where we return responses (Ack / Pong).
 * sendToGs() routes by payload class: Response -> commander, everything else -> sink. The
 * platform stack does no outbound ARP, so each endpoint carries its peer MAC.
 *
 * The commander's real address is not known yet; for bring-up it MIRRORS the telemetry sink
 * so a single test machine can do both ends. TODO: set the real commander MAC / IP / port. */
inline constexpr std::array<uint8_t, 6> TELEMETRY_MAC  = {0x00, 0xE0, 0x4C, 0x33, 0x0F, 0x98};
inline constexpr uint32_t               TELEMETRY_IPV4 = make_ipv4(192, 168, 0, 111);
inline constexpr uint16_t               TELEMETRY_PORT = 7520;

inline constexpr std::array<uint8_t, 6> COMMANDER_MAC  = TELEMETRY_MAC;   // TODO: real commander MAC
inline constexpr uint32_t               COMMANDER_IPV4 = TELEMETRY_IPV4;  // TODO: real commander IP
inline constexpr uint16_t               COMMANDER_PORT = TELEMETRY_PORT;  // TODO: real commander port

} // namespace detail

/** @brief An inbound CAN frame with its 29-bit id already decoded into a CanHeader,
 *         so the orchestrator routes on the header without re-parsing the id. */
struct InboundFrame {
    CanHeader                      header;
    logic::communication::CanFrame frame;
};

/**
 * @brief The FCU communication layer, parameterised on its two transports.
 * @tparam E A type modelling logic::communication::Ethernet (the UDP link to the GS).
 * @tparam C A type modelling logic::communication::Can (the FDCAN bus to the ECU).
 */
template <logic::communication::Ethernet E, logic::communication::Can C>
class Communication {
public:
    /** @brief Max payload bytes that fit in one GS datagram (after header + CRC).
     *  Telemetry uses this to size its record batches. */
    static constexpr std::size_t GS_PAYLOAD_CAPACITY =
        detail::UDP_MAX_PAYLOAD_BYTES - sizeof(EthernetHeader) - sizeof(uint32_t);

    /** @brief Construct over the two transports; does not touch hardware. */
    Communication(E& eth, C& can) : eth_(eth), can_(can) {}

    /** @brief Resolve the two ground-station endpoints (telemetry sink + commander). Call
     *         once before the first send. */
    void init()
    {
        telemetry_endpoint_.mac  = detail::TELEMETRY_MAC;
        telemetry_endpoint_.ipv4 = detail::TELEMETRY_IPV4;
        telemetry_endpoint_.port = detail::TELEMETRY_PORT;

        command_endpoint_.mac  = detail::COMMANDER_MAC;
        command_endpoint_.ipv4 = detail::COMMANDER_IPV4;
        command_endpoint_.port = detail::COMMANDER_PORT;
    }

    /** @brief Service the link so receiveDatagram() can return inbound traffic. */
    void tick() { eth_.tick(); }

    /* ---- Egress -------------------------------------------------------------- */

    /**
     * @brief Frame @p payload for the ground station and send it: an EthernetHeader
     *        (from @p sourceId to GsControl, tagged @p payloadType / @p payloadId /
     *        @p sourceState / @p seq / @p now_ms) + the payload + a CRC over the header
     *        AND the payload (the header has no checksum of its own).
     *
     * Routed by payload class: a Response goes back to the COMMANDER (the host that sends us
     * commands); all telemetry goes to the telemetry SINK. The two are distinct hosts.
     *
     * @p payloadId is opaque here — a TelemetryType, ResponseType, etc. @p seq is the
     * GS's command sequence echoed back on a relayed response (0 for telemetry). The
     * caller owns their meaning; this layer only routes and frames.
     */
    void sendToGs(BoardId sourceId, PayloadType payloadType, uint8_t payloadId,
                  uint8_t sourceState, uint8_t seq, std::span<const uint8_t> payload, uint32_t now_ms)
    {
        static std::array<uint8_t, detail::UDP_MAX_PAYLOAD_BYTES> packet;

        // A response answers a command, so it returns to the commander; telemetry streams to
        // the sink. (The FCU never sends a Command to the GS, so those are the only classes.)
        const logic::communication::Endpoint& dest =
            (payloadType == PayloadType::Response) ? command_endpoint_ : telemetry_endpoint_;

        const std::size_t chunk =
            payload.size() < GS_PAYLOAD_CAPACITY ? payload.size() : GS_PAYLOAD_CAPACITY;

        EthernetHeader header = {};
        header.sender_id           = static_cast<uint8_t>(sourceId);
        header.target_id           = static_cast<uint8_t>(BoardId::GsControl);
        header.payload_type        = static_cast<uint8_t>(payloadType);
        header.payload_id          = payloadId;
        header.payload_size_bytes  = static_cast<uint16_t>(chunk);
        header.sender_state        = sourceState;
        header.seq                 = seq;
        header.sender_timestamp_ms = now_ms;

        std::memcpy(packet.data(), &header, sizeof(header));
        if (chunk != 0) {
            std::memcpy(packet.data() + sizeof(header), payload.data(), chunk);
        }
        // CRC covers the header AND the payload (the header carries no checksum of its
        // own), i.e. everything in the datagram up to the CRC field itself.
        const uint32_t crc = logic::data_integrity::crc32(packet.data(), sizeof(header) + chunk);
        std::memcpy(packet.data() + sizeof(header) + chunk, &crc, sizeof(crc));

        (void)eth_.send(dest, std::span<const uint8_t>(packet.data(), sizeof(header) + chunk + sizeof(crc)));
    }

    /**
     * @brief Frame @p payload as a message to the ECU over CAN (from FillingStation
     *        to Engine, tagged @p payloadType / @p payloadId / @p senderState / @p seq)
     *        and send. @p seq tags a reliable command so the reply can be matched + retried
     *        (4-bit, wraps). A classic CAN frame carries at most 8 bytes; longer payloads
     *        are clipped.
     */
    void sendToEcu(PayloadType payloadType, uint8_t payloadId, uint8_t senderState, uint8_t seq,
                   std::span<const uint8_t> payload)
    {
        CanHeader header = {};
        header.frame.sender_id    = static_cast<uint8_t>(BoardId::FillingStation);
        header.frame.target_id    = static_cast<uint8_t>(BoardId::Engine);
        header.frame.sender_state = senderState;
        header.frame.payload_type = static_cast<uint8_t>(payloadType);
        header.frame.payload_id   = payloadId;
        header.frame.seq          = static_cast<uint8_t>(seq & 0x0F);
        header.frame.priority     = canBusPriority(payloadType);  // commands preempt the telemetry stream

        logic::communication::CanFrame frame;
        frame.id = header.code;
        const std::size_t n = payload.size() < frame.data.size() ? payload.size() : frame.data.size();
        if (n != 0) {
            std::memcpy(frame.data.data(), payload.data(), n);
        }
        frame.length = static_cast<uint8_t>(n);
        (void)can_.send(frame);
    }

    /* ---- Ingress ------------------------------------------------------------- */

    /** @brief Pop the next inbound UDP datagram whose trailing CRC checks out, or
     *         std::nullopt. The CRC covers the EthernetHeader + payload (appended
     *         little-endian, mirroring sendToGs); a datagram that fails it — too short,
     *         corrupted, or forged — is dropped and the next is tried, so a bad datagram
     *         never reaches the parser and does not block the queue. */
    [[nodiscard]] std::optional<logic::communication::Datagram> receiveDatagram()
    {
        while (auto datagram = eth_.receive()) {
            if (crcValid(datagram->payload)) {
                return datagram;
            }
            if (rx_crc_errors_ != 0xFFFFFFFFu) {
                ++rx_crc_errors_;   // saturating; a corrupt/forged datagram, dropped
            }
        }
        return std::nullopt;
    }

    /** @brief Count of inbound datagrams dropped for a bad CRC (saturating). */
    [[nodiscard]] uint32_t rxCrcErrors() const { return rx_crc_errors_; }

    /** @brief Pop the next inbound CAN frame with its header decoded, or std::nullopt. */
    [[nodiscard]] std::optional<InboundFrame> receiveFrame()
    {
        auto frame = can_.receive();
        if (!frame) {
            return std::nullopt;
        }
        InboundFrame in;
        in.frame      = *frame;
        in.header.code = frame->id;
        return in;
    }

    /* ---- Link health (read by telemetry) ------------------------------------- */

    [[nodiscard]] ::EthernetInfo ethInfo() const { return eth_.info(); }
    [[nodiscard]] ::CanInfo      canInfo() const { return can_.info(); }

private:
    // Verify the trailing CRC-32 the sender appended over the EthernetHeader + payload
    // (little-endian). Rejects a datagram too short to hold a header + CRC, or whose CRC
    // does not match — the same coverage sendToGs writes, so the two ends agree.
    [[nodiscard]] static bool crcValid(std::span<const uint8_t> datagram)
    {
        if (datagram.size() < sizeof(EthernetHeader) + sizeof(uint32_t)) {
            return false;
        }
        const std::size_t covered = datagram.size() - sizeof(uint32_t);
        const uint32_t    expected = logic::data_integrity::crc32(datagram.data(), covered);
        uint32_t received = 0;
        std::memcpy(&received, datagram.data() + covered, sizeof(received));
        return received == expected;
    }

    E&                             eth_;   // injected UDP link to the GS
    C&                             can_;   // injected CAN bus to the ECU
    logic::communication::Endpoint telemetry_endpoint_{};  // telemetry sink (resolved in init())
    logic::communication::Endpoint command_endpoint_{};    // commander: where responses return
    uint32_t                       rx_crc_errors_ = 0;  // inbound datagrams dropped for a bad CRC
};

} // namespace logic::fcu
