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

/* Ground-station endpoint (telemetry / response destination). */
inline constexpr std::array<uint8_t, 6> GS_MAC  = {0x00, 0xE0, 0x4C, 0x33, 0x0F, 0x98};
inline constexpr uint16_t               GS_PORT = 7520;

/* The link's UDP payload limit (EthernetHeader + payload + CRC must fit; keep in
   sync with the platform stack). */
inline constexpr std::size_t UDP_MAX_PAYLOAD_BYTES = 1432;

inline constexpr uint32_t make_ipv4(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return (static_cast<uint32_t>(b1) << 24) | (static_cast<uint32_t>(b2) << 16) |
           (static_cast<uint32_t>(b3) << 8) | static_cast<uint32_t>(b4);
}

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

    /** @brief Resolve the ground-station endpoint. Call once before the first send. */
    void init()
    {
        gs_.mac  = detail::GS_MAC;
        gs_.ipv4 = detail::make_ipv4(192, 168, 0, 111);
        gs_.port = detail::GS_PORT;
    }

    /** @brief Service the link so receiveDatagram() can return inbound traffic. */
    void tick() { eth_.tick(); }

    /* ---- Egress -------------------------------------------------------------- */

    /**
     * @brief Frame @p payload for the ground station and send it: an EthernetHeader
     *        (from @p sourceId to GsControl, tagged @p payloadType / @p payloadId /
     *        @p sourceState / @p seq / @p now_ms) + the payload + a CRC over the payload.
     *
     * @p payloadId is opaque here — a TelemetryType, ResponseType, etc. @p seq is the
     * GS's command sequence echoed back on a relayed response (0 for telemetry). The
     * caller owns their meaning; this layer only routes and frames.
     */
    void sendToGs(BoardId sourceId, PayloadType payloadType, uint8_t payloadId,
                  uint8_t sourceState, uint8_t seq, std::span<const uint8_t> payload, uint32_t now_ms)
    {
        static std::array<uint8_t, detail::UDP_MAX_PAYLOAD_BYTES> packet;

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
        const uint32_t crc = logic::data_integrity::crc32(packet.data() + sizeof(header), chunk);
        std::memcpy(packet.data() + sizeof(header) + chunk, &crc, sizeof(crc));

        (void)eth_.send(gs_, std::span<const uint8_t>(packet.data(), sizeof(header) + chunk + sizeof(crc)));
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

    /** @brief Pop the next inbound UDP datagram, or std::nullopt. The caller parses it. */
    [[nodiscard]] std::optional<logic::communication::Datagram> receiveDatagram()
    {
        return eth_.receive();
    }

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
    E&                             eth_;   // injected UDP link to the GS
    C&                             can_;   // injected CAN bus to the ECU
    logic::communication::Endpoint gs_{};  // ground-station endpoint (resolved in init())
};

} // namespace logic::fcu
