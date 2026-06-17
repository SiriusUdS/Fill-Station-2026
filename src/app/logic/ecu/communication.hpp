#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "communication/interfaces/can.hpp"   // logic::communication::Can + CanFrame

#include "communication/protocol/framing/can_header.hpp"    // CanHeader
#include "communication/protocol/framing/payload_type.hpp"  // PayloadType
#include "system/board_id.hpp"                              // BoardId

/* ------------------------------------------------------------------------- *
 * ECU communication layer (HAL-free) — the single owner of the CAN bus (the ECU
 * has no Ethernet). Telemetry and Control hold a reference to one of these and
 * speak through it; nobody else touches can_ directly. The CAN-only sibling of
 * logic::fcu::Communication.
 *
 * It owns the wire MECHANICS — CAN header construction, board-id routing, the raw
 * transport — but NOT payload meaning. `payload_id` is opaque routing bytes here.
 * Telemetry fragments are framed by the shared SystemState codec (which builds whole
 * frames), so they go out via sendFrame(); single replies (a Pong, later an ack) are
 * framed here by sendToFcu(). Inbound frames are handed up raw for Control to decode.
 * ------------------------------------------------------------------------- */

namespace logic::ecu {

/**
 * @brief The ECU communication layer, parameterised on its CAN transport.
 * @tparam C A type modelling logic::communication::Can (the FDCAN bus to the FCU).
 */
template <logic::communication::Can C>
class Communication {
public:
    /** @brief Construct over the CAN bus; does not touch hardware. */
    explicit Communication(C& can) : can_(can) {}

    /* ---- Egress -------------------------------------------------------------- */

    /**
     * @brief Frame @p payload as a single message to the FCU (from Engine to
     *        FillingStation, tagged @p payloadType / @p payloadId / @p senderState / @p seq)
     *        and send. Used for replies (Pong) — @p payloadId is opaque here, and @p seq is
     *        echoed from the command being answered so the FCU can match + stop retrying.
     *        A classic CAN frame carries at most 8 bytes; longer payloads are clipped.
     */
    void sendToFcu(PayloadType payloadType, uint8_t payloadId, uint8_t senderState, uint8_t seq,
                   std::span<const uint8_t> payload)
    {
        CanHeader header = {};
        header.frame.sender_id    = static_cast<uint8_t>(BoardId::Engine);
        header.frame.target_id    = static_cast<uint8_t>(BoardId::FillingStation);
        header.frame.sender_state = senderState;
        header.frame.payload_type = static_cast<uint8_t>(payloadType);
        header.frame.payload_id   = payloadId;
        header.frame.seq          = static_cast<uint8_t>(seq & 0x0F);
        header.frame.priority     = canBusPriority(payloadType);  // responses preempt the telemetry stream

        logic::communication::CanFrame frame;
        frame.id = header.code;
        const std::size_t n = payload.size() < frame.data.size() ? payload.size() : frame.data.size();
        if (n != 0) {
            std::memcpy(frame.data.data(), payload.data(), n);
        }
        frame.length = static_cast<uint8_t>(n);
        (void)can_.send(frame);
    }

    /** @brief Send a fully-formed frame as-is. Used for telemetry fragments the shared
     *         SystemState codec has already framed (header + index + bytes). Returns the
     *         transport result (nullopt = queued) so the telemetry drain can honour
     *         backpressure: a full TX ring reports an error, and the drain holds its cursor
     *         and retries the frame next tick rather than dropping it. */
    std::optional<logic::communication::CanError>
    sendFrame(const logic::communication::CanFrame& frame) { return can_.send(frame); }

    /* ---- Ingress ------------------------------------------------------------- */

    /** @brief Pop the next inbound CAN frame, or std::nullopt. The caller decodes it. */
    [[nodiscard]] std::optional<logic::communication::CanFrame> receive() { return can_.receive(); }

    /* ---- Link health (read by telemetry) ------------------------------------- */

    [[nodiscard]] ::CanInfo canInfo() const { return can_.info(); }

private:
    C& can_;   // injected CAN bus to the FCU
};

} // namespace logic::ecu
