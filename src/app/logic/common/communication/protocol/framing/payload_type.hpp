#pragma once

#include <cstdint>
#include <optional>

/* ------------------------------------------------------------------------- *
 * Payload class — the 2-bit `payload_type` field carried in every frame header
 * (CAN and Ethernet alike). It says how to interpret the companion `payload_id`:
 *
 *   Command   -> a request; payload_id is a CommandType   (command_type.hpp)
 *   Telemetry -> unsolicited cadence data; payload_id is a TelemetryId (telemetry_id.hpp)
 *   Response  -> a reply to a command (e.g. Pong); payload_id is a ResponseType (response_type.hpp)
 *
 * One payload_type per id enum, so payload_type alone says which enum payload_id
 * indexes — a Ping (Command/0x01), a Pong (Response/0x01) and a SystemState
 * (Telemetry/0x01) can share a raw id without ambiguity.
 *
 * Single source of truth shared by both transports, mirroring how CommandType /
 * TelemetryId / ResponseType are the SSOT for the ids themselves. Two bits, so
 * 0x00 is reserved (an unset header reads as "not a known type").
 * ------------------------------------------------------------------------- */

enum class PayloadType : uint8_t {
    Command   = 0x01,  /**< payload_id is a CommandType (boards consume these). */
    Telemetry = 0x02,  /**< payload_id is a TelemetryId, sent on a cadence. */
    Response  = 0x03,  /**< payload_id is a TelemetryId, sent in reply to a command. */
};
static_assert(sizeof(PayloadType) == 1, "PayloadType must be exactly 1 byte (on the wire)");

/**
 * @brief Validate a raw on-wire value and recover the payload class.
 * @return The PayloadType, or std::nullopt if @p value is not a known class.
 */
[[nodiscard]] constexpr std::optional<PayloadType> toPayloadType(uint8_t value)
{
    switch (static_cast<PayloadType>(value)) {
        case PayloadType::Command:
        case PayloadType::Telemetry:
        case PayloadType::Response:
            return static_cast<PayloadType>(value);
    }
    return std::nullopt;
}
