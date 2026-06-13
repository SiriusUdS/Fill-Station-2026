#pragma once

#include <cstdint>
#include <optional>

/* ------------------------------------------------------------------------- *
 * Response frame ids — replies to commands, the reply-path counterpart to
 * CommandType. Carried in `payload_id` when `payload_type == Response`.
 *
 * Same wire field as CommandType / TelemetryId, disambiguated by payload_type
 * (see payload_type.hpp): a Pong here and a Ping in CommandType can share the
 * raw value 0x01 because the payload_type tells them apart. Single source of
 * truth for reply ids; never spell these numbers anywhere else.
 * ------------------------------------------------------------------------- */

enum class ResponseType : uint8_t {
    Pong = 0x01,  /**< Reply to CommandType::Ping. No payload. */
};
static_assert(sizeof(ResponseType) == 1, "ResponseType must be exactly 1 byte (on the wire)");

/**
 * @brief Validate a raw on-wire id and recover the response it denotes.
 * @return The ResponseType, or std::nullopt if @p id is not a known response.
 */
[[nodiscard]] constexpr std::optional<ResponseType> toResponseType(uint8_t id)
{
    switch (static_cast<ResponseType>(id)) {
        case ResponseType::Pong:
            return static_cast<ResponseType>(id);
    }
    return std::nullopt;
}
