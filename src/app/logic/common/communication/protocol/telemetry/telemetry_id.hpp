#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Telemetry / response frame ids — the non-command counterpart to CommandType.
 *
 * Carried in the same wire field as CommandType (CAN messageID, Ethernet
 * payloadID). A frame is a command if its id is a CommandType, telemetry if it
 * is a TelemetryId; the receiver interprets the field by the direction it
 * expects (boards consume commands, the ground station consumes telemetry).
 * ------------------------------------------------------------------------- */

enum class TelemetryId : uint8_t {
    SystemState = 0x01,  /**< SystemState downlink (Ethernet to GS) / CAN telemetry from the ECU. */
};
static_assert(sizeof(TelemetryId) == 1, "TelemetryId must be exactly 1 byte (on the wire)");
