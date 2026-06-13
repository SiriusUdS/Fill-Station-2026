#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Telemetry frame ids — the cadence-data counterpart to CommandType.
 *
 * Carried in the same wire field as CommandType (CAN payload_id, Ethernet
 * payload_id), disambiguated by payload_type == Telemetry (see payload_type.hpp).
 * The ground station consumes telemetry; boards emit it on a cadence.
 * ------------------------------------------------------------------------- */

enum class TelemetryType : uint8_t {
    SystemState = 0x01,  /**< SystemState downlink (Ethernet to GS) / CAN telemetry from the ECU. */
};
static_assert(sizeof(TelemetryType) == 1, "TelemetryType must be exactly 1 byte (on the wire)");
