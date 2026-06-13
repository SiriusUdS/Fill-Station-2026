#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Custom 29-bit CAN extended identifier used by the filling-station bus.
 *
 * Overlays the 29-bit extended id of a classic CAN frame, packing the routing
 * and status metadata every board exchanges. The functional fields occupy 29
 * bits; reserved pads to a full 32-bit word. The routing/payload fields use the
 * same names and meanings as EthernetHeader so the two transports stay uniform:
 * sender_id/target_id carry BoardId values, payload_type is a PayloadType, and
 * payload_id carries a CommandType (commands) or a TelemetryType (telemetry/
 * responses). seq tags a command so its reply can be matched + retried (mirrors
 * EthernetHeader::seq); error_code is a CAN-only sender diagnostic.
 * ------------------------------------------------------------------------- */

struct FrameCanHeader {
    uint32_t sender_id:4;     /**< BoardId that emitted the frame. */
    uint32_t target_id:4;     /**< Destination BoardId (or Broadcast). */
    uint32_t sender_state:4;  /**< Sender's device state / command sub-field. */
    uint32_t payload_type:2;  /**< Command vs telemetry vs response class (PayloadType). */
    uint32_t payload_id:6;    /**< A CommandType or a TelemetryType. */
    uint32_t seq:4;           /**< Command/response sequence: tags a command so its reply
                                   (e.g. Pong) is matched to it and retried; the receiver echoes
                                   it. 0 for telemetry. Mirrors EthernetHeader::seq. */
    uint32_t error_code:5;    /**< Error code reported by the sender (CAN-only diagnostic). */
    uint32_t reserved:3;      /**< Unused; pads the 29-bit id to 32 bits. */
};

union CanHeader {
    FrameCanHeader frame;
    uint32_t       code;
};
static_assert(sizeof(CanHeader) == 4, "CanHeader must overlay a single 29-bit CAN id (4 bytes)");
