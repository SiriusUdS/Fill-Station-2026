#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Custom 29-bit CAN extended identifier used by the filling-station bus.
 *
 * Overlays the 29-bit extended id of a classic CAN frame, packing the routing
 * and status metadata every board exchanges. The six functional fields occupy
 * 29 bits; reserved pads to a full 32-bit word. messageID carries a CommandType
 * (command frames) or a TelemetryId (telemetry/response frames); senderID and
 * targetID carry BoardId values.
 * ------------------------------------------------------------------------- */

struct FrameCanHeader {
    uint32_t senderID:4;     /**< Board that emitted the frame (BoardId). */
    uint32_t targetID:4;     /**< Destination board (BoardId, or Broadcast). */
    uint32_t deviceState:4;  /**< Sender's device state / command sub-field. */
    uint32_t messageID:8;    /**< Frame type: a CommandType or a TelemetryId. */
    uint32_t errorCtrl:2;    /**< Error-control flags. */
    uint32_t errorCode:7;    /**< Error code reported by the sender. */
    uint32_t reserved:3;     /**< Unused; pads the 29-bit id to 32 bits. */
};

union CanHeader {
    FrameCanHeader frame;
    uint32_t       code;
};
static_assert(sizeof(CanHeader) == 4, "CanHeader must overlay a single 29-bit CAN id (4 bytes)");
