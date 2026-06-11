#pragma once

#include <stdint.h>

/**
 * @brief Custom 29-bit CAN extended identifier used by the filling station bus.
 *
 * Overlays the 29-bit extended identifier of a classic CAN frame, packing the
 * routing and status metadata that every board exchanges. The six functional
 * fields occupy 29 bits; @ref reserved pads the structure out to a full 32-bit
 * word.
 */
typedef struct {
    uint32_t senderID:4;     /**< ID of the board that emitted the frame. */
    uint32_t targetID:4;     /**< ID of the destination board (or broadcast). */
    uint32_t deviceState:4;  /**< Sender's current device state. */
    uint32_t messageID:8;    /**< Message type / command identifier. */
    uint32_t errorCtrl:2;    /**< Error-control flags. */
    uint32_t errorCode:7;    /**< Error code reported by the sender. */
    uint32_t reserved:3;     /**< Unused; pads the 29-bit ID to 32 bits. */
}CanHeader;

// The header must overlay a single 32-bit word (a 29-bit CAN extended identifier
// plus 3 reserved bits). If the bit-fields ever exceed 32 bits, the compiler opens
// a second storage unit and sizeof jumps to 8 — this assert catches that.
static_assert(sizeof(CanHeader) == 4,
              "CanHeader must stay within one 32-bit word — the bit-fields exceed 32 bits");