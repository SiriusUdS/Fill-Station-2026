#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * System-wide board identities — the single source of truth for who is who on
 * every transport (CAN/Ethernet sender_id/target_id). The enumerator
 * value IS the on-wire id; never spell these numbers anywhere else.
 * ------------------------------------------------------------------------- */

enum class BoardId : uint8_t {
    Engine         = 0x01,  /**< ECU. */
    FillingStation = 0x02,  /**< FCU. */
    GsControl      = 0x03,  /**< Ground station. */
    Broadcast      = 0x04,  /**< Addressed to every board. */
};
static_assert(sizeof(BoardId) == 1, "BoardId must be exactly 1 byte (on the wire)");
