#pragma once

#include <cstdint>

/* Telemetry representation of the Ethernet link's state. */

/** @brief The Ethernet link's reported state. */
enum class EthernetState : uint8_t {
    Unknown = 0x00,  /**< not yet initialized */
    Up      = 0x01,  /**< initialized, stack running */
    Down    = 0x02,  /**< link down / no carrier */
};

// Wire layout guard: must be packed with no implicit padding.
static_assert(sizeof(EthernetState) == 1, "EthernetState must be exactly 1 byte");
