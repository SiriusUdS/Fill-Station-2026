#pragma once

#include <cstdint>

/** @brief Ethernet status register (8-bit), the Status half of EthernetInfo. */
struct EthernetStatus {
    uint8_t initialized : 1;  /**< true once the stack is up (init() completed). */
    uint8_t tx_busy     : 1;  /**< true if the last send() was deferred (transmit path busy). */
    uint8_t tx_error    : 1;  /**< true if the last send() failed (payload too large / internal). */
    uint8_t reserved    : 5;  /**< Padding; room for future status bits (link, phy, …). */
};
static_assert(sizeof(EthernetStatus) == 1, "EthernetStatus must be exactly 1 byte (on the wire)");
