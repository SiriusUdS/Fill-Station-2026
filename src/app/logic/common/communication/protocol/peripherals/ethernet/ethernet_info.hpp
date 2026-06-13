#pragma once

#include <cstdint>

#include "peripherals/ethernet/ethernet_state.hpp"
#include "peripherals/ethernet/ethernet_status.hpp"

/* Telemetry unit for the Ethernet link: its state, status bits and a dropped-
 * datagram counter, owned and kept current by the Ethernet driver. */

/** @brief The Ethernet link's reported state + status + dropped-datagram count. */
struct EthernetInfo {
    EthernetState  state;
    EthernetStatus status;
    uint16_t       rx_dropped;  /**< inbound datagrams dropped on RX-ring overflow (saturating). */
};

// Wire layout guard: must be packed with no implicit padding.
static_assert(sizeof(EthernetInfo) == 4, "EthernetInfo must be exactly 4 bytes");
static_assert(sizeof(EthernetInfo) == 2 * sizeof(uint8_t) + sizeof(uint16_t),
              "EthernetInfo has implicit padding — keep state/status then the uint16 counter");
