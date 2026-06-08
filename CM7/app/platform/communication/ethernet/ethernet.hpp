#pragma once

/* ------------------------------------------------------------------------- *
 * FCU Ethernet platform driver.
 *
 * Owns the HAL ETH peripheral, the raw Ethernet/IPv4/UDP/ICMP/ARP stack and the
 * DMA RX buffer pool, and DEFINES the logic-side UDP seam declared in
 * communication/interfaces/ethernet.hpp (logic::communication::udp::send/
 * receive).
 *
 * Only the HAL-coupled entry point (init) is exposed here; servicing the link
 * and UDP messaging are reached through the logic interface (udp::tick / send /
 * receive).
 * ------------------------------------------------------------------------- */

namespace platform::communication::ethernet {

/**
 * @brief  Start the ETH peripheral, initialise the TX frame templates and the
 *         DMA RX pool. Call once at startup, after MX_ETH_Init().
 */
void init();

} // namespace platform::communication::ethernet
