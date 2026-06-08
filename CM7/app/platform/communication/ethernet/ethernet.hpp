#pragma once

/* ------------------------------------------------------------------------- *
 * FCU Ethernet platform driver.
 *
 * Owns the HAL ETH peripheral, the raw Ethernet/IPv4/UDP/ICMP/ARP stack and the
 * DMA RX buffer pool, and DEFINES the logic-side UDP seam declared in
 * communication/interfaces/ethernet.hpp (logic::communication::udp::send/
 * receive).
 *
 * Only the HAL-coupled entry points (init, process) are exposed here; UDP
 * messaging is reached through the logic interface.
 * ------------------------------------------------------------------------- */

namespace platform::communication::ethernet {

/**
 * @brief  Start the ETH peripheral, initialise the TX frame templates and the
 *         DMA RX pool. Call once at startup, after MX_ETH_Init().
 */
void init();

/**
 * @brief  Drain received frames from the DMA pool and dispatch them: UDP
 *         datagrams are queued for logic::communication::udp::receive(), ICMP
 *         echo requests and ARP requests are answered internally. Call
 *         periodically from main().
 */
void process();

} // namespace platform::communication::ethernet
