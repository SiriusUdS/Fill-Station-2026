#pragma once

#include <optional>
#include <span>

#include "communication/interfaces/ethernet.hpp"   // logic::communication::Ethernet contract

/* ------------------------------------------------------------------------- *
 * FCU Ethernet platform driver.
 *
 * One instance owns the logic-side seam to the HAL ETH peripheral: the raw
 * Ethernet/IPv4/UDP/ICMP/ARP stack and the DMA RX buffer pool. The board has
 * exactly one link, so the heavy stack state (DMA pools, TX templates) is
 * file-static in the .cpp and this class is a thin handle over it that models
 * logic::communication::Ethernet. init() is the HAL-coupled bring-up; tick() /
 * receive() / send() / info() are the contract the logic layer consumes.
 * ------------------------------------------------------------------------- */

namespace platform::communication::ethernet {

class Ethernet {
public:
    Ethernet() = default;

    /**
     * @brief  Start the ETH peripheral, initialise the TX frame templates and the
     *         DMA RX pool, and mark the link up. Call once at startup, after
     *         MX_ETH_Init().
     */
    void init();

    /* ---- logic::communication::Ethernet contract ------------------------- */

    /** @brief Service the link: process inbound frames, answer ARP / ICMP echo. */
    void tick();

    /** @brief Pop the oldest received UDP datagram, or std::nullopt if none. */
    [[nodiscard]] std::optional<logic::communication::Datagram> receive();

    /** @brief Send a UDP datagram (payload copied into the DMA TX region first). */
    [[nodiscard]] std::optional<logic::communication::NetError> send(
        const logic::communication::Endpoint& dest, std::span<const uint8_t> payload);

    /** @brief The link's own info record: state + status + dropped-datagram count. */
    [[nodiscard]] EthernetInfo info() const;
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::communication::Ethernet<Ethernet>);

} // namespace platform::communication::ethernet
