#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

/* ------------------------------------------------------------------------- *
 * Statically-linked UDP-over-Ethernet interface for the FCU logic layer.
 *
 * FCU-only: the ground-station link lives entirely in this board's app, not in
 * the shared submodule. Logic depends ONLY on the declarations below; the
 * platform provides the definitions (raw Ethernet/IPv4/UDP stack, DMA buffers,
 * HAL ETH driver) at link time. No HAL type appears here.
 *
 * send() copies the payload into the DMA-accessible transmit region before
 * framing it, so callers may pass any buffer.
 * ------------------------------------------------------------------------- */

namespace logic::communication {

/** @brief Number of bytes in a MAC address. */
inline constexpr std::size_t MAC_LENGTH_BYTES = 6;

/**
 * @brief A network endpoint addressing one peer.
 *
 * Carries the MAC as well as the IPv4 address because the platform stack does
 * no outbound ARP resolution — the caller supplies the destination MAC.
 */
struct Endpoint {
    std::array<uint8_t, MAC_LENGTH_BYTES> mac{};  /**< Peer MAC address. */
    uint32_t ipv4 = 0;                            /**< Peer IPv4 address (host order). */
    uint16_t port = 0;                            /**< Peer UDP port. */
};

/** @brief Errors the UDP interface can report. */
enum class NetError {
    InternalError,  /**< Unspecified failure in the underlying stack. */
    Busy,           /**< Transmit path busy; retry later. */
};

namespace udp {

/**
 * @brief  A received UDP datagram.
 *
 * @c payload views platform-owned storage and is valid only until the next
 * call to receive(); copy it out if it must outlive that.
 */
struct Datagram {
    Endpoint source;                   /**< Sender endpoint. */
    std::span<const uint8_t> payload;  /**< Datagram payload (platform-owned). */
};

/**
 * @brief  Send a UDP datagram.
 * @param  dest     Destination endpoint (MAC + IPv4 + port).
 * @param  payload  Bytes to send; copied into the DMA-accessible transmit
 *                  region before framing.
 * @return std::nullopt on success, or a NetError describing the failure.
 */
[[nodiscard]] std::optional<NetError> send(const Endpoint& dest,
                                           std::span<const uint8_t> payload);

/**
 * @brief  Pop the oldest received UDP datagram.
 * @return The datagram, or std::nullopt if none are available.
 */
[[nodiscard]] std::optional<Datagram> receive();

} // namespace udp

} // namespace logic::communication
