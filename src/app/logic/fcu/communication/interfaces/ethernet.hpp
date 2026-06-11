#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "communication/protocol/telemetry/ethernet_info.hpp"   // EthernetInfo (the link's own info record)

/* ------------------------------------------------------------------------- *
 * Class-based UDP-over-Ethernet contract for the FCU logic layer (C++23 concept).
 *
 * Mirrors the valve/adc/storage seams: the contract is a concept, the platform
 * driver (the raw Ethernet/IPv4/UDP stack) models it, and a host fake models it
 * for tests. No HAL type appears here. The link is FCU-only — the ground-station
 * connection lives in this board's app, not the shared submodule.
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
 * @brief The contract a UDP-over-Ethernet link must satisfy.
 *
 * A conforming type exposes:
 *   - tick()        — service the link so received datagrams become available and
 *                     ARP / ICMP echo requests are answered.
 *   - receive()     — pop the oldest received datagram, or std::nullopt.
 *   - send(d, p)    — send a UDP datagram; payload is copied before framing.
 *   - info()        — the link's own EthernetInfo (state + status + drop count).
 */
template <typename T>
concept Ethernet = requires(T eth, const Endpoint& dest, std::span<const uint8_t> payload) {
    { eth.tick() }              -> std::same_as<void>;
    { eth.receive() }          -> std::same_as<std::optional<Datagram>>;
    { eth.send(dest, payload) } -> std::same_as<std::optional<NetError>>;
    { eth.info() }             -> std::same_as<::EthernetInfo>;
};

} // namespace logic::communication
