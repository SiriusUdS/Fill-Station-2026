#pragma once

/* ------------------------------------------------------------------------- *
 * Host test doubles for the logic communication interfaces.
 *
 * The FCU logic (fcu_controller.cpp) depends only on the HAL-free interface
 * declarations in communication/interfaces/{can,ethernet}.hpp; the firmware
 * provides the definitions at link time. For host unit tests we link these
 * fakes instead: tests push inbound CAN frames / UDP datagrams via the FakeBus,
 * run logic::fcu::tick(), then inspect what the logic sent back.
 *
 * The single FakeBus instance is reached through bus(); reset it in each test's
 * SetUp().
 * ------------------------------------------------------------------------- */

#include "communication/interfaces/can.hpp"
#include "communication/interfaces/ethernet.hpp"

#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <vector>

/** @brief A UDP datagram captured from FakeEthernet::send(), with its payload copied. */
struct SentDatagram {
    logic::communication::Endpoint dest;
    std::vector<uint8_t>           payload;
};

/**
 * @brief In-memory stand-in for the CAN + UDP transports.
 *
 * Inbound queues (*_rx) feed the logic's receive() calls; outbound vectors
 * (*_tx) record what the logic transmitted so tests can assert on it.
 */
struct FakeBus {
    /* ---- CAN ---- */
    std::deque<logic::communication::CanFrame>  can_rx;  /**< Frames receive() will hand to the logic. */
    std::vector<logic::communication::CanFrame> can_tx;  /**< Frames the logic sent. */

    /* ---- UDP ---- */
    std::vector<SentDatagram> udp_tx;          /**< Datagrams the logic sent. */
    int                       udp_tick_count;  /**< Number of FakeEthernet::tick() calls. */

    /* Inbound datagram storage. Datagram::payload is a span into platform-owned
       bytes; udp_rx_storage owns those bytes (std::deque keeps stable addresses
       so the spans in udp_rx stay valid). */
    std::deque<std::vector<uint8_t>>             udp_rx_storage;
    std::deque<logic::communication::Datagram>   udp_rx;

    /** @brief Clear every queue and counter back to the initial state. */
    void reset();

    /** @brief Queue a CAN frame for the logic to receive(). */
    void push_can(const logic::communication::CanFrame& frame);

    /** @brief Queue a UDP datagram (bytes are copied) for the logic to receive(). */
    void push_udp(const logic::communication::Endpoint& source,
                  std::span<const uint8_t>              bytes);
};

/** @brief The process-wide FakeBus the interface fakes delegate to. */
FakeBus& bus();

/**
 * @brief Held stand-in for the UDP-over-Ethernet link (models
 *        logic::communication::Ethernet).
 *
 * Forwards to the process-wide FakeBus so existing assertions (bus().udp_tx,
 * bus().push_udp, bus().udp_tick_count) keep working, and exposes a scriptable
 * info() for the link-health telemetry. The controller fixtures hold one and
 * inject it as the Ethernet template parameter.
 */
struct FakeEthernet {
    EthernetInfo info_value{};   /**< Returned verbatim by info(); script .state / .status. */

    void tick() { bus().udp_tick_count++; }

    [[nodiscard]] std::optional<logic::communication::Datagram> receive()
    {
        auto& rx = bus().udp_rx;
        if (rx.empty()) {
            return std::nullopt;
        }
        logic::communication::Datagram datagram = rx.front();
        rx.pop_front();
        return datagram;
    }

    [[nodiscard]] std::optional<logic::communication::NetError> send(
        const logic::communication::Endpoint& dest, std::span<const uint8_t> payload)
    {
        bus().udp_tx.push_back(SentDatagram{dest, std::vector<uint8_t>(payload.begin(), payload.end())});
        return std::nullopt;
    }

    [[nodiscard]] EthernetInfo info() const { return info_value; }
};

static_assert(logic::communication::Ethernet<FakeEthernet>);

/**
 * @brief Held stand-in for the CAN bus (models logic::communication::Can).
 *
 * Forwards to the process-wide FakeBus so existing assertions (bus().can_tx,
 * bus().push_can) keep working, and exposes a scriptable info() for the
 * link-health telemetry. The controller fixtures hold one and inject it as the
 * Can template parameter.
 */
struct FakeCan {
    CanInfo info_value{};   /**< Returned verbatim by info(); script .state / .status. */

    [[nodiscard]] std::optional<logic::communication::CanError> send(
        const logic::communication::CanFrame& frame)
    {
        bus().can_tx.push_back(frame);
        return std::nullopt;
    }

    [[nodiscard]] std::optional<logic::communication::CanFrame> receive()
    {
        auto& rx = bus().can_rx;
        if (rx.empty()) {
            return std::nullopt;
        }
        logic::communication::CanFrame frame = rx.front();
        rx.pop_front();
        return frame;
    }

    [[nodiscard]] CanInfo info() const { return info_value; }
};

static_assert(logic::communication::Can<FakeCan>);
