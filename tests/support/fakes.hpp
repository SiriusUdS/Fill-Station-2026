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
#include <span>
#include <vector>

/** @brief A UDP datagram captured from udp::send(), with its payload copied. */
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
    int                       udp_tick_count;  /**< Number of udp::tick() calls. */

    /* Inbound datagram storage. Datagram::payload is a span into platform-owned
       bytes; udp_rx_storage owns those bytes (std::deque keeps stable addresses
       so the spans in udp_rx stay valid). */
    std::deque<std::vector<uint8_t>>             udp_rx_storage;
    std::deque<logic::communication::udp::Datagram> udp_rx;

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
