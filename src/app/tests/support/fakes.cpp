/* ------------------------------------------------------------------------- *
 * Definitions of the logic communication interfaces, backed by FakeBus.
 *
 * These satisfy the same symbols the firmware platform provides, so linking
 * this file in place of the platform lets the FCU logic run on the host.
 * ------------------------------------------------------------------------- */

#include "fakes.hpp"

FakeBus& bus()
{
    static FakeBus instance;
    return instance;
}

void FakeBus::reset()
{
    can_rx.clear();
    can_tx.clear();
    udp_tx.clear();
    udp_tick_count = 0;
    udp_rx_storage.clear();
    udp_rx.clear();
}

void FakeBus::push_can(const logic::communication::CanFrame& frame)
{
    can_rx.push_back(frame);
}

void FakeBus::push_udp(const logic::communication::Endpoint& source,
                       std::span<const uint8_t>              bytes)
{
    udp_rx_storage.emplace_back(bytes.begin(), bytes.end());
    logic::communication::Datagram datagram;
    datagram.source  = source;
    datagram.payload = std::span<const uint8_t>(udp_rx_storage.back());
    udp_rx.push_back(datagram);
}

/* The CAN and UDP-over-Ethernet seams are no longer free-function interfaces:
   they are the held FakeCan / FakeEthernet classes (see fakes.hpp), which
   forward to this same FakeBus. */
