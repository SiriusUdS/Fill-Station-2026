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
    logic::communication::udp::Datagram datagram;
    datagram.source  = source;
    datagram.payload = std::span<const uint8_t>(udp_rx_storage.back());
    udp_rx.push_back(datagram);
}

/* ---- Interface implementations the logic links against -------------------- */

namespace logic::communication {

namespace can {

std::optional<CanError> send(const CanFrame& frame)
{
    bus().can_tx.push_back(frame);
    return std::nullopt;
}

std::optional<CanFrame> receive()
{
    auto& rx = bus().can_rx;
    if (rx.empty()) {
        return std::nullopt;
    }
    CanFrame frame = rx.front();
    rx.pop_front();
    return frame;
}

} // namespace can

namespace udp {

std::optional<NetError> send(const Endpoint& dest, std::span<const uint8_t> payload)
{
    bus().udp_tx.push_back(SentDatagram{dest, std::vector<uint8_t>(payload.begin(), payload.end())});
    return std::nullopt;
}

std::optional<Datagram> receive()
{
    auto& rx = bus().udp_rx;
    if (rx.empty()) {
        return std::nullopt;
    }
    Datagram datagram = rx.front();
    rx.pop_front();
    return datagram;
}

void tick()
{
    bus().udp_tick_count++;
}

} // namespace udp

} // namespace logic::communication
