#pragma once

/* ------------------------------------------------------------------------- *
 * Test helper: append the GS-frame CRC-32 to a built datagram.
 *
 * The FCU verifies a trailing CRC over the EthernetHeader + payload on every inbound
 * datagram (Communication::receiveDatagram), so a test that delivers a command through
 * the controller's tick() must append it — exactly as the ground station does. The CRC
 * is the reflected/zlib variant the firmware uses (here the host software reference,
 * crc32_host.cpp), appended little-endian.
 * ------------------------------------------------------------------------- */

#include "data_integrity/crc32.hpp"   // logic::data_integrity::crc32

#include <cstdint>
#include <cstring>
#include <vector>

/** @brief Append the CRC-32 over the whole current datagram (header + payload), LE. */
inline void appendGsCrc(std::vector<uint8_t>& datagram)
{
    const uint32_t crc = logic::data_integrity::crc32(datagram.data(), datagram.size());
    const std::size_t off = datagram.size();
    datagram.resize(off + sizeof(crc));
    std::memcpy(datagram.data() + off, &crc, sizeof(crc));
}
