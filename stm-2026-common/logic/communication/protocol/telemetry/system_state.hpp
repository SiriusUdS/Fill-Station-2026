#pragma once

#include <cstdint>

#include "sirius-headers-common/Telecommunication/InterfaceField.h"
#include "communication/protocol/telemetry/adc_info.hpp"
#include "communication/protocol/telemetry/valve_info.hpp"
#include "communication/protocol/telemetry/storage_info.hpp"
#include "communication/protocol/telemetry/ethernet_info.hpp"
#include "communication/protocol/telemetry/can_info.hpp"
#include "communication/protocol/ethernet/ethernet_header.hpp"

/* Periodic downlink telemetry: the board's full live state sent to the ground
 * station — timestamps, interface flags, the streaming ADC, the valves, the SD
 * card, the Ethernet link and the CAN bus. */

struct SystemState {
    uint32_t       frameTs_MS;          /**< Time this frame was assembled. */
    uint32_t       lastHandshakeTs_MS;  /**< Time of the last GS handshake. */
    InterfaceField interfaces;
    AdcInfo        adc_info;            /**< Streaming ADC: state + status + channels. */
    ValveInfo      valve_info[2];       /**< 2 x 3 bytes. */
    StorageInfo    storage_info;        /**< SD card: state + status (fills the former 2-byte pad). */
    EthernetInfo   eth_info;            /**< Ethernet link: state + status + dropped-datagram count. */
    CanInfo        can_info;            /**< CAN bus: state + status + dropped-frame count. */
};

// Wire layout guard: the downlink telemetry must be packed with no implicit
// padding so the ground station decodes it byte-for-byte. If a field change
// introduces a gap, this fails — add an explicit reserved field to close it.
static_assert(sizeof(SystemState) == 2 * sizeof(uint32_t)    // frameTs_MS + lastHandshakeTs_MS
                                   + sizeof(InterfaceField)   // interfaces
                                   + sizeof(AdcInfo)          // adc_info
                                   + 2 * sizeof(ValveInfo)    // valve_info
                                   + sizeof(StorageInfo)      // storage_info
                                   + sizeof(EthernetInfo)     // eth_info
                                   + sizeof(CanInfo),         // can_info
              "SystemState has implicit padding — add explicit reserved bytes");

/* The GET_SYSTEM downlink packet on the wire: the 12-byte EthernetHeader, the
 * SystemState payload, then a trailing CRC32 over the payload. Both the board
 * (sender) and the ground station (which reinterprets the received datagram)
 * use this struct, so it must be padding-free. */
struct SystemStatePacket {
    EthernetHeader header;
    SystemState    state;
    uint32_t       crc;
};

static_assert(sizeof(SystemStatePacket) ==
                  sizeof(EthernetHeader) + sizeof(SystemState) + sizeof(uint32_t),
              "SystemStatePacket has implicit padding — the wire packet must be packed");
