#pragma once

#include "telemetry/system_state_base.hpp"
#include "peripherals/ethernet/ethernet_info.hpp"

/* Periodic FCU telemetry record — the filling-station board's live state,
 * downlinked over Ethernet to the ground station. Adds the Ethernet link's info
 * to the common base; the ECU record (EcuSystemState) has no eth_info because the
 * engine board has no Ethernet peripheral. */

struct FcuSystemState {
    SystemStateBase base;
    EthernetInfo    eth_info;  /**< Ethernet link: state + status + dropped-datagram count. */
};

// Wire layout guard: base + eth_info packed with no implicit padding between them.
static_assert(sizeof(FcuSystemState) == sizeof(SystemStateBase) + sizeof(EthernetInfo),
              "FcuSystemState has implicit padding — add explicit reserved bytes");
