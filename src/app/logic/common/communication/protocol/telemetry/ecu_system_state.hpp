#pragma once

#include "telemetry/system_state_base.hpp"

/* Periodic ECU telemetry record — the engine board's live state, downlinked over
 * CAN to the FCU (which relays it to the ground station).
 *
 * The ECU has no board-specific telemetry beyond the common base YET, so this is
 * `base` and nothing more. It is still a distinct type (not an alias) so the
 * engine's record can grow its own fields without disturbing the FCU's. The FCU's
 * Ethernet info lives only in FcuSystemState — the ECU has no Ethernet. */

struct EcuSystemState {
    SystemStateBase base;
    // ECU-specific telemetry fields are added here as the two records diverge.
};

static_assert(sizeof(EcuSystemState) == sizeof(SystemStateBase),
              "EcuSystemState has implicit padding — add explicit reserved bytes");
