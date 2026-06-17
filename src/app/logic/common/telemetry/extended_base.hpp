#pragma once

#include <cstdint>

#include "communication/protocol/telemetry/extended_system_state_base.hpp"  // ExtendedSystemStateBase
#include "control/control_flags.hpp"           // base_control_flags
#include "control/backup_status.hpp"           // backup_status (backup-domain retention health)
#include "control/last_ping.hpp"               // last_ping_ms (the GS-heartbeat liveness clock)
#include "control/refused_transition.hpp"      // last_refused_transition + count
#include "control/refused_control_flag.hpp"    // last_refused_control_flag + count
#include "control/refused_valve.hpp"           // last_refused_valve + count

/* ------------------------------------------------------------------------- *
 * Fill the shared ExtendedSystemStateBase from the process-wide control state, so
 * both boards' telemetry pipelines build the common prefix identically. The base
 * control-flags byte is common; the per-board control-flags byte is the caller's
 * (the FCU passes fcu_control_flags.raw(); the ECU passes 0 — it has none).
 * ------------------------------------------------------------------------- */

namespace logic::telemetry {

inline void fillExtendedBase(ExtendedSystemStateBase& base, uint32_t now_ms,
                             uint8_t control_flags_board)
{
    base.creation_timestamp_ms = now_ms;
    base.control_flags_base    = logic::control::base_control_flags.raw();  // common BASE flags (SD recording etc.)
    base.control_flags_board   = control_flags_board;                       // this board's PER-BOARD flags
    base.backup_status         = static_cast<uint8_t>(logic::control::backup_status);  // backup-domain retention health (boot probe)

    // Whole seconds since the last Ping this board received (the GS heartbeat), saturating at 255.
    // last_ping_ms is 0 until the first Ping, so before then this counts up from boot.
    const uint32_t since_ms = now_ms - logic::control::last_ping_ms;
    const uint32_t since_s  = since_ms / 1000u;
    base.seconds_since_last_ping = since_s > 255u ? 255u : static_cast<uint8_t>(since_s);

    RefusedCommandInfo& rc = base.refused_command_info;
    rc.set_state_from = static_cast<uint8_t>(logic::control::last_refused_transition.from);
    rc.set_state_to   = static_cast<uint8_t>(logic::control::last_refused_transition.to);
    rc.set_flag_id    = logic::control::last_refused_control_flag.flag;
    rc.set_flag_value = logic::control::last_refused_control_flag.value;
    rc.set_flag_state = static_cast<uint8_t>(logic::control::last_refused_control_flag.state);
    rc.set_state_refused_count = logic::control::refused_transition_count;
    rc.set_flag_refused_count  = logic::control::refused_control_flag_count;
    rc.set_valve_id            = logic::control::last_refused_valve.valve;
    rc.set_valve_action        = logic::control::last_refused_valve.action;
    rc.set_valve_value         = logic::control::last_refused_valve.value;
    rc.set_valve_state         = static_cast<uint8_t>(logic::control::last_refused_valve.state);
    rc.set_valve_refused_count = logic::control::refused_valve_count;
}

} // namespace logic::telemetry
