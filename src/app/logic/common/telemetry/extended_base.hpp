#pragma once

#include <cstdint>

#include "communication/protocol/telemetry/extended_system_state_base.hpp"  // ExtendedSystemStateBase
#include "control/control_flags.hpp"           // base_control_flags
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
