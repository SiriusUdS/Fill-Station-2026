#pragma once

#include <cstdint>

#include "communication/protocol/command/set_control_flag.hpp"  // ControlFlag — the on-wire SSOT

/* ------------------------------------------------------------------------- *
 * Runtime control-flag state — the live on/off value of each control flag.
 *
 * ControlFlag (set_control_flag.hpp) is the single source of truth for *which*
 * flags exist; this is just their current values, stored as a bitmask indexed by
 * the enum, so adding a flag to ControlFlag needs no change here. A
 * CommandType::SetControlFlag command sets a flag; the subsystem that owns it
 * reads it (e.g. the telemetry pipeline reads PersistingData to decide whether to
 * write a drained record to the SD card). Decouples the command handler (sets values) from the subsystems
 * (read values). Not battery-backed: all flags reset to off on every boot.
 * ------------------------------------------------------------------------- */

namespace logic::control {

class ControlFlags {
public:
    void set(ControlFlag flag, bool on)
    {
        const uint32_t bit = 1u << static_cast<uint8_t>(flag);
        if (on) { bits_ |= bit; } else { bits_ &= ~bit; }
    }

    [[nodiscard]] bool get(ControlFlag flag) const
    {
        return (bits_ & (1u << static_cast<uint8_t>(flag))) != 0u;
    }

    /** @brief The whole bitmask (bit N = ControlFlag value N). Surfaced in the
     *         ExtendedSystemState telemetry so the GS sees the live config. */
    [[nodiscard]] uint32_t raw() const { return bits_; }

private:
    uint32_t bits_ = 0;   // all flags off on boot (ControlFlag ids are bit positions 0..31)
};

/** @brief The process-wide control-flag state (one instance, like persistent_state). */
inline ControlFlags control_flags;

} // namespace logic::control
