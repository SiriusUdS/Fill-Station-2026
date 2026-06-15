#pragma once

#include <cstdint>

#include "communication/protocol/command/set_control_flag.hpp"  // ControlFlagBase / FcuControlFlag — the on-wire SSOT

/* ------------------------------------------------------------------------- *
 * Runtime control-flag state — the live on/off value of each control flag.
 *
 * The flag enums (set_control_flag.hpp) are the single source of truth for *which*
 * flags exist; this holds their current values. Flags split into a BASE set (common
 * to every board) and a PER-BOARD set (board-specific), each its own 8-bit bitmask
 * keyed by its own enum — together the 16-bit control-flags space the GS reads from
 * telemetry. A CommandType::SetControlFlag command sets a flag; the subsystem that
 * owns it reads it (e.g. the SD recorder reads PersistingData/FastRecording). Not
 * battery-backed: all flags reset to off on every boot.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/** @brief An 8-bit bitmask of on/off flags, keyed by the flag enum @p Flag (whose values
 *         are bit positions 0..7). One instance per flag set (base, per-board). */
template <typename Flag>
class ControlFlags {
public:
    void set(Flag flag, bool on)
    {
        const uint8_t bit = static_cast<uint8_t>(1u << static_cast<uint8_t>(flag));
        if (on) { bits_ = static_cast<uint8_t>(bits_ | bit); }
        else    { bits_ = static_cast<uint8_t>(bits_ & ~bit); }
    }

    [[nodiscard]] bool get(Flag flag) const
    {
        return (bits_ & (1u << static_cast<uint8_t>(flag))) != 0u;
    }

    /** @brief The whole 8-bit mask (bit N = flag value N). Surfaced in the ExtendedSystemState
     *         telemetry (base byte or per-board byte) so the GS sees the live config. */
    [[nodiscard]] uint8_t raw() const { return bits_; }

private:
    uint8_t bits_ = 0;   // all flags off on boot (flag values are bit positions 0..7)
};

/** @brief The process-wide BASE control-flag state (common to every board). */
inline ControlFlags<ControlFlagBase> base_control_flags;

/** @brief The process-wide FCU PER-BOARD control-flag state (FCU only; unused on the ECU). */
inline ControlFlags<FcuControlFlag> fcu_control_flags;

} // namespace logic::control
