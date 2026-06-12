#pragma once

#include <cstdint>

/**
 * @brief The FCU's valves — the single source of truth for FCU valve identity.
 *
 * The enumerator value IS the valve's index in the telemetry array
 * (FcuSystemState::base.valve_info[]) and its on-wire selector in command payloads
 * (SetValvePositionFrame::valve). Add a valve here and everything indexed by it
 * follows. (The ECU has its own EcuValves enum.)
 */
enum class FcuValves : uint8_t {
    Fill = 0x00,
    Dump = 0x01,
};
static_assert(sizeof(FcuValves) == 1, "FcuValves must be exactly 1 byte (on the wire)");
