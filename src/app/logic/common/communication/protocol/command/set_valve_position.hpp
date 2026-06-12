#pragma once

#include <cstdint>

#include "system/valves/fcu.hpp"   // FcuValves — the SSOT for which valve

/* Payload for CommandType::SetValvePosition — drive a valve to a position. */

/** @brief What to do to the targeted valve. */
enum class ValveCommand : uint8_t {
    Open         = 0x00,  /**< Drive fully open. @ref SetValvePositionFrame::value is ignored. */
    Close        = 0x01,  /**< Drive fully closed. @ref SetValvePositionFrame::value is ignored. */
    SetOpenedPct = 0x02,  /**< Drive to @ref SetValvePositionFrame::value percent open. */
};
static_assert(sizeof(ValveCommand) == 1, "ValveCommand must be exactly 1 byte (on the wire)");

/** @brief SetValvePosition payload (3 bytes, like the telemetry valve record). */
struct SetValvePositionFrame {
    FcuValves    valve;   /**< Which valve to drive. */
    ValveCommand action;  /**< Open / Close / SetOpenedPct. */
    uint8_t      value;   /**< Opened %, 0..100. Used only by SetOpenedPct; optional (ignored) for Open/Close. */
};
static_assert(sizeof(SetValvePositionFrame) == 3, "SetValvePositionFrame must be 3 packed bytes");
