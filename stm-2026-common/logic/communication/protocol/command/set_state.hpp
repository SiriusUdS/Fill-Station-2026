#pragma once

#include <cstdint>

/* Payload for CommandType::SetState — request a device state change. */

/** @name SetState control flags (bitmask in SetStateFrame::flags) */
///@{
inline constexpr uint8_t SET_STATE_FLAG_GET_SYSTEM_AFTER = 1u << 0;
inline constexpr uint8_t SET_STATE_FLAG_RESET            = 1u << 1;
inline constexpr uint8_t SET_STATE_FLAG_START_LOGGING    = 1u << 2;
inline constexpr uint8_t SET_STATE_FLAG_SPREAD           = 1u << 3;
///@}

/** @brief SetState payload: the requested state plus control flags. */
struct SetStateFrame {
    uint8_t flags;        /**< Bitmask of SET_STATE_FLAG_*. */
    uint8_t requestedID;  /**< Requested state id (TODO: becomes a typed State). */
};
