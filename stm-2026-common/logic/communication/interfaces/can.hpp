#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "communication/protocol/telemetry/can_info.hpp"   // CanInfo (the link's own info record)

/* ------------------------------------------------------------------------- *
 * Class-based CAN contract for the logic layer (C++23 concept).
 *
 * Mirrors the valve/adc/storage/ethernet seams: the contract is a concept, the
 * platform driver (the FDCAN DIL) models it, and a host fake models it for
 * tests. No vtable and no HAL type appears here, so logic stays unaware of
 * FDCAN, DMA, or any board-specific detail.
 * ------------------------------------------------------------------------- */

namespace logic::communication {

/**
 * @brief Errors the CAN interface can report.
 */
enum class CanError {
    InternalError,  /**< Unspecified failure in the underlying transport. */
};

/** @brief Maximum number of payload bytes a classic CAN frame can carry. */
inline constexpr std::size_t MAX_PAYLOAD_LENGTH_BYTES = 8;

/**
 * @brief A hardware-agnostic CAN frame as seen by the logic layer.
 */
struct CanFrame {
    uint32_t id{};                                         /**< 29-bit extended identifier. */
    std::array<uint8_t, MAX_PAYLOAD_LENGTH_BYTES> data{};  /**< Payload */
    uint8_t length{};                                      /**< Valid bytes in @ref data (0..MAX_PAYLOAD_LENGTH_BYTES). */
};

/**
 * @brief The contract a CAN interface must satisfy.
 *
 * A conforming type exposes:
 *   - send(frame)   — queue a frame for transmission; nullopt on success.
 *   - receive()     — pop the oldest received frame, or std::nullopt.
 *   - info()        — the link's own CanInfo (state + status + drop count).
 */
template <typename T>
concept Can = requires(T can, const CanFrame& frame) {
    { can.send(frame) } -> std::same_as<std::optional<CanError>>;
    { can.receive() }   -> std::same_as<std::optional<CanFrame>>;
    { can.info() }      -> std::same_as<::CanInfo>;
};

} // namespace logic::communication
