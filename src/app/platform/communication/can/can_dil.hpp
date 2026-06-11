#pragma once

#include <cstdint>
#include <optional>

#include "stm32h7xx_hal.h"                    // FDCAN_HandleTypeDef + FDCAN HAL API
#include "communication/interfaces/can.hpp"   // logic::communication::Can contract

/* ------------------------------------------------------------------------- *
 * Platform CAN DIL (Driver Interface Layer).
 *
 * One instance owns the logic-side seam to the STM32H7 FDCAN peripheral: RX
 * filtering, the interrupt-driven RX ring buffer and the TX path. The board has
 * exactly one CAN node, so the ring/handle state is file-static in the .cpp and
 * this class is a thin handle over it that models logic::communication::Can.
 * init() is the HAL-coupled bring-up (it needs the FDCAN handle); send() /
 * receive() / info() are the contract the logic layer consumes.
 * ------------------------------------------------------------------------- */

namespace platform::communication::can {

class Can {
public:
    Can() = default;

    /**
     * @brief  Bind the FDCAN peripheral, install the RX acceptance filter for this
     *         node, start the peripheral and enable the RX FIFO0 interrupt.
     * @param  hfdcan  FDCAN peripheral handle (e.g. &hfdcan1), already created by
     *                 the CubeMX init.
     * @param  nodeId  This node's identifier, matched against the target-id field
     *                 of incoming extended identifiers.
     * @return std::nullopt on success, or a CanError describing the failure.
     */
    [[nodiscard]] std::optional<logic::communication::CanError>
    init(FDCAN_HandleTypeDef* hfdcan, uint8_t nodeId);

    /* ---- logic::communication::Can contract ------------------------------ */

    /** @brief Queue a frame for transmission; nullopt on success. */
    [[nodiscard]] std::optional<logic::communication::CanError>
    send(const logic::communication::CanFrame& frame);

    /** @brief Pop the oldest received frame, or std::nullopt if none. */
    [[nodiscard]] std::optional<logic::communication::CanFrame> receive();

    /** @brief The interface's own info record: state + status + dropped-frame count. */
    [[nodiscard]] CanInfo info() const;
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::communication::Can<Can>);

} // namespace platform::communication::can
