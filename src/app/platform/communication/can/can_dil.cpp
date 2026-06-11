/**
  ******************************************************************************
  * @file    communication/can/can_dil.cpp
  * @brief   STM32H7 FDCAN driver and platform-side definition of the logic CAN
  *          interface. RX filtering, an interrupt-driven ring buffer and the TX
  *          path, built directly on the STM32H7 HAL.
  ******************************************************************************
  */

#include "communication/can/can_dil.hpp"
#include "communication/interfaces/can.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

using logic::communication::CanError;
using logic::communication::CanFrame;
using logic::communication::MAX_PAYLOAD_LENGTH_BYTES;

namespace {

/* CAN extended-identifier layout: the target id occupies a 4-bit field that
   starts at bit 4 (senderID is the low nibble). The RX filter matches on it. */
constexpr uint32_t TARGET_ID_SHIFT_BITS = 4;
constexpr uint32_t TARGET_ID_WIDTH_BITS = 4;
constexpr uint32_t TARGET_ID_MASK =
    ((1U << TARGET_ID_WIDTH_BITS) - 1U) << TARGET_ID_SHIFT_BITS;

/* Capacity of the software RX ring. Single-producer (ISR) / single-consumer
   (main loop), so a plain head/tail index pair is enough. */
constexpr std::size_t RX_RING_CAPACITY_FRAMES = 32;

struct RxFrame {
    FDCAN_RxHeaderTypeDef header{};
    std::array<uint8_t, MAX_PAYLOAD_LENGTH_BYTES> data{};
};

struct RxRing {
    std::array<RxFrame, RX_RING_CAPACITY_FRAMES> frames{};
    volatile uint16_t head = 0;
    volatile uint16_t tail = 0;
};

RxRing s_rxRing;

/* Handle captured at init so send() can reach the peripheral. */
FDCAN_HandleTypeDef* s_hfdcan = nullptr;

/* The interface's telemetry record. The board has one CAN node, so this is
   file-static like the ring; the Can handle exposes it through info(). The RX
   ISR bumps rx_dropped; init() / send() update state and status from the main
   loop. */
CanInfo s_info{};

} // namespace

/* -------------------------------------------------------------------------- */
/* Platform DIL entry point                                                   */
/* -------------------------------------------------------------------------- */

namespace platform::communication::can {

std::optional<CanError> Can::init(FDCAN_HandleTypeDef* hfdcan, uint8_t nodeId)
{
    s_hfdcan = hfdcan;

    FDCAN_FilterTypeDef filter{};
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = static_cast<uint32_t>(nodeId) << TARGET_ID_SHIFT_BITS;
    filter.FilterID2    = TARGET_ID_MASK;

    const auto fail = [] {
        s_info.state = CanState::Error;
        return std::optional<CanError>{CanError::InternalError};
    };

    if (HAL_FDCAN_ConfigFilter(s_hfdcan, &filter) != HAL_OK) {
        return fail();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(s_hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        return fail();
    }
    if (HAL_FDCAN_Start(s_hfdcan) != HAL_OK) {
        return fail();
    }
    if (HAL_FDCAN_ActivateNotification(s_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return fail();
    }

    s_info.status.initialized = 1u;
    s_info.state              = CanState::Active;
    return std::nullopt;
}

std::optional<CanError> Can::send(const CanFrame& frame)
{
    FDCAN_TxHeaderTypeDef txHeader{};
    txHeader.Identifier          = frame.id;
    txHeader.IdType              = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_8;  // classic 8-byte frame
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) == 0) {
        s_info.status.tx_error = 1u;
        return CanError::InternalError;  // TX FIFO full
    }

    // The HAL signature takes a non-const payload pointer although it only reads it.
    if (HAL_FDCAN_AddMessageToTxFifoQ(
            s_hfdcan, &txHeader,
            const_cast<uint8_t*>(frame.data.data())) != HAL_OK) {
        s_info.status.tx_error = 1u;
        return CanError::InternalError;
    }

    s_info.status.tx_error = 0u;
    return std::nullopt;
}

std::optional<CanFrame> Can::receive()
{
    if (s_rxRing.tail == s_rxRing.head) {
        return std::nullopt;  // ring empty
    }

    const RxFrame& msg = s_rxRing.frames[s_rxRing.tail];

    CanFrame frame;
    frame.id     = msg.header.Identifier;
    frame.length = MAX_PAYLOAD_LENGTH_BYTES;  // peripheral delivers a full 8-byte frame
    frame.data   = msg.data;

    s_rxRing.tail = (s_rxRing.tail + 1) % RX_RING_CAPACITY_FRAMES;
    return frame;
}

CanInfo Can::info() const
{
    return s_info;
}

} // namespace platform::communication::can

/* -------------------------------------------------------------------------- */
/* Interrupt service routine                                                  */
/* -------------------------------------------------------------------------- */

/* FDCAN RX FIFO0 "new message" interrupt: producer side of the ring buffer.
   Keeps C linkage so it overrides the HAL's weak symbol. */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rxFifo0ITs)
{
    if ((rxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
        return;
    }

    const uint16_t nextHead = (s_rxRing.head + 1) % RX_RING_CAPACITY_FRAMES;

    if (nextHead != s_rxRing.tail) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &s_rxRing.frames[s_rxRing.head].header,
                                   s_rxRing.frames[s_rxRing.head].data.data()) == HAL_OK) {
            s_rxRing.head = nextHead;
        }
    } else {
        // Ring full: drain the hardware FIFO so the peripheral does not lock up,
        // and count the dropped frame (saturating) for telemetry.
        FDCAN_RxHeaderTypeDef dummyHeader{};
        std::array<uint8_t, MAX_PAYLOAD_LENGTH_BYTES> dummyData{};
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &dummyHeader, dummyData.data());
        if (s_info.rx_dropped != 0xFFFFu) {
            ++s_info.rx_dropped;
        }
    }
}
