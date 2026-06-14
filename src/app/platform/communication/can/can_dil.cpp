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
   starts at bit 4 (sender_id is the low nibble). The RX filter matches on it. */
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

/* FDCAN latches CCCR.INIT and stops participating in the bus — RX and ACK included —
   when it enters bus-off, and it does NOT self-recover: software must clear INIT. This
   bites when the peer power-cycles. A frame queued with auto-retransmission gets no ACK
   while the peer is down, the transmit error counter runs to bus-off, INIT latches, and
   the link never returns even after the peer is back (the deaf node won't ACK, so the
   peer bus-offs too). Detect bus-off here and kick off the standard FDCAN recovery:
   clearing INIT makes the controller wait for 128x11 recessive bits, then rejoin. We
   keep the HAL State as-is (still "started") and just toggle the register, so the next
   receive()/send() keep working once the bus is healthy again. Clearing INIT when it is
   already clear (during the recovery wait) is a harmless no-op. */
void recoverIfBusOff()
{
    if (s_hfdcan == nullptr) {
        return;
    }
    if ((s_hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0u) {
        CLEAR_BIT(s_hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);  // begin bus-off recovery
    }
}

/* CAN-FD data-length-code <-> byte-count. The HAL stores the RAW DLC code (0..15) in the
   header DataLength field — it shifts it <<16 itself and indexes a 16-entry DLCtoBytes
   table — so we must hand it the code, NOT a pre-shifted value. Codes 0..8 == byte count
   (FDCAN_DLC_BYTES_0..8 == 0..8); codes 9..15 map to 12/16/20/24/32/48/64. A payload that
   is not an exact FD length rides the next larger frame (padding; our records are
   fixed-size so the receiver knows the real length regardless). */
uint32_t bytesToDlc(std::size_t n)
{
    if (n <= 8)  return static_cast<uint32_t>(n);   // FDCAN_DLC_BYTES_0..8 are the codes 0..8
    if (n <= 12) return FDCAN_DLC_BYTES_12;
    if (n <= 16) return FDCAN_DLC_BYTES_16;
    if (n <= 20) return FDCAN_DLC_BYTES_20;
    if (n <= 24) return FDCAN_DLC_BYTES_24;
    if (n <= 32) return FDCAN_DLC_BYTES_32;
    if (n <= 48) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

std::size_t dlcToBytes(uint32_t dlc)
{
    static const uint8_t lut[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    return lut[dlc & 0xFu];
}

} // namespace

/* -------------------------------------------------------------------------- */
/* Platform DIL entry point                                                   */
/* -------------------------------------------------------------------------- */

namespace platform::communication::can {

/* ---- Live FDCAN diagnostics (debug aid; watch g_can_diag in the debugger) -------- *
 * Snapshotted from the peripheral every receive() (i.e. every controller tick). Tells
 * apart the two failure modes of a bench-less FD bring-up:
 *   - wire / data-phase problem: tec/rec climb, ep/bo set, dlec != 0, busoff_events rises.
 *   - overrun:                   rxf0_lost rises (RX FIFO0 message lost), rxf0_full,
 *                                tx_fifo_full rises, or s_info.rx_dropped (ring) climbs.
 *   - link is actually working:  redl == 1 and rbrs == 1 (an FD/BRS frame was received),
 *                                tdcv ~= 10 (measured loop delay in tq), counters near 0.
 * Pure status reads (non-destructive) except the deliberate RF0L clear. */
struct CanDiag {
    uint32_t psr;            // raw protocol status register
    uint32_t ecr;            // raw error counter register
    uint8_t  tec;            // transmit error counter
    uint8_t  rec;            // receive error counter
    uint8_t  lec;            // last error code (arbitration phase): 0 none .. 6 crc, 7 no change
    uint8_t  dlec;           // last error code (DATA phase) — the key one for FD/BRS
    uint8_t  act;            // activity: 0 sync, 1 idle, 2 receiver, 3 transmitter
    uint8_t  tdcv;           // measured transmitter delay comp value (data tq)
    bool     ep;             // error passive
    bool     ew;             // error warning
    bool     bo;             // bus off
    bool     redl;           // an FD (EDL) frame was received since reset
    bool     rbrs;           // a bit-rate-switched (BRS) frame was received since reset
    uint8_t  rxf0_fill;      // RX FIFO0 fill level
    bool     rxf0_full;      // RX FIFO0 full
    uint32_t rxf0_lost;      // RX FIFO0 message-lost (overrun) events
    uint32_t busoff_events;  // times the node entered bus-off
    uint32_t tx_fifo_full;   // times send() found the TX FIFO full
};
__attribute__((used)) CanDiag g_can_diag{};

void snapshotDiag()
{
    if (s_hfdcan == nullptr) {
        return;
    }
    FDCAN_GlobalTypeDef* const p = s_hfdcan->Instance;
    const uint32_t psr = p->PSR;
    const uint32_t ecr = p->ECR;

    g_can_diag.psr  = psr;
    g_can_diag.ecr  = ecr;
    g_can_diag.tec  = static_cast<uint8_t>(ecr & 0xFFu);
    g_can_diag.rec  = static_cast<uint8_t>((ecr >> 8) & 0x7Fu);
    g_can_diag.lec  = static_cast<uint8_t>(psr & 0x7u);
    g_can_diag.dlec = static_cast<uint8_t>((psr >> 8) & 0x7u);
    g_can_diag.act  = static_cast<uint8_t>((psr >> 3) & 0x3u);
    g_can_diag.tdcv = static_cast<uint8_t>((psr >> 16) & 0x7Fu);
    g_can_diag.ep   = (psr & FDCAN_PSR_EP)   != 0u;
    g_can_diag.ew   = (psr & FDCAN_PSR_EW)   != 0u;
    g_can_diag.redl = (psr & FDCAN_PSR_REDL) != 0u;
    g_can_diag.rbrs = (psr & FDCAN_PSR_RBRS) != 0u;

    const bool bo = (psr & FDCAN_PSR_BO) != 0u;
    static bool s_prev_bo = false;
    if (bo && !s_prev_bo) {
        ++g_can_diag.busoff_events;   // rising edge: entered bus-off
    }
    s_prev_bo     = bo;
    g_can_diag.bo = bo;

    const uint32_t rxf0s = p->RXF0S;
    g_can_diag.rxf0_fill = static_cast<uint8_t>(rxf0s & 0x7Fu);
    g_can_diag.rxf0_full = (rxf0s & FDCAN_RXF0S_F0F) != 0u;

    if ((p->IR & FDCAN_IR_RF0L) != 0u) {   // RX FIFO0 message lost = overrun
        ++g_can_diag.rxf0_lost;
        p->IR = FDCAN_IR_RF0L;             // clear (write 1)
    }
}

std::optional<CanError> Can::init(FDCAN_HandleTypeDef* hfdcan, uint8_t nodeId)
{
    s_hfdcan = hfdcan;

    const auto fail = [] {
        s_info.state = CanState::Error;
        return std::optional<CanError>{CanError::InternalError};
    };

    // --- Enable CAN-FD with bit-rate switching (config in code) ---------------------
    // Write the FD framing + bit-timing registers DIRECTLY, while the peripheral is still
    // in config mode (CCCR.INIT|CCE left set by MX_FDCAN1_Init, before HAL_FDCAN_Start
    // below) — the same window the DAR bit below uses. We must NOT call HAL_FDCAN_Init a
    // second time: it re-runs FDCAN_CalculateRamBlockAddresses, which re-lays the message
    // RAM and re-writes the TX FIFO config, and on the ECU's deeper TX FIFO (16 elements vs
    // the FCU's 2) that hard-faults. Writing the timing registers leaves the working RAM
    // layout from MX_FDCAN1_Init untouched. Both rates come from the 50 MHz PLL1Q FDCAN
    // clock (HSE 25 MHz, /M2 x N16 = 200 MHz VCO, /Q4); register fields encode (value - 1):
    //   nominal/arbitration: presc 1, 1 + 39 + 10 = 50 tq -> 1 Mbit (SP 80%)
    //   data phase (BRS):     presc 1, 1 + 19 +  5 = 25 tq -> 2 Mbit (SP 80%)
    // 2 Mbit (not the ISOW1044's 5 Mbit max) keeps its ~205 ns isolated loop delay at ~41%
    // of the data bit -> wide TDC margin, safe without bench validation. To dial back, edit
    // the seg/prescaler values below, or clear FDOE/BRSE for classic CAN. (Element sizes
    // stay 8 bytes here; widening to 64 — collapsing 8 fragments/record to 1 — lands later.)
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);  // FD + bit-rate switch

    hfdcan->Instance->NBTP =
        (9u  << FDCAN_NBTP_NSJW_Pos)   |   // SJW 10
        (0u  << FDCAN_NBTP_NBRP_Pos)   |   // prescaler 1
        (38u << FDCAN_NBTP_NTSEG1_Pos) |   // TSeg1 39
        (9u  << FDCAN_NBTP_NTSEG2_Pos);    // TSeg2 10

    hfdcan->Instance->DBTP =
        (0u  << FDCAN_DBTP_DBRP_Pos)   |   // prescaler 1
        (18u << FDCAN_DBTP_DTSEG1_Pos) |   // TSeg1 19
        (4u  << FDCAN_DBTP_DTSEG2_Pos) |   // TSeg2 5
        (4u  << FDCAN_DBTP_DSJW_Pos)   |   // SJW 5
        FDCAN_DBTP_TDC;                    // enable transmitter delay compensation

    // Secondary sample point past the ~205 ns loop delay: offset 19 data tq (~380 ns),
    // no filter window.
    hfdcan->Instance->TDCR = (19u << FDCAN_TDCR_TDCO_Pos);

    // Disable automatic retransmission (CCCR.DAR). With it on, a frame that is not
    // ACKed — e.g. while the peer is power-cycled — is retried forever, runs the error
    // counter to bus-off, and (with the recovery above) just bus-off/recovers in a loop.
    // With it off, an un-ACKed frame is dropped after one attempt, so the link stays
    // healthy when the peer is absent. Set here in the driver (config-in-code) rather
    // than the CubeMX .ioc, while the peripheral is still in config mode (INIT/CCE set
    // by MX_FDCAN1_Init, before HAL_FDCAN_Start below).
    //
    // The trade: CAN TX is now best-effort. That is fine for ECU->FCU telemetry (already
    // downsampled / best-effort), but FCU->ECU commands must NOT be fire-and-forget —
    // each needs its own response + retry at the application layer (see Control).
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_DAR);

    FDCAN_FilterTypeDef filter{};
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = static_cast<uint32_t>(nodeId) << TARGET_ID_SHIFT_BITS;
    filter.FilterID2    = TARGET_ID_MASK;

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

    /* ActivateNotification only unmasks the interrupt at the peripheral; the CPU
       still won't vector unless the NVIC line is enabled. CubeMX does not emit this
       (FDCAN1 is not ticked in the .ioc NVIC tab), so the driver owns it here — this
       is what makes HAL_FDCAN_RxFifo0Callback (and thus receive()) fire. Both boards
       wire FDCAN1, so line IT0 is the RX FIFO0 interrupt for each. */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

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
    txHeader.DataLength          = bytesToDlc(frame.length);  // FD DLC for the payload length
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_ON;    // switch to the fast (2 Mbit) data phase
    txHeader.FDFormat            = FDCAN_FD_CAN;     // CAN-FD frame (was classic)
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) == 0) {
        s_info.status.tx_error = 1u;
        ++g_can_diag.tx_fifo_full;
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
    recoverIfBusOff();  // self-heal a latched bus-off (e.g. after the peer power-cycled)
    snapshotDiag();     // refresh g_can_diag (error counters / FD status) for the debugger

    if (s_rxRing.tail == s_rxRing.head) {
        return std::nullopt;  // ring empty
    }

    const RxFrame& msg = s_rxRing.frames[s_rxRing.tail];

    CanFrame frame;
    frame.id     = msg.header.Identifier;
    // FD DLC -> byte count, clamped to the buffer (the RX element is 8 bytes; never let a
    // reported length exceed frame.data so downstream consumers can't over-read).
    const std::size_t rx_bytes = dlcToBytes(msg.header.DataLength);
    frame.length = static_cast<uint8_t>(rx_bytes < MAX_PAYLOAD_LENGTH_BYTES ? rx_bytes
                                                                            : MAX_PAYLOAD_LENGTH_BYTES);
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
