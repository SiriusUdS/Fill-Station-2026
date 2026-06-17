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
#include "communication/protocol/framing/can_header.hpp"   // CAN_ID_PRIORITY_BIT (TX lane selection)

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
   (main loop), so a plain head/tail index pair is enough. Deep enough that the FCU,
   which receives the ECU's full 2 kHz telemetry, can absorb a burst across a slow
   main-loop tick (e.g. one stalled on an SD write) without the hardware FIFO
   overrunning. */
constexpr std::size_t RX_RING_CAPACITY_FRAMES = 64;

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

/* Software TX rings, in TWO priority lanes. The ECU's full-rate telemetry drain hands off a whole
   telemetry slot at once — one 16 KB SD-block slot holds ~292 records = ~292 FD frames — and, if
   the main loop stalled, the drain may chase several slots; the 16-deep hardware TX FIFO cannot hold
   that, so send() queues into a RAM ring and pumpTx() feeds the FIFO as it drains (from the
   TX-FIFO-empty ISR + opportunistically on the main loop).

   Two lanes so a command/response never queues behind that telemetry burst: send() lanes a
   frame by its id priority bit (FrameCanHeader::priority, the id's MSB), and pumpTx() always
   empties the HI lane before the LO lane. Combined with the hardware running in TX-QUEUE mode
   (lowest id transmitted first, set in init()), an urgent frame preempts both the software
   backlog and any telemetry already queued in the hardware — its latency drops to about one
   in-flight frame instead of the ~44 ms it would wait behind a single full LO lane.

   LO is sized to hold one whole telemetry slot's worth of records (≥ ~292 at the 16 KB SD block)
   so a drain burst fits WITHOUT pacing — full-rate, no throttle. The telemetry drain additionally
   honours this ring's fullness as backpressure (it holds its cursor and retries rather than
   dropping), so a transient backlog degrades gracefully instead of silently losing frames; the
   ring size keeps that safety net from ever engaging in steady state. Bump LO higher if a deeper
   backlog (e.g. a longer SD stall) must ride without pacing. HI is small — commands/responses are
   short and infrequent, but a dropped one is serious (counted separately). send() (main loop) is
   the sole producer; pumpTx() (under an FDCAN1_IT0 mask) is the sole consumer. */
constexpr std::size_t TX_RING_LO_FRAMES = 320;   // telemetry: one 16 KB slot (~292 records) + margin
constexpr std::size_t TX_RING_HI_FRAMES = 32;    // commands / responses (low-rate, must not drop)

template <std::size_t Capacity>
struct TxRing {
    std::array<CanFrame, Capacity> frames{};
    volatile uint16_t head = 0;   // producer: send() (main loop)
    volatile uint16_t tail = 0;   // consumer: pumpTx()
};

TxRing<TX_RING_LO_FRAMES> s_txLo;   // telemetry lane
TxRing<TX_RING_HI_FRAMES> s_txHi;   // command/response lane (drained first)

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

/* Append one frame to a lane. Returns false (frame dropped) if the lane is full. Producer
   side, called from send() with FDCAN1_IT0 masked. */
template <std::size_t N>
bool ringPush(TxRing<N>& r, const CanFrame& f)
{
    const uint16_t next = (r.head + 1) % N;
    if (next == r.tail) {
        return false;   // ring full
    }
    r.frames[r.head] = f;
    r.head = next;
    return true;
}

/* True while the hardware TX FIFO/queue has room for another frame. We must NOT use
   HAL_FDCAN_GetTxFifoFreeLevel() here: it reads TXFQS.TFFL, which the hardware forces to 0
   whenever TX-QUEUE mode is selected (TXBC.TFQM=1) — so the free level always looks "full",
   the pump never hands a frame to the peripheral, and the FIFO idles while the software ring
   overflows. The TXFQS.TFQF "full" flag is valid in BOTH FIFO and queue mode, so test that. */
inline bool txHasRoom()
{
    return (s_hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) == 0u;
}

/* Drain one lane into the hardware TX FIFO/queue while it has room. Consumer side. */
template <std::size_t N>
void ringDrainToHw(TxRing<N>& r)
{
    while (r.tail != r.head && txHasRoom()) {
        const CanFrame& f = r.frames[r.tail];

        FDCAN_TxHeaderTypeDef txHeader{};
        txHeader.Identifier          = f.id;
        txHeader.IdType              = FDCAN_EXTENDED_ID;
        txHeader.TxFrameType         = FDCAN_DATA_FRAME;
        txHeader.DataLength          = bytesToDlc(f.length);
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch       = FDCAN_BRS_ON;
        txHeader.FDFormat            = FDCAN_FD_CAN;
        txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker       = 0;

        if (HAL_FDCAN_AddMessageToTxFifoQ(
                s_hfdcan, &txHeader, const_cast<uint8_t*>(f.data.data())) != HAL_OK) {
            s_info.status.tx_error = 1u;
            return;   // leave it queued; retry on the next pump
        }
        s_info.status.tx_error = 0u;
        r.tail = (r.tail + 1) % N;
    }
}

/* Move queued frames into the hardware TX FIFO/queue, HI lane (commands/responses) first so
   they preempt telemetry. The decoupling point of the full-rate downlink: send() drops a
   whole drained half into the LO ring in microseconds, and this drains it onto the wire over
   the following milliseconds. Re-armed by the TX-FIFO-empty interrupt, and kicked by send()
   so an idle FIFO starts transmitting immediately.

   NOT re-entrant: send() (main loop) calls it with FDCAN1_IT0 masked, and the ISR runs on
   that same line, so the two can never overlap — each tail is advanced from one context at a
   time. */
void pumpTx()
{
    if (s_hfdcan == nullptr) {
        return;
    }
    ringDrainToHw(s_txHi);   // commands / responses first
    ringDrainToHw(s_txLo);   // then telemetry
}

} // namespace

/* -------------------------------------------------------------------------- */
/* Platform DIL entry point                                                   */
/* -------------------------------------------------------------------------- */

namespace platform::communication::can {

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
    // the seg/prescaler values below, or clear FDOE/BRSE for classic CAN. (The 64-byte
    // element size — one whole record per frame — is set in the .ioc, not here.)
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
    // The trade: CAN TX is now best-effort. That is fine for ECU->FCU telemetry (a live
    // full-rate stream — a dropped frame is just one stale sample), but FCU->ECU commands
    // must NOT be fire-and-forget — each needs its own response + retry at the application
    // layer (see Control).
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_DAR);

    // TX QUEUE mode (TXBC.TFQM): transmit the LOWEST-id pending frame first instead of in FIFO
    // order. Our id carries priority in its MSB (FrameCanHeader::priority), so this makes the
    // hardware send a queued command/response ahead of any telemetry already sitting in the TX
    // buffers — the last-leg companion to the two software lanes. MX_FDCAN1_Init wrote the TX
    // buffer addresses/count into TXBC; we only flip the mode bit, in the same config window
    // (INIT/CCE set) as the timing/DAR writes, so the RAM layout is untouched.
    SET_BIT(hfdcan->Instance->TXBC, FDCAN_TXBC_TFQM);

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
    // TX FIFO empty: re-arms pumpTx() to refill the hardware FIFO from the software TX ring
    // as it drains, so the ECU's full-rate downlink burst clocks out in the background
    // without the main loop having to pump it. Both notifications land on interrupt line 0
    // (FDCAN1_IT0), already NVIC-enabled below.
    if (HAL_FDCAN_ActivateNotification(s_hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK) {
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
    // Lane the frame by its id priority bit (0 = command/response -> HI, 1 = telemetry -> LO),
    // then pump as much as the hardware will take. The ring absorbs the ECU's full-rate drain
    // burst (a whole telemetry half at once), which the 16-deep hardware FIFO could not;
    // pumpTx() drains it onto the wire over the following milliseconds, re-armed by the
    // TX-FIFO-empty interrupt. send() therefore returns in microseconds and never blocks.
    //
    // FDCAN1_IT0 carries both the RX-FIFO0 and TX-FIFO-empty interrupts; mask it for the brief
    // enqueue + pump so the ISR pump cannot run concurrently (single consumer of each lane's
    // tail). Masking only this line leaves the 2 kHz record timer and the SD DMA untouched.
    const bool urgent = ((frame.id >> CAN_ID_PRIORITY_BIT) & 1u) == 0u;

    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);

    const bool queued = urgent ? ringPush(s_txHi, frame) : ringPush(s_txLo, frame);
    if (!queued) {   // lane full: drop (best-effort), like the old full-FIFO path
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        s_info.status.tx_error = 1u;
        return CanError::InternalError;
    }

    pumpTx();   // kick the hardware (it may be idle, awaiting the first frame of a burst)

    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    return std::nullopt;
}

std::optional<CanFrame> Can::receive()
{
    recoverIfBusOff();  // self-heal a latched bus-off (e.g. after the peer power-cycled)

    // Opportunistic top-up: the TX-FIFO-empty ISR is the primary pump, but receive() runs
    // every controller tick, so draining the ring here too keeps it moving even if the
    // FIFO never fully emptied (TFE would not have fired). Mask FDCAN1_IT0 so it cannot
    // race the ISR pump on the ring tail.
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    pumpTx();
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

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

/* FDCAN TX FIFO empty interrupt: refill the hardware FIFO from the software TX ring.
   Keeps C linkage so it overrides the HAL's weak symbol. Runs on FDCAN1_IT0 (the same
   line send()/receive() mask around their pumps), so it is the sole pump in flight. */
extern "C" void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef* hfdcan)
{
    (void)hfdcan;
    pumpTx();
}

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
