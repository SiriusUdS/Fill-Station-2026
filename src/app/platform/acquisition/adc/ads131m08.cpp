/**
 ******************************************************************************
 * @file    acquisition/adc/ads131m08.cpp
 * @brief   TI ADS131M08 8-channel 24-bit ADC driver. One Ads131m08 object per
 *          device, owning its latest-sample storage and modelling the logic-side
 *          logic::communication::StreamingAdc contract. Binds the SPI bus and CS,
 *          configures clock/OSR and per-channel PGA gain, and acquires
 *          DRDY-paced: each DRDY edge kicks one frame DMA and the completion ISR
 *          parses the channel words into per-channel counts.
 *
 *          Board wiring (SPI handle, CS, DRDY pin/IRQ) is supplied by bring-up
 *          through Config - the driver hardcodes no pin numbers, so it ports to
 *          boards with a different pinout. The board configures the DRDY pin as
 *          a falling-edge EXTI input and provides the matching EXTI vector
 *          handler (its symbol is fixed by the pin group); this driver only arms
 *          the line (NVIC) in start() and filters HAL_GPIO_EXTI_Callback by the
 *          configured DRDY pin.
 *
 *          The board runs a single ADS131M08, so the ISR hooks (the SPI/DMA frame
 *          completion and the DRDY EXTI callback, both fixed-signature free
 *          functions that carry no `this`) dispatch to the one live instance
 *          through s_instance, set while acquisition is running (start()..stop()).
 *          Supporting several would mean routing by DRDY pin / active transfer,
 *          but the DMA bus serialises transfers anyway, so we keep it to one until
 *          a second device actually exists.
 ******************************************************************************
 */

#include "acquisition/adc/ads131m08.hpp"
#include "communication/spi/spi_dil.hpp"      // platform SPI seam: init/transfer/receive
#include "communication/spi/spi_dma.hpp"      // dma::set_frame_callback (per-frame parse)

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "stm32h7xx_hal.h"   // HAL_GPIO_Init / HAL_NVIC_* / HAL_GetTick

namespace spi = platform::communication::spi;
using spi::SpiBus;
using spi::SpiError;

namespace platform::acquisition::adc::ads131m08 {

namespace {

// The single live device while acquisition is running (start()..stop()), so the
// fixed-signature ISR hooks below can reach the instance that owns the sample.
Ads131m08* s_instance = nullptr;

// The ADS131M08 frames live on the DMA SPI transport (Spi4 selects DMA, the
// peripheral handle itself comes from Config). Not a board pin.
constexpr SpiBus BUS = SpiBus::Spi4;

// DRDY interrupt preemption priority. Below the SPI/DMA completion ISRs
// (priority 0) so a frame in flight finishes cleanly before the next edge.
constexpr uint32_t DRDY_IRQ_PRIO = 5;

// How long a blocking register write waits for its frame to complete at init.
constexpr uint32_t WRITE_TIMEOUT_MS = 10;

// How many times each register write is issued at init, and how long we pause between
// attempts. A WREG that the device does not latch is SILENT - it keeps streaming at its
// power-up defaults, so a dropped write looks exactly like a successful one on DOUT.
// Repeating each write costs a few frames at boot and makes a single dropped frame
// non-fatal; writing the same value again is idempotent, so the extra attempts are
// harmless once one has landed. The pause gives the device time to digest a write that
// restarts conversions (a CLOCK/OSR change does) before the next frame arrives.
constexpr uint8_t  WRITE_ATTEMPTS  = 3;
constexpr uint32_t WRITE_SETTLE_MS = 1;

// Device frame layout: a 3-byte STATUS word, then one 3-byte (24-bit) word per
// channel, then a 3-byte CRC word - MESSAGE_LENGTH (30) bytes total.
constexpr std::size_t WORD_BYTES   = 3;
constexpr std::size_t STATUS_BYTES = WORD_BYTES;

// SPI command/register word: write the bitfields, then ship the two bytes MSB
// first. Layout relies on the little-endian ordering of the command word.
union RW_REG {
    uint16_t COMMAND;
    struct {
        uint16_t REGISTER_NUMBER : 7;
        uint16_t ADDRESS         : 6;
        uint16_t CMD             : 3;
    } field;
    struct {
        uint8_t LSB;
        uint8_t MSB;
    } bytes;
};

// Build the 6-byte WREG frame head (command word + value word, each padded to
// the device's 24-bit word size). The driver zero-fills the rest of the frame.
std::array<uint8_t, 6> make_write_frame(uint8_t address, uint16_t value)
{
    RW_REG command{};
    command.field.REGISTER_NUMBER = 0;
    command.field.ADDRESS         = address;
    command.field.CMD             = WREG_CMD;

    uint8_t valueMSB = (value & MSB_UINT16_MASK) >> 8;
    uint8_t valueLSB = (value & LSB_UINT16_MASK);

    return {command.bytes.MSB, command.bytes.LSB, 0, valueMSB, valueLSB, 0};
}

// The acknowledgment word a MULTI-register RREG reply opens with: 111a aaaa annn nnnn
// (datasheet table 8-11, note 1) - the address and register count we transmitted, echoed
// back. The top 3 bits are 111, NOT the 101 RREG opcode.
//
// This is the strongest transmit-path check the device offers: the address and count in it
// can only be right if our command word actually reached DIN. A mismatch localises the fault
// to the transmit path, separately from the register contents that follow.
constexpr uint16_t RREG_ACK_CMD = 0b111;

uint16_t expected_read_ack(uint8_t address, uint8_t count)
{
    RW_REG ack{};
    ack.field.REGISTER_NUMBER = static_cast<uint16_t>(count - 1);  // n-1 registers
    ack.field.ADDRESS         = address;
    ack.field.CMD             = RREG_ACK_CMD;

    return ack.COMMAND;
}

// Build the 3-byte RREG command word (read @p count registers starting at
// @p address). The device replies on the FOLLOWING frame, so a read is two
// frames: this command, then a NULL frame to clock the reply back.
std::array<uint8_t, 3> make_read_frame(uint8_t address, uint8_t count)
{
    RW_REG command{};
    command.field.REGISTER_NUMBER = static_cast<uint16_t>(count - 1);  // n-1 registers
    command.field.ADDRESS         = address;
    command.field.CMD             = RREG_CMD;

    return {command.bytes.MSB, command.bytes.LSB, 0};
}

// Issue one frame and block until it completes (or times out), returning the
// bytes the device clocked back. std::nullopt on timeout. Single frame in flight
// at a time - used only from init(), before DRDY-paced acquisition starts.
std::optional<std::span<const uint8_t>> transfer_blocking(std::span<const uint8_t> tx)
{
    const uint32_t start = HAL_GetTick();
    while (spi::transfer(BUS, tx) == SpiError::Busy) {
        if (HAL_GetTick() - start > WRITE_TIMEOUT_MS) {
            return std::nullopt;  // bus stuck; give up
        }
    }
    std::optional<std::span<const uint8_t>> rx;
    while (!(rx = spi::receive(BUS))) {
        if (HAL_GetTick() - start > WRITE_TIMEOUT_MS) {
            return std::nullopt;  // frame never completed; give up
        }
    }
    return rx;
}

// Issue one WREG and block until the frame completes (or times out), so the
// init sequence configures registers one at a time without racing the bus. Repeated
// WRITE_ATTEMPTS times with a settle between (see the constant) so one dropped frame
// does not silently leave the register at its power-up default.
void write_register_blocking(uint8_t address, uint16_t value)
{
    const std::array<uint8_t, 6> frame = make_write_frame(address, value);
    for (uint8_t attempt = 0; attempt < WRITE_ATTEMPTS; ++attempt) {
        (void)transfer_blocking(frame);
        HAL_Delay(WRITE_SETTLE_MS);
    }
}

// Read @p count (<=8) consecutive 16-bit registers starting at @p address into
// @p out. Returns false on timeout / short reply. The RREG reply lands on the
// NEXT frame, so we clock the command, discard its (stale) reply, then clock a
// NULL frame whose reply carries the register words - each in the top 16 bits of
// a 24-bit device word (bytes [0],[1] of the word; low byte reserved 0).
//
// Where the data starts in that reply depends on how many registers were asked for
// (datasheet 8.5.1.10.7, figures 8-23 and 8-24):
//   - single register (count == 1): the response word IS the register contents, so the
//     data is in word 0 - there is no acknowledgment word.
//   - multiple registers (count > 1): the reply opens with an RREG acknowledgment word
//     and the register data follows from word 1. That ack echoes the address and count we
//     transmitted, so it is checked here (see expected_read_ack) - it fails a read whose
//     command word never reached the device, rather than letting a garbage compare stand in.
bool read_registers_blocking(uint8_t address, uint8_t count, uint16_t* out)
{
    const std::array<uint8_t, 3> cmd = make_read_frame(address, count);

    if (!transfer_blocking(cmd)) {
        return false;  // command frame never completed
    }
    const std::optional<std::span<const uint8_t>> reply =
        transfer_blocking(std::span<const uint8_t>{});  // NULL frame clocks the reply back
    if (!reply) {
        return false;
    }
    const std::span<const uint8_t> frame = *reply;
    const std::size_t first_data_word = (count > 1) ? 1u : 0u;  // skip the RREG ack on a multi-read
    if (first_data_word != 0u) {
        if (frame.size() < WORD_BYTES) {
            return false;  // no room for the ack word
        }
        const uint16_t ack =
            static_cast<uint16_t>((static_cast<uint16_t>(frame[0]) << 8) | frame[1]);
        if (ack != expected_read_ack(address, count)) {
            return false;  // the device did not echo our command word back
        }
    }
    for (uint8_t i = 0; i < count; ++i) {
        const std::size_t off = (first_data_word + i) * WORD_BYTES;
        if (off + 1 >= frame.size()) {
            return false;  // reply shorter than expected
        }
        out[i] = static_cast<uint16_t>((static_cast<uint16_t>(frame[off]) << 8) | frame[off + 1]);
    }
    return true;
}

// Sign-extend a 24-bit two's-complement sample into a full int32.
int32_t sign_extend_24(uint32_t raw)
{
    return (raw & 0x00800000u) ? static_cast<int32_t>(raw | 0xFF000000u)
                               : static_cast<int32_t>(raw);
}

// Per-frame completion hook (DMA ISR context): forward to the live instance.
void on_frame(std::span<const uint8_t> frame)
{
    if (s_instance != nullptr) {
        s_instance->handle_frame(frame);
    }
}

} // namespace

void Ads131m08::handle_frame(std::span<const uint8_t> frame)
{
    const std::size_t need = STATUS_BYTES + channel_count * WORD_BYTES;
    if (frame.size() < need) {
        return;  // short frame; leave the last good sample in place
    }
    AdcInfo sample{};
    sample.state                   = AdcState::Streaming;
    sample.status.initialized      = 1u;
    sample.status.data_valid       = 1u;
    sample.status.config_successful = config_successful_ ? 1u : 0u;  // carry the init-time verdict
    for (std::size_t i = 0; i < channel_count; ++i) {
        const uint8_t* w = frame.data() + STATUS_BYTES + i * WORD_BYTES;
        const uint32_t raw = (static_cast<uint32_t>(w[0]) << 16) |
                             (static_cast<uint32_t>(w[1]) << 8) |
                              static_cast<uint32_t>(w[2]);
        sample.channels[i] = sign_extend_24(raw);
    }
    info_ = sample;  // latest, for info() / the controller's filler

    // Push into the SPSC ring for the controller to drain. Drop on overflow (the
    // consumer fell behind beyond RING_SIZE conversions).
    const std::size_t next = (ring_head_ + 1) % RING_SIZE;
    if (next != ring_tail_) {
        ring_[ring_head_] = sample;
        __DMB();               // publish the slot before advancing the head index
        ring_head_ = next;
    }
}

std::optional<AdcInfo> Ads131m08::pop()
{
    if (ring_tail_ == ring_head_) {
        return std::nullopt;   // empty
    }
    __DMB();                   // read the slot the producer published before the index
    const AdcInfo out = ring_[ring_tail_];
    ring_tail_ = (ring_tail_ + 1) % RING_SIZE;
    return out;
}

void Ads131m08::handle_drdy(uint16_t gpio_pin)
{
    if (gpio_pin == cfg_.drdy_pin) {
        (void)spi::transfer(BUS, std::span<const uint8_t>{});
    }
}

void Ads131m08::write_register(uint8_t address, uint16_t value)
{
    const std::array<uint8_t, 6> frame = make_write_frame(address, value);
    (void)spi::transfer(BUS, frame);
}

void Ads131m08::init(const Config& config)
{
    cfg_                     = config;
    cs_ports_[0]             = config.cs_port;
    cs_pins_[0]              = config.cs_pin;
    info_                    = AdcInfo{};   // state Unknown, no channels yet
    info_.status.initialized = 1u;

    // Bind the SPI bus to the ADS131M08 frame size and CS before issuing any
    // command. The driver drives CS low around each command and acquisition
    // frame. cs_ports_/cs_pins_ back the BusConfig pointers, so they must outlive
    // the bus binding - they do, the instance has static lifetime.
    spi::BusConfig bus{};
    bus.hspi         = config.hspi;
    bus.cs_ports     = cs_ports_;
    bus.cs_pins      = cs_pins_;
    bus.cs_num       = 1;
    bus.frame_length = MESSAGE_LENGTH;
    bus.manage_cs    = true;
    spi::init(BUS, bus);

    REG_CLOCK_t CLOCK_REG;
    CLOCK_REG.all = 0xFF0E;
    CLOCK_REG.bits.OSR = 0b100; // osr 2048 = 2kSPS
    write_register_blocking(REG_ADDR_CLOCK, CLOCK_REG.all);

    // PGA analog gain: a 3-bit code per channel. The PgaGain enum value IS that code, so it
    // drops straight into the bitfield. Channels 0-3 live in GAIN (0x04), channels 4-7 in
    // GAIN2 (0x05); reserved bits stay 0 (the .all = 0 clears them).
    REG_GAIN_t GAIN_REG;
    GAIN_REG.all = 0;
    GAIN_REG.bits.PGAGAIN0 = static_cast<uint16_t>(cfg_.pga_gain[0]);
    GAIN_REG.bits.PGAGAIN1 = static_cast<uint16_t>(cfg_.pga_gain[1]);
    GAIN_REG.bits.PGAGAIN2 = static_cast<uint16_t>(cfg_.pga_gain[2]);
    GAIN_REG.bits.PGAGAIN3 = static_cast<uint16_t>(cfg_.pga_gain[3]);
    write_register_blocking(REG_ADDR_GAIN, GAIN_REG.all);

    REG_GAIN2_t GAIN_REG2;
    GAIN_REG2.all = 0;
    GAIN_REG2.bits.PGAGAIN4 = static_cast<uint16_t>(cfg_.pga_gain[4]);
    GAIN_REG2.bits.PGAGAIN5 = static_cast<uint16_t>(cfg_.pga_gain[5]);
    GAIN_REG2.bits.PGAGAIN6 = static_cast<uint16_t>(cfg_.pga_gain[6]);
    GAIN_REG2.bits.PGAGAIN7 = static_cast<uint16_t>(cfg_.pga_gain[7]);
    write_register_blocking(REG_ADDR_GAIN2, GAIN_REG2.all);

    // Per-channel OCAL offset + GCAL gain calibration. The channel register banks are regular:
    // channel n starts at CH0_CFG + n*CH_REG_STRIDE, and within a bank the order is
    // CFG, OCAL_MSB, OCAL_LSB, GCAL_MSB, GCAL_LSB. Each cal value is 24-bit: the MSB register
    // holds bits 23:8, the LSB register holds bits 7:0 in its high byte (low byte reserved 0).
    constexpr uint8_t CH_REG_STRIDE = REG_ADDR_CH1_CFG - REG_ADDR_CH0_CFG;  // 5 registers per channel
    for (std::size_t ch = 0; ch < channel_count; ++ch) {
        const uint8_t base = static_cast<uint8_t>(REG_ADDR_CH0_CFG + ch * CH_REG_STRIDE);

        const uint32_t ocal = static_cast<uint32_t>(cfg_.ocal_offset[ch]) & 0x00FFFFFFu;
        write_register_blocking(base + 1u, static_cast<uint16_t>((ocal >> 8) & 0xFFFFu));   // OCAL_MSB (23:8)
        write_register_blocking(base + 2u, static_cast<uint16_t>((ocal & 0xFFu) << 8));     // OCAL_LSB (7:0)

        const uint32_t gcal = static_cast<uint32_t>(cfg_.gcal_gain[ch]) & 0x00FFFFFFu;
        write_register_blocking(base + 3u, static_cast<uint16_t>((gcal >> 8) & 0xFFFFu));   // GCAL_MSB (23:8)
        write_register_blocking(base + 4u, static_cast<uint16_t>((gcal & 0xFFu) << 8));     // GCAL_LSB (7:0)
    }

    // Config-landed self-check. The ADS131M08 streams conversions in continuous
    // mode with NO register writes at all (that is the power-up default), so a
    // silently dead DIN line still yields plausible frames on DOUT - only a
    // register readback proves the WREGs above actually landed. Read GAIN/GAIN2
    // back (contiguous at 0x04/0x05) and require them to echo exactly what we
    // wrote; their reserved bits are written 0 and read 0, so the compare is
    // clean (no masking). The read also checks the RREG acknowledgment word, which
    // echoes the address and count we transmitted - so the verdict separates a dead
    // transmit path (the command word never arrived, ack wrong) from writes that
    // arrived but did not stick (ack right, contents wrong). A failure at either
    // step clears config_successful, which rides AdcInfo into telemetry so a
    // miswired/misconfigured link shows up instead of masquerading as good data at
    // the wrong gain. Acquisition still starts either way - the flag reports, it
    // does not gate.
    uint16_t readback[2] = {0, 0};
    const bool read_ok = read_registers_blocking(REG_ADDR_GAIN, 2, readback);
    config_successful_ = read_ok &&
                         readback[0] == GAIN_REG.all &&
                         readback[1] == GAIN_REG2.all;
    info_.status.config_successful = config_successful_ ? 1u : 0u;
}

void Ads131m08::start()
{
    // Route this instance's ISR frames, then parse each completed frame straight
    // from the SPI completion ISR.
    s_instance = this;
    spi::dma::set_frame_callback(&on_frame);

    // The board has already configured the DRDY pin as a falling-edge EXTI
    // input; arm it. Each edge then kicks one frame via HAL_GPIO_EXTI_Callback.
    HAL_NVIC_SetPriority(cfg_.drdy_irqn, DRDY_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(cfg_.drdy_irqn);

    info_.state = AdcState::Streaming;  // acquiring from here; handle_frame keeps it current
}

void Ads131m08::stop()
{
    HAL_NVIC_DisableIRQ(cfg_.drdy_irqn);
    spi::dma::set_frame_callback(nullptr);
    s_instance = nullptr;
}

// DRDY edge dispatch. The board's EXTI vector handler calls
// HAL_GPIO_EXTI_IRQHandler, which lands here; forward to the live instance,
// which kicks one NULL frame when it is its DRDY pin.
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (s_instance != nullptr) {
        s_instance->handle_drdy(GPIO_Pin);
    }
}

} // namespace platform::acquisition::adc::ads131m08
