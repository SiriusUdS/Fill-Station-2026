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

// Issue one WREG and block until the frame completes (or times out), so the
// init sequence configures registers one at a time without racing the bus.
void write_register_blocking(uint8_t address, uint16_t value)
{
    const std::array<uint8_t, 6> frame = make_write_frame(address, value);

    const uint32_t start = HAL_GetTick();
    while (spi::transfer(BUS, frame) == SpiError::Busy) {
        if (HAL_GetTick() - start > WRITE_TIMEOUT_MS) {
            return;  // bus stuck; give up on this write
        }
    }
    while (!spi::receive(BUS)) {
        if (HAL_GetTick() - start > WRITE_TIMEOUT_MS) {
            return;  // frame never completed; give up
        }
    }
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
    sample.state             = AdcState::Streaming;
    sample.status.initialized = 1u;
    sample.status.data_valid  = 1u;
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

    REG_GAIN_t GAIN_REG;
    GAIN_REG.all = 0;
    GAIN_REG.bits.PGAGAIN0 = 0b101; // 101 = 32 gain
    GAIN_REG.bits.PGAGAIN1 = 0b101;
    GAIN_REG.bits.PGAGAIN2 = 0b101;
    GAIN_REG.bits.PGAGAIN3 = 0b101;
    write_register_blocking(REG_ADDR_GAIN, GAIN_REG.all);

    REG_GAIN2_t GAIN_REG2;
    GAIN_REG2.all = 0;
    GAIN_REG2.bits.PGAGAIN4 = 0b101;
    GAIN_REG2.bits.PGAGAIN5 = 0b101;
    GAIN_REG2.bits.PGAGAIN6 = 0b101;
    GAIN_REG2.bits.PGAGAIN7 = 0b101;
    write_register_blocking(REG_ADDR_GAIN2, GAIN_REG2.all);
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
