/**
 ******************************************************************************
 * @file    acquisition/adc/ads131m08.cpp
 * @brief   TI ADS131M08 8-channel 24-bit ADC driver. Binds the SPI bus and CS,
 *          configures clock/OSR and per-channel PGA gain, and acquires
 *          DRDY-paced: each DRDY edge kicks one frame DMA and the completion ISR
 *          parses the channel words into per-channel counts, published through
 *          the logic ADC seam (logic::communication::adc::samples).
 *
 *          Board wiring (SPI handle, CS, DRDY pin/IRQ) is supplied by bring-up
 *          through Config - the driver hardcodes no pin numbers, so it ports to
 *          boards with a different pinout. The board configures the DRDY pin as
 *          a falling-edge EXTI input and provides the matching EXTI vector
 *          handler (its symbol is fixed by the pin group); this driver only arms
 *          the line (NVIC) in start() and filters HAL_GPIO_EXTI_Callback by the
 *          configured DRDY pin.
 ******************************************************************************
 */

#include "acquisition/adc/ads131m08.hpp"
#include "communication/spi/spi_dil.hpp"      // platform SPI seam: init/transfer/receive
#include "communication/spi/spi_dma.hpp"      // dma::set_frame_callback (per-frame parse)
#include "communication/interfaces/adc.hpp"   // logic ADC seam this driver defines

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "stm32h7xx_hal.h"   // HAL_GPIO_Init / HAL_NVIC_* / HAL_GetTick

namespace spi = platform::communication::spi;
namespace adc_if = logic::communication::adc;
using spi::SpiBus;
using spi::SpiError;

namespace {

// Latest conversion, one signed count per channel. Written by the DMA ISR
// (on_frame) and read by the ADC seam samples(); single-producer (ISR) /
// single-consumer (caller). At file scope so both the driver and the seam
// definition below can reach it. samples() returns a view over s_latest while
// the ISR may overwrite it, so callers consume it promptly.
std::array<int32_t, adc_if::CHANNEL_COUNT> s_latest{};
volatile bool s_new = false;

// Optional per-sample callback, invoked from on_frame (ISR) after parsing.
adc_if::SampleCallback s_sample_cb = nullptr;

} // namespace

namespace platform::acquisition::adc::ads131m08 {

namespace {

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

// Board wiring captured at init(). The CS arrays back the BusConfig (which keeps
// pointers into them), so they have static lifetime.
Config        s_cfg{};
GPIO_TypeDef* s_cs_ports[1] = {};
uint16_t      s_cs_pins[1]  = {};

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

// Per-frame completion callback (DMA ISR context): parse the channel words out
// of one frame into s_latest. Kept short - no SPI or blocking work here.
void on_frame(std::span<const uint8_t> frame)
{
    const std::size_t need = STATUS_BYTES + adc_if::CHANNEL_COUNT * WORD_BYTES;
    if (frame.size() < need) {
        return;  // short frame; leave the last good sample in place
    }
    for (std::size_t i = 0; i < adc_if::CHANNEL_COUNT; ++i) {
        const uint8_t* w = frame.data() + STATUS_BYTES + i * WORD_BYTES;
        const uint32_t raw = (static_cast<uint32_t>(w[0]) << 16) |
                             (static_cast<uint32_t>(w[1]) << 8) |
                              static_cast<uint32_t>(w[2]);
        s_latest[i] = sign_extend_24(raw);
    }
    s_new = true;

    if (s_sample_cb != nullptr) {
        s_sample_cb(std::span<const int32_t>(s_latest.data(), s_latest.size()));
    }
}

} // namespace

void write_register(uint8_t address, uint16_t value)
{
    const std::array<uint8_t, 6> frame = make_write_frame(address, value);
    (void)spi::transfer(BUS, frame);
}

void init(const Config& config)
{
    s_cfg         = config;
    s_cs_ports[0] = config.cs_port;
    s_cs_pins[0]  = config.cs_pin;

    // Bind the SPI bus to the ADS131M08 frame size and CS before issuing any
    // command. The driver drives CS low around each command and acquisition
    // frame.
    spi::BusConfig bus{};
    bus.hspi         = config.hspi;
    bus.cs_ports     = s_cs_ports;
    bus.cs_pins      = s_cs_pins;
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

void start()
{
    // Parse each completed frame straight from the SPI completion ISR.
    spi::dma::set_frame_callback(&on_frame);

    // The board has already configured the DRDY pin as a falling-edge EXTI
    // input; arm it. Each edge then kicks one frame via HAL_GPIO_EXTI_Callback.
    HAL_NVIC_SetPriority(s_cfg.drdy_irqn, DRDY_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(s_cfg.drdy_irqn);
}

void stop()
{
    HAL_NVIC_DisableIRQ(s_cfg.drdy_irqn);
    spi::dma::set_frame_callback(nullptr);
}

// DRDY edge dispatch. The board's EXTI vector handler calls
// HAL_GPIO_EXTI_IRQHandler, which lands here; kick one NULL frame to clock the
// channels back when it is our DRDY pin.
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == s_cfg.drdy_pin) {
        (void)spi::transfer(BUS, std::span<const uint8_t>{});
    }
}

} // namespace platform::acquisition::adc::ads131m08

/* -------------------------------------------------------------------------- */
/* Logic-side seam: definition for communication/interfaces/adc.hpp           */
/* -------------------------------------------------------------------------- */

namespace logic::communication::adc {

std::optional<std::span<const int32_t>> samples()
{
    if (!s_new) {
        return std::nullopt;
    }
    s_new = false;
    return std::span<const int32_t>(s_latest.data(), s_latest.size());
}

void set_sample_callback(SampleCallback cb)
{
    s_sample_cb = cb;
}

} // namespace logic::communication::adc
