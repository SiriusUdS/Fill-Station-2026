#pragma once

/* ------------------------------------------------------------------------- *
 * TI ADS131M08 8-channel 24-bit delta-sigma ADC driver.
 *
 * Talks to the device over SPI4, the platform DMA SPI transport
 * (communication/spi/spi_dma.hpp); SPI is a private implementation detail of
 * this driver. The logic layer consumes the conversions through the ADC seam
 * (logic/communication/interfaces/adc.hpp), which this driver defines.
 *
 * init() binds the bus/CS and configures clock/OSR and per-channel gain;
 * start() begins DRDY-paced acquisition - each DRDY edge clocks one frame, which
 * is parsed and pushed into the ring the controller drains through pop().
 *
 * The REG_* unions below mirror the on-chip register map (16-bit words); write
 * the bitfields, then hand reg.all to write_register().
 * ------------------------------------------------------------------------- */

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "stm32h7xx_hal.h"                   // SPI_HandleTypeDef, GPIO_TypeDef, IRQn_Type (Config)
#include "communication/interfaces/adc.hpp"  // logic::communication::StreamingAdc contract

namespace platform::acquisition::adc::ads131m08 {

constexpr uint16_t MSB_UINT16_MASK = 0xFF00;
constexpr uint16_t LSB_UINT16_MASK = 0x00FF;

constexpr uint16_t MESSAGE_LENGTH = 30;

// SPI command opcodes (top 3 bits of the command word).
constexpr uint8_t WREG_CMD = 0b011;
constexpr uint8_t RREG_CMD = 0b101;
constexpr uint8_t NULL_CMD = 0;

// ---------------------------
// ADS131M08 Register Addresses
// ---------------------------

// DEVICE SETTINGS AND INDICATORS
constexpr uint8_t REG_ADDR_ID          = 0x00;
constexpr uint8_t REG_ADDR_STATUS      = 0x01;

// GLOBAL SETTINGS ACROSS CHANNELS
constexpr uint8_t REG_ADDR_MODE        = 0x02;
constexpr uint8_t REG_ADDR_CLOCK       = 0x03;
constexpr uint8_t REG_ADDR_GAIN        = 0x04;
constexpr uint8_t REG_ADDR_GAIN2       = 0x05;
constexpr uint8_t REG_ADDR_CFG         = 0x06;
constexpr uint8_t REG_ADDR_THRSHLD_MSB = 0x07;
constexpr uint8_t REG_ADDR_THRSHLD_LSB = 0x08;

// CHANNEL 0
constexpr uint8_t REG_ADDR_CH0_CFG      = 0x09;
constexpr uint8_t REG_ADDR_CH0_OCAL_MSB = 0x0A;
constexpr uint8_t REG_ADDR_CH0_OCAL_LSB = 0x0B;
constexpr uint8_t REG_ADDR_CH0_GCAL_MSB = 0x0C;
constexpr uint8_t REG_ADDR_CH0_GCAL_LSB = 0x0D;

// CHANNEL 1
constexpr uint8_t REG_ADDR_CH1_CFG      = 0x0E;
constexpr uint8_t REG_ADDR_CH1_OCAL_MSB = 0x0F;
constexpr uint8_t REG_ADDR_CH1_OCAL_LSB = 0x10;
constexpr uint8_t REG_ADDR_CH1_GCAL_MSB = 0x11;
constexpr uint8_t REG_ADDR_CH1_GCAL_LSB = 0x12;

// CHANNEL 2
constexpr uint8_t REG_ADDR_CH2_CFG      = 0x13;
constexpr uint8_t REG_ADDR_CH2_OCAL_MSB = 0x14;
constexpr uint8_t REG_ADDR_CH2_OCAL_LSB = 0x15;
constexpr uint8_t REG_ADDR_CH2_GCAL_MSB = 0x16;
constexpr uint8_t REG_ADDR_CH2_GCAL_LSB = 0x17;

// CHANNEL 3
constexpr uint8_t REG_ADDR_CH3_CFG      = 0x18;
constexpr uint8_t REG_ADDR_CH3_OCAL_MSB = 0x19;
constexpr uint8_t REG_ADDR_CH3_OCAL_LSB = 0x1A;
constexpr uint8_t REG_ADDR_CH3_GCAL_MSB = 0x1B;
constexpr uint8_t REG_ADDR_CH3_GCAL_LSB = 0x1C;

// REGISTER MAP CRC AND RESERVED
constexpr uint8_t REG_ADDR_REGMAP_CRC = 0x3E;
constexpr uint8_t REG_ADDR_RESERVED   = 0x3F;

// ---------------------------
// 0x02 MODE register
// ---------------------------
union REG_MODE_t {
    uint16_t all;
    struct {
        uint16_t DRDY_FMT   : 1;  // Bit 0
        uint16_t DRDY_HiZ   : 1;  // Bit 1
        uint16_t DRDY_SEL   : 2;  // Bits 3:2
        uint16_t TIMEOUT    : 1;  // Bit 4
        uint16_t RESERVED0  : 3;  // Bits 7:5
        uint16_t WLENGTH    : 2;  // Bits 9:8
        uint16_t RESET      : 1;  // Bit 10
        uint16_t CRC_TYPE   : 1;  // Bit 11
        uint16_t RX_CRC_EN  : 1;  // Bit 12
        uint16_t REG_CRC_EN : 1;  // Bit 13
        uint16_t RESERVED1  : 2;  // Bits 15:14
    } bits;
};

// ---------------------------
// 0x03 CLOCK register
// ---------------------------
union REG_CLOCK_t {
    uint16_t all;
    struct {
        uint16_t PWR_       : 2;  // Bits 1:0
        uint16_t OSR        : 3;  // Bits 4:2
        uint16_t TBM        : 1;  // Bit 5
        uint16_t RESERVED0  : 2;  // Bits 7:6
        uint16_t CH0_EN     : 1;  // Bit 8
        uint16_t CH1_EN     : 1;  // Bit 9
        uint16_t CH2_EN     : 1;  // Bit 10
        uint16_t CH3_EN     : 1;  // Bit 11
        uint16_t RESERVED1  : 4;  // Bits 15:12
    } bits;
};

// ---------------------------
// 0x04 GAIN register
// ---------------------------
union REG_GAIN_t {
    uint16_t all;
    struct {
        uint16_t PGAGAIN0   : 3;  // Bits 2:0
        uint16_t RESERVED0  : 1;  // Bit 3
        uint16_t PGAGAIN1   : 3;  // Bits 6:4
        uint16_t RESERVED1  : 1;  // Bit 7
        uint16_t PGAGAIN2   : 3;  // Bits 10:8
        uint16_t RESERVED2  : 1;  // Bit 11
        uint16_t PGAGAIN3   : 3;  // Bits 14:12
        uint16_t RESERVED3  : 1;  // Bit 15
    } bits;
};

// ---------------------------
// 0x05 GAIN2 register
// ---------------------------
union REG_GAIN2_t {
    uint16_t all;
    struct {
        uint16_t PGAGAIN4   : 3;  // Bits 2:0
        uint16_t RESERVED0  : 1;  // Bit 3
        uint16_t PGAGAIN5   : 3;  // Bits 6:4
        uint16_t RESERVED1  : 1;  // Bit 7
        uint16_t PGAGAIN6   : 3;  // Bits 10:8
        uint16_t RESERVED2  : 1;  // Bit 11
        uint16_t PGAGAIN7   : 3;  // Bits 14:12
        uint16_t RESERVED3  : 1;  // Bit 15
    } bits;
};

// ---------------------------
// 0x06 CFG register
// ---------------------------
union REG_CFG_t {
    uint16_t all;
    struct {
        uint16_t CD_EN      : 1;  // Bit 0
        uint16_t CD_LEN     : 3;  // Bits 3:1
        uint16_t CD_NUM     : 3;  // Bits 6:4
        uint16_t CD_ALLCH   : 1;  // Bit 7
        uint16_t GC_EN      : 1;  // Bit 8
        uint16_t GC_DLY     : 4;  // Bits 12:9
        uint16_t RESERVED   : 3;  // Bits 15:13
    } bits;
};

// ---------------------------
// 0x07 THRSHLD_MSB register
// ---------------------------
union REG_THRSHLD_MSB_t {
    uint16_t all;
    struct {
        uint16_t CD_TH_MSB  : 16; // Bits 15:0
    } bits;
};

// ---------------------------
// 0x08 THRSHLD_LSB register
// ---------------------------
union REG_THRSHLD_LSB_t {
    uint16_t all;
    struct {
        uint16_t DCBLOCK    : 4;  // Bits 3:0
        uint16_t RESERVED0  : 4;  // Bits 7:4
        uint16_t CD_TH_LSB  : 8;  // Bits 15:8
    } bits;
};

/* ============================================================
 * ADS131M08 CHANNEL REGISTER BITFIELD STRUCTURES
 * ============================================================ */

// Channel 0

/* ===================== CH0_CFG (0x09) ===================== */
union ADS131_CH0_CFG_t {
    uint16_t reg;
    struct {
        uint16_t MUX0        : 2;   // Bits 1:0
        uint16_t DCBLK0_DIS  : 1;   // Bit 2
        uint16_t RESERVED    : 3;   // Bits 5:3 (write 000)
        uint16_t PHASE0      : 10;  // Bits 15:6 (two's complement)
    } bit;
};

/* ================== CH0_OCAL_MSB (0x0A) =================== */
union ADS131_CH0_OCAL_MSB_t {
    uint16_t reg;
    struct {
        uint16_t OCAL0_MSB : 16;   // Bits 23:8 of offset calibration
    } bit;
};

/* ================== CH0_OCAL_LSB (0x0B) =================== */
union ADS131_CH0_OCAL_LSB_t {
    uint16_t reg;
    struct {
        uint16_t RESERVED    : 8;   // Bits 7:0 (always 0)
        uint16_t OCAL0_LSB   : 8;   // Bits 15:8
    } bit;
};

/* ================== CH0_GCAL_MSB (0x0C) =================== */
union ADS131_CH0_GCAL_MSB_t {
    uint16_t reg;
    struct {
        uint16_t GCAL0_MSB : 16;   // Bits 23:8 of gain calibration
    } bit;
};

/* ================== CH0_GCAL_LSB (0x0D) =================== */
union ADS131_CH0_GCAL_LSB_t {
    uint16_t reg;
    struct {
        uint16_t RESERVED    : 8;   // Bits 7:0 (always 0)
        uint16_t GCAL0_LSB   : 8;   // Bits 15:8
    } bit;
};

// Channel 1

/* ===================== CH1_CFG (0x0E) ===================== */
union ADS131_CH1_CFG_t {
    uint16_t reg;
    struct {
        uint16_t MUX1        : 2;   // Bits 1:0
        uint16_t DCBLK1_DIS  : 1;   // Bit 2
        uint16_t RESERVED    : 3;   // Bits 5:3
        uint16_t PHASE1      : 10;  // Bits 15:6
    } bit;
};

/* ================== CH1_OCAL_MSB (0x0F) =================== */
union ADS131_CH1_OCAL_MSB_t {
    uint16_t reg;
    struct {
        uint16_t OCAL1_MSB : 16;   // Bits 23:8
    } bit;
};

/* ================== CH1_OCAL_LSB (0x10) =================== */
union ADS131_CH1_OCAL_LSB_t {
    uint16_t reg;
    struct {
        uint16_t RESERVED    : 8;   // Bits 7:0
        uint16_t OCAL1_LSB   : 8;   // Bits 15:8
    } bit;
};

/* ================== CH1_GCAL_MSB (0x11) =================== */
union ADS131_CH1_GCAL_MSB_t {
    uint16_t reg;
    struct {
        uint16_t GCAL1_MSB : 16;   // Bits 15:0
                                   // Gain calibration bits [23:8]
                                   // Reset = 0x8000
    } bit;
};

/* ================== CH1_GCAL_LSB (0x12) =================== */
union ADS131_CH1_GCAL_LSB_t {
    uint16_t reg;
    struct {
        uint16_t RESERVED    : 8;  // Bits 7:0 (always reads 0)
        uint16_t GCAL1_LSB   : 8;  // Bits 15:8
                                   // Gain calibration bits [7:0]
    } bit;
};

// Channel 2

/* ===================== CH2_CFG (0x13) ===================== */
union ADS131_CH2_CFG_t {
    uint16_t reg;
    struct {
        uint16_t MUX2        : 2;   // Bits 1:0
        uint16_t DCBLK2_DIS  : 1;   // Bit 2
        uint16_t RESERVED    : 3;   // Bits 5:3 (always 000)
        uint16_t PHASE2      : 10;  // Bits 15:6 (two's complement)
    } bit;
};

/* ================== CH2_OCAL_MSB (0x14) =================== */
union ADS131_CH2_OCAL_MSB_t {
    uint16_t reg;
    struct {
        uint16_t OCAL2_MSB : 16;   // Bits 15:0
                                   // Offset calibration bits [23:8]
    } bit;
};

/* ================== CH2_OCAL_LSB (0x15) =================== */
union ADS131_CH2_OCAL_LSB_t {
    uint16_t reg;
    struct {
        uint16_t RESERVED   : 8;   // Bits 7:0 (always reads 0)
        uint16_t OCAL2_LSB  : 8;   // Bits 15:8
                                   // Offset calibration bits [7:0]
    } bit;
};

/* ================== CH2_GCAL_MSB (0x16) =================== */
union ADS131_CH2_GCAL_MSB_t {
    uint16_t reg;
    struct {
        uint16_t GCAL2_MSB : 16;   // Bits 15:0
                                   // Gain calibration bits [23:8]
                                   // Reset = 0x8000
    } bit;
};

/* ================== CH2_GCAL_LSB (0x17) =================== */
union ADS131_CH2_GCAL_LSB_t {
    uint16_t reg;
    struct {
        uint16_t RESERVED   : 8;   // Bits 7:0 (always reads 0)
        uint16_t GCAL2_LSB  : 8;   // Bits 15:8
                                   // Gain calibration bits [7:0]
    } bit;
};

// Channel 3

// ---------------------------
// Channel 3 Configuration Register (CH3_CFG, Address 0x18)
// ---------------------------
union CH3_CFG_Reg {
    uint16_t all; // Full 16-bit access
    struct {
        uint16_t MUX3      : 2;  // Bits 1:0 - Input selection
                                   // 00 = AIN3P/AIN3N
                                   // 01 = ADC inputs shorted
                                   // 10 = Positive DC test signal
                                   // 11 = Negative DC test signal
        uint16_t DCBLK3_DIS0 : 1; // Bit 2 - DC block filter disable
                                   // 0 = Controlled by DCBLOCK[3:0] (default)
                                   // 1 = Disabled for this channel
        uint16_t RESERVED     : 3; // Bits 5:3 - Reserved, always reads 000
        int16_t PHASE3        : 10; // Bits 15:6 - Phase delay in modulator clock cycles (two's complement)
    } bits;
};

// ---------------------------
// Channel 3 Offset Calibration
// ---------------------------

// MSB register (0x19)
union CH3_OCAL_MSB_Reg {
    uint16_t all; // full 16-bit access
    struct {
        uint16_t OCAL3_MSB : 8;  // bits 15:8 - offset MSB
        uint16_t unused_MSB : 8; // bits 7:0 (not used for calibration)
    } bits;
};

// LSB register (0x1A)
union CH3_OCAL_LSB_Reg {
    uint16_t all;
    struct {
        uint16_t OCAL3_LSB : 8;  // bits 15:8 - offset LSB
        uint16_t RESERVED  : 8;  // bits 7:0 - always reads 0
    } bits;
};

// ---------------------------
// Channel 3 Gain Calibration
// ---------------------------

// MSB register (0x1B)
union CH3_GCAL_MSB_Reg {
    uint16_t all;
    struct {
        uint16_t GCAL3_MSB : 16;  // bits 15:8 - gain MSB
    } bits;
};

// LSB register (0x1C)
union CH3_GCAL_LSB_Reg {
    uint16_t all;
    struct {
        uint16_t GCAL3_LSB : 8;  // bits 15:8 - gain LSB
        uint16_t unused_LSB : 8; // bits 7:0 (not used for calibration)
    } bits;
};

/**
 * @brief  Per-channel PGA analog gain. The value IS the 3-bit PGAGAINn register code
 *         (GAIN 0x04 for ch0-3, GAIN2 0x05 for ch4-7), so it writes straight into the
 *         bitfield — no magic literal. This is the coarse analog gain ahead of the ADC;
 *         the digital GCAL trim (see gcal_gain) is a separate, fine adjustment near unity.
 */
enum class PgaGain : uint8_t {
    x1   = 0b000,
    x2   = 0b001,
    x4   = 0b010,
    x8   = 0b011,
    x16  = 0b100,
    x32  = 0b101,
    x64  = 0b110,
    x128 = 0b111,
};

/** @brief Reset value of a channel's 24-bit GCAL gain-calibration register = unity (x1 trim).
 *         Use this as the gcal_gain entry for a channel you do not want to trim. */
inline constexpr int32_t GCAL_UNITY = 0x800000;

/**
 * @brief  Board wiring + per-channel calibration for the ADS131M08, supplied by platform
 *         bring-up so the driver carries no board-specific values (it ports across boards).
 *
 * The DRDY pin must be configured by the board as a falling-edge EXTI input, with a matching
 * EXTI vector handler, before start() arms @c drdy_irqn.
 *
 * Calibration is one constant per channel and is board/sensor-specific, so it lives here (set
 * by the board), not baked into the shared driver. The defaults reproduce the historical fixed
 * setting (all channels x32 PGA gain, no offset, unity GCAL), so a board that omits them is
 * unchanged; override any channel's entry to tune it. init() programs all three into the device.
 */
struct Config {
    SPI_HandleTypeDef* hspi;       /**< DMA-capable SPI peripheral handle. */
    GPIO_TypeDef*      cs_port;    /**< Chip-select GPIO port. */
    uint16_t           cs_pin;     /**< Chip-select GPIO pin. */
    uint16_t           drdy_pin;   /**< DRDY GPIO pin, matched in the EXTI callback. */
    IRQn_Type          drdy_irqn;  /**< NVIC line for the DRDY EXTI group; armed by start(). */

    /** Per-channel PGA analog gain (default x32 — the historical fixed setting). */
    std::array<PgaGain, ADC_CHANNEL_COUNT> pga_gain = {
        PgaGain::x1, PgaGain::x1, PgaGain::x1, PgaGain::x1,
        PgaGain::x1, PgaGain::x1, PgaGain::x1, PgaGain::x1,
    };
    /** Per-channel 24-bit signed OCAL offset calibration, in ADC counts; subtracted before
     *  output. 0 = no offset (the chip reset value), so the default array is all zeros. */
    std::array<int32_t, ADC_CHANNEL_COUNT> ocal_offset = {};
    /** Per-channel 24-bit GCAL gain-calibration (fine digital trim). GCAL_UNITY = x1 (no trim);
     *  the default array is all-unity, matching the chip reset. */
    std::array<int32_t, ADC_CHANNEL_COUNT> gcal_gain = {
        GCAL_UNITY, GCAL_UNITY, GCAL_UNITY, GCAL_UNITY,
        GCAL_UNITY, GCAL_UNITY, GCAL_UNITY, GCAL_UNITY,
    };
};

/**
 * @brief  The board's single ADS131M08: a continuous, DRDY-paced, DMA-fed ADC
 *         that owns its latest-sample storage and models the logic-side
 *         logic::communication::StreamingAdc contract.
 *
 * There is one instance per device (the FCU has exactly one, on SPI4); bring-up
 * holds it and init()/start() it. The HAL-free contract members (info(), pop())
 * are what the logic layer consumes; init()/start()/stop()/write_register() are
 * platform-only.
 *
 * Acquisition runs from interrupt context: each DRDY edge kicks one frame, and
 * the SPI/DMA completion lands in handle_frame(). Those ISR entry points are
 * dispatched to this instance from the board's free-function EXTI/DMA hooks, so
 * they are public but are NOT part of the contract — logic never calls them.
 */
class Ads131m08 {
public:
    /** @brief Channels the device streams (8-channel part). */
    static constexpr std::size_t channel_count = ADC_CHANNEL_COUNT;

    Ads131m08() = default;

    /* ---- platform-only bring-up ------------------------------------------ */

    /**
     * @brief  Bind the SPI bus and CS to the device, then configure clock/OSR and
     *         per-channel PGA gain. Blocks until each register write completes (or
     *         times out). Call once at startup, after the SPI peripheral is inited.
     */
    void init(const Config& config);

    /**
     * @brief  Start DRDY-paced acquisition: route this instance's frames, register
     *         the per-frame parser and arm the DRDY interrupt. From then on each
     *         DRDY edge clocks one frame into samples()/the callback, with no
     *         polling. Call after init() and after the board has configured the
     *         DRDY pin as a falling-edge EXTI input.
     */
    void start();

    /** @brief Stop acquisition: disable the DRDY interrupt and unregister the parser. */
    void stop();

    /**
     * @brief  Write one 16-bit register over the bus. @p value is the .all word of a
     *         REG_* union. Asynchronous: the frame may be dropped (Busy) if a
     *         transfer is already in flight.
     */
    void write_register(uint8_t address, uint16_t value);

    /* ---- logic::communication::StreamingAdc contract --------------------- */

    /**
     * @brief  The ADC's latest info record: state + status + the most recent
     *         simultaneous conversion. Kept up to date by the device as it
     *         converts; used for the controller's filler record when the ring is
     *         empty (does NOT consume the ring).
     */
    [[nodiscard]] AdcInfo info() const { return info_; }

    /**
     * @brief  Pop the oldest queued conversion from the ring (a complete AdcInfo,
     *         data_valid set), or std::nullopt if the ring is empty. The DRDY ISR
     *         pushes; the controller's record timer pops — every conversion is
     *         delivered exactly once.
     */
    [[nodiscard]] std::optional<AdcInfo> pop();

    /* ---- ISR entry points (platform dispatch; not the contract) ---------- */

    /** @brief Parse one completed SPI frame into the latest sample (DMA ISR). */
    void handle_frame(std::span<const uint8_t> frame);

    /** @brief On our DRDY edge, kick one frame to clock the channels back (EXTI ISR). */
    void handle_drdy(uint16_t gpio_pin);

private:
    static constexpr std::size_t RING_SIZE = 32;  // conversions buffered between drains (power of two)

    Config        cfg_{};
    GPIO_TypeDef* cs_ports_[1] = {};  /**< Backs BusConfig; instance-lived (instance is static). */
    uint16_t      cs_pins_[1]  = {};
    AdcInfo       info_{};            /**< Latest conversion; see info(). */
    bool          config_successful_ = false;  /**< init() GAIN/GAIN2 readback verdict; stamped into every frame. */

    /* Single-producer (DRDY ISR push) / single-consumer (controller pop) ring. */
    AdcInfo              ring_[RING_SIZE]{};
    volatile std::size_t ring_head_ = 0;  // producer index (ISR)
    volatile std::size_t ring_tail_ = 0;  // consumer index
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::communication::StreamingAdc<Ads131m08>);

} // namespace platform::acquisition::adc::ads131m08
