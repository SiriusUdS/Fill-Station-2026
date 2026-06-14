#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "stm32h7xx_hal.h"   // SPI_HandleTypeDef, GPIO_TypeDef, HAL_*

#include "communication/interfaces/thermocouple.hpp"   // logic::communication::ThermocoupleBank
#include "communication/protocol/peripherals/thermocouple/thermocouple_info.hpp"  // ThermocoupleInfo, THERMOCOUPLE_COUNT

/* ------------------------------------------------------------------------- *
 * Maxim MAX31856 thermocouple-to-digital converter bank (the FCU's 4 channels).
 *
 * One bank object owns all THERMOCOUPLE_COUNT devices on a single shared SPI bus
 * (SPI6 on the FCU) plus their individual chip-selects. Acquisition is NON-BLOCKING
 * so it is safe in the main loop: service() drives an interrupt-driven round-robin
 * (one channel at a time) over the platform SPI6 IT transport (spi_dil/spi_irq),
 * harvesting each completed frame from the foreground and timing a stuck transfer
 * out — it never busy-waits on the bus. The async/single-CS spi_dil seam cannot
 * model 4 multiplexed chip-selects, so the bank owns its CS GPIOs and asserts one
 * at a time (manage_cs is off on the shared bus). info() returns the cached
 * snapshot. It models the logic::communication::ThermocoupleBank seam.
 *
 * The one-time register configuration in init() is done with short, bounded
 * blocking writes (acceptable before the loop starts); only the per-loop sampling
 * is interrupt-driven. The SPI peripheral must be configured CPHA = 1 (SPI mode 1
 * or 3) to match the MAX31856.
 * ------------------------------------------------------------------------- */

namespace platform::acquisition::thermocouple::max31856 {

/* ---- Register addresses (read address; write address = addr | WRITE_FLAG) ---- */
constexpr uint8_t REG_CR0  = 0x00;  // config 0: conversion mode, OC detection, CJ enable, 50/60 Hz
constexpr uint8_t REG_CR1  = 0x01;  // config 1: averaging + thermocouple type
constexpr uint8_t REG_MASK = 0x02;  // fault mask (FAULT pin); masked since we poll SR
constexpr uint8_t REG_CJTH = 0x0A;  // cold-junction temp high byte (start of the burst read)
constexpr uint8_t REG_SR   = 0x0F;  // fault status register

constexpr uint8_t WRITE_FLAG = 0x80;

/* ---- CR0 / CR1 field values ---- */
constexpr uint8_t CR0_CMODE_AUTO = 0x80;  // bit7: automatic (continuous) conversion
constexpr uint8_t CR0_OCFAULT_1  = 0x10;  // bits5:4 = 01: open-circuit detection (< 5 kOhm)
constexpr uint8_t CR1_TYPE_K     = 0x03;  // bits3:0 = 0011: type-K thermocouple

/* ---- Cadence + timeout ---- */
constexpr uint32_t POLL_INTERVAL_MS  = 100;  // gap between full 4-channel rounds (auto-convert ~10 Hz)
constexpr uint32_t SAMPLE_TIMEOUT_MS = 5;    // abandon a frame that has not completed in this long
constexpr uint16_t READ_FRAME_BYTES  = 7;    // 1 address byte + 6 burst-read registers (CJTH..SR)

/** @brief Board wiring for the 4-device MAX31856 bank on one shared SPI bus. */
struct Config {
    SPI_HandleTypeDef* hspi = nullptr;                  /**< Shared SPI handle, e.g. &hspi6. */
    GPIO_TypeDef*      cs_ports[THERMOCOUPLE_COUNT]{};  /**< Per-device chip-select port. */
    uint16_t           cs_pins[THERMOCOUPLE_COUNT]{};   /**< Per-device chip-select pin. */
    uint8_t            tc_type = CR1_TYPE_K;            /**< Thermocouple type (CR1[3:0]). */
};

class Max31856Bank {
public:
    Max31856Bank() = default;

    /**
     * @brief  Bind the bus + the 4 chip-selects, drive every CS deasserted, write
     *         each device's configuration (auto-convert, open-circuit detection, TC
     *         type), then arm the SPI6 IT transport for the per-loop reads. The
     *         config writes are short bounded blocking transfers (one-time, before
     *         the loop). Call once at bring-up after MX_SPIx_Init().
     */
    void init(const Config& config);

    /**
     * @brief  Advance acquisition by one non-blocking step: kick the next channel's
     *         interrupt-driven read, or harvest / time-out the one in flight. Call
     *         every foreground iteration; it self-paces to POLL_INTERVAL_MS between
     *         rounds and never blocks.
     */
    void service(uint32_t now_ms);

    /** @brief The latest per-channel snapshot (state + status + temperatures). */
    [[nodiscard]] std::array<ThermocoupleInfo, THERMOCOUPLE_COUNT> info() const { return info_; }

private:
    enum class Phase : uint8_t { Idle, InFlight };

    void select(std::size_t ch, bool on);
    void writeRegister(std::size_t ch, uint8_t addr, uint8_t value);
    void startChannel(uint32_t now_ms);
    void harvestOrTimeout(uint32_t now_ms);
    void finishChannel(std::size_t ch, uint32_t now_ms);
    void parseFrame(std::size_t ch, const uint8_t* rx);

    Config                                           cfg_{};
    std::array<ThermocoupleInfo, THERMOCOUPLE_COUNT> info_{};
    Phase    phase_         = Phase::Idle;
    uint8_t  ch_            = 0;   // channel currently being read / next to read
    uint32_t xfer_start_ms_ = 0;   // tick the in-flight transfer was kicked (timeout base)
    uint32_t next_round_ms_ = 0;   // earliest tick the next full round may start
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::communication::ThermocoupleBank<Max31856Bank>);

} // namespace platform::acquisition::thermocouple::max31856
