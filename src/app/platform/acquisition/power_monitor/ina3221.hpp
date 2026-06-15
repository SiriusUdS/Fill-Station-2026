#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32h7xx_hal.h"   // I2C_HandleTypeDef, IRQn_Type (Config)

#include "communication/interfaces/power_monitor.hpp"   // logic::communication::PowerMonitor
#include "communication/protocol/peripherals/power_monitor/power_monitor_info.hpp"  // PowerMonitorInfo

/* ------------------------------------------------------------------------- *
 * TI INA3221 3-channel current/bus-voltage monitor (one device on I2C4).
 *
 * One object owns the device + its I2C bus. Acquisition is NON-BLOCKING so it is
 * safe in the main loop: service() drives an interrupt-driven round-robin over the 6
 * measurement registers (shunt + bus for each of the 3 channels) one step at a time,
 * harvesting each completed transfer from the foreground and timing a stuck one out —
 * it never busy-waits on the bus. info() returns the cached snapshot.
 *
 * I2C4 is in the D3 domain (BDMA-only, which the M7 path does not use — same as
 * SPI6), so the per-loop reads go through the interrupt transport
 * (HAL_I2C_Mem_Read_IT). The board supplies the I2C handle, the device address and
 * the EV/ER IRQ lines through Config, so the driver carries no board-specific values.
 * The board must provide the matching I2C event/error vector handlers (which call
 * HAL_I2C_EV_IRQHandler / HAL_I2C_ER_IRQHandler); this driver only arms the NVIC
 * lines in init() and owns the HAL completion/error callbacks.
 *
 * Measurements are stored as the device's raw register codes (sign-extended); the
 * ground station applies the LSB scale and the shunt resistance — no rounding or
 * current math on the board (same convention as AdcInfo / ThermocoupleInfo). It
 * models the logic::communication::PowerMonitor seam.
 * ------------------------------------------------------------------------- */

namespace platform::acquisition::power_monitor::ina3221 {

/* ---- Register addresses ---- */
constexpr uint8_t REG_CONFIG = 0x00;  // configuration: channel enables, averaging, conv times, mode
constexpr uint8_t REG_CH1_SHUNT = 0x01;  // first measurement register; CH1..CH3 shunt/bus are 0x01..0x06
constexpr uint8_t REG_MFG_ID = 0xFE;  // manufacturer id (0x5449 'TI')
constexpr uint8_t REG_DIE_ID = 0xFF;  // die id (0x3220)

/* ---- Config register value ---- */
// All 3 channels enabled, 16-sample averaging, 1.1 ms bus + shunt conversion, continuous
// shunt-and-bus mode. AVG=16 gives a ~9.5 Hz full-set update (6 × 1.1 ms × 16 ≈ 106 ms) — low
// noise to match the ~10 Hz ExtendedSystemState cadence we read at. Written at init() (also a
// presence check). (Reset default is 0x7127 = AVG 1.)
constexpr uint16_t CONFIG_DEFAULT = 0x7527;

/* ---- Default 7-bit I2C address (A0 = GND); other strappings via Config::address ---- */
constexpr uint8_t DEFAULT_ADDRESS = 0x40;

/* ---- Cadence + timeout ---- */
constexpr uint32_t POLL_INTERVAL_MS  = 100;  // gap between full 3-channel rounds (~10 Hz)
constexpr uint32_t SAMPLE_TIMEOUT_MS = 5;    // abandon a transfer that has not completed in this long

/** @brief Board wiring for the INA3221 on one I2C bus. */
struct Config {
    I2C_HandleTypeDef* hi2c    = nullptr;            /**< I2C handle, e.g. &hi2c4. */
    uint8_t            address = DEFAULT_ADDRESS;    /**< 7-bit device address (A0 strap). */
    IRQn_Type          ev_irqn = I2C4_EV_IRQn;       /**< I2C event NVIC line; armed by init(). */
    IRQn_Type          er_irqn = I2C4_ER_IRQn;       /**< I2C error NVIC line; armed by init(). */
    bool               stubbed = false;              /**< If set, the driver NEVER touches the bus:
                                                          init() skips the config write + NVIC, service()
                                                          is a no-op, and info() stays Unknown / invalid.
                                                          For boards where I2C is unusable in hardware
                                                          (e.g. the ECU's swapped SDA/SCL); clear it (one
                                                          line) to enable once the PCB is fixed. */
};

class Ina3221 {
public:
    /** @brief Channels the device monitors (3-channel part). */
    static constexpr std::size_t channel_count = POWER_MONITOR_CHANNEL_COUNT;

    Ina3221() = default;

    /**
     * @brief  Bind the bus + address, write the configuration register (a short bounded
     *         blocking transfer that also checks the device is present), and arm the I2C
     *         event/error interrupts for the per-loop reads. Call once at bring-up after
     *         MX_I2Cx_Init(). On a missing/NACKing device the state is left Faulted.
     */
    void init(const Config& config);

    /**
     * @brief  Advance acquisition by one non-blocking step: kick the next measurement
     *         register's interrupt-driven read, or harvest / time-out the one in flight.
     *         Call every foreground iteration; it self-paces to POLL_INTERVAL_MS between
     *         rounds and never blocks.
     */
    void service(uint32_t now_ms);

    /** @brief The latest snapshot (state + status + per-channel shunt/bus codes). */
    [[nodiscard]] PowerMonitorInfo info() const { return info_; }

    /* ---- ISR entry points (platform dispatch; not the contract) ---------- */

    /** @brief I2C transfer completed for @p hi2c (HAL completion ISR). */
    void handleComplete(const I2C_HandleTypeDef* hi2c) { if (hi2c == cfg_.hi2c) done_ = true; }

    /** @brief I2C transfer errored for @p hi2c (HAL error ISR). */
    void handleError(const I2C_HandleTypeDef* hi2c) { if (hi2c == cfg_.hi2c) { error_ = true; done_ = true; } }

private:
    enum class Phase : uint8_t { Idle, InFlight };

    static constexpr std::size_t REG_COUNT = 2 * POWER_MONITOR_CHANNEL_COUNT;  // shunt+bus per channel

    void startRead(uint32_t now_ms);
    void harvestOrTimeout(uint32_t now_ms);
    void finishRegister(uint32_t now_ms, bool ok);
    void parseRegister(std::size_t reg_idx, const uint8_t* rx);

    Config           cfg_{};
    PowerMonitorInfo info_{};
    uint16_t         addr8_         = 0;   // HAL 8-bit address (7-bit << 1)
    Phase            phase_         = Phase::Idle;
    uint8_t          reg_idx_       = 0;   // measurement register being read / next to read (0..REG_COUNT-1)
    bool             round_ok_      = true;  // no fault seen yet this round (decides data_valid at round end)
    uint32_t         xfer_start_ms_ = 0;   // tick the in-flight transfer was kicked (timeout base)
    uint32_t         next_round_ms_ = 0;   // earliest tick the next full round may start
    uint8_t          rx_[2]         = {};  // one 16-bit register, MSB first

    volatile bool    done_  = false;  // set by the completion/error ISR
    volatile bool    error_ = false;  // set by the error ISR (transfer NACKed / bus error)
};

// The driver is the logic seam: enforce conformance here so a contract drift is
// caught in the platform layer rather than at a logic call.
static_assert(logic::communication::PowerMonitor<Ina3221>);

} // namespace platform::acquisition::power_monitor::ina3221
