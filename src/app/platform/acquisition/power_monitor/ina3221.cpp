/**
 ******************************************************************************
 * @file    acquisition/power_monitor/ina3221.cpp
 * @brief   TI INA3221 3-channel power monitor: one-time blocking register config,
 *          then a non-blocking interrupt-driven round-robin that reads each channel's
 *          shunt + bus voltage registers over I2C. I2C4 is BDMA-only on the M7 (same
 *          as SPI6), so the per-loop reads use the HAL interrupt transport rather than
 *          DMA. One live instance reachable from the HAL I2C completion/error
 *          callbacks (fixed-signature, carry no `this`) through s_instance.
 ******************************************************************************
 */

#include "acquisition/power_monitor/ina3221.hpp"

namespace platform::acquisition::power_monitor::ina3221 {

namespace {

constexpr uint32_t CONFIG_WRITE_TIMEOUT_MS = 5;  // bounded blocking write, one-time at init
constexpr uint32_t I2C_IRQ_PRIO            = 5;  // below the SPI/DMA completion ISRs (priority 0)

// The single live device, so the fixed-signature HAL I2C callbacks below can reach the
// instance that owns the in-flight transfer (set in init(), the device's lifetime).
Ina3221* s_instance = nullptr;

// The 6 measurement registers, in read order: CH1 shunt, CH1 bus, CH2 shunt, ... They are
// contiguous from REG_CH1_SHUNT (0x01..0x06); the INA3221 does not auto-increment its
// pointer across a read, so each is its own register-pointer-write + 2-byte read.
constexpr uint8_t REG_MEAS[2 * POWER_MONITOR_CHANNEL_COUNT] = {
    REG_CH1_SHUNT + 0, REG_CH1_SHUNT + 1, REG_CH1_SHUNT + 2,
    REG_CH1_SHUNT + 3, REG_CH1_SHUNT + 4, REG_CH1_SHUNT + 5,
};

} // namespace

void Ina3221::init(const Config& config)
{
    cfg_     = config;
    addr8_   = static_cast<uint16_t>(config.address << 1);  // HAL wants the 8-bit address
    info_    = PowerMonitorInfo{};                          // state Unknown, no channels yet
    reg_idx_ = 0;
    phase_   = Phase::Idle;
    round_ok_ = true;
    next_round_ms_ = 0;
    done_    = false;
    error_   = false;

    s_instance = this;

    // Stubbed (I2C unusable in hardware on this board): do not touch the bus at all. Leave the
    // record Unknown / data_valid 0 so the GS sees "no power data" rather than zeros that look
    // real. service() is a no-op too. Clear Config::stubbed to enable once the PCB is fixed.
    if (cfg_.stubbed) {
        return;
    }

    // Write CONFIG (continuous shunt+bus on all 3 channels). Short bounded blocking — one-time,
    // before the loop runs — and doubles as a presence check: a missing device NACKs, so the
    // write fails and we start Faulted instead of streaming zeros as if they were real.
    uint8_t cfg_bytes[2] = {
        static_cast<uint8_t>(CONFIG_DEFAULT >> 8),
        static_cast<uint8_t>(CONFIG_DEFAULT & 0xFF),
    };
    if (HAL_I2C_Mem_Write(cfg_.hi2c, addr8_, REG_CONFIG, I2C_MEMADD_SIZE_8BIT,
                          cfg_bytes, sizeof(cfg_bytes), CONFIG_WRITE_TIMEOUT_MS) != HAL_OK) {
        info_.state             = PowerMonitorState::Faulted;
        info_.status.read_error = 1u;
    }

    // Arm the I2C event + error interrupts for the per-round reads (the board provides the
    // matching vector handlers, which drive the HAL state machine to the completion callback).
    HAL_NVIC_SetPriority(cfg_.ev_irqn, I2C_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(cfg_.ev_irqn);
    HAL_NVIC_SetPriority(cfg_.er_irqn, I2C_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(cfg_.er_irqn);
}

void Ina3221::startRead(uint32_t now_ms)
{
    done_  = false;
    error_ = false;
    if (HAL_I2C_Mem_Read_IT(cfg_.hi2c, addr8_, REG_MEAS[reg_idx_], I2C_MEMADD_SIZE_8BIT,
                            rx_, sizeof(rx_)) == HAL_OK) {
        xfer_start_ms_ = now_ms;
        phase_         = Phase::InFlight;
    }
    // else the bus would not accept the transfer (busy/error): stay Idle and retry next call.
}

void Ina3221::harvestOrTimeout(uint32_t now_ms)
{
    if (done_) {
        if (error_) {
            finishRegister(now_ms, /*ok=*/false);   // NACK / bus error
        } else {
            parseRegister(reg_idx_, rx_);
            finishRegister(now_ms, /*ok=*/true);
        }
        return;
    }
    if ((now_ms - xfer_start_ms_) >= SAMPLE_TIMEOUT_MS) {
        finishRegister(now_ms, /*ok=*/false);        // never completed; abandon it, move on
    }
}

void Ina3221::parseRegister(std::size_t reg_idx, const uint8_t* rx)
{
    // INA3221 registers are big-endian; the shunt/bus value is the 13-bit two's-complement
    // field in bits 15:3 (bits 2:0 reserved), so read as int16 then arithmetic-shift right 3.
    const int16_t raw  = static_cast<int16_t>((rx[0] << 8) | rx[1]);
    const int16_t code = static_cast<int16_t>(static_cast<int32_t>(raw) >> 3);

    const std::size_t ch = reg_idx / 2;
    if ((reg_idx & 1u) == 0u) {
        info_.channels[ch].shunt_code = code;   // even register = shunt voltage
    } else {
        info_.channels[ch].bus_code = code;     // odd register  = bus voltage
    }
}

void Ina3221::finishRegister(uint32_t now_ms, bool ok)
{
    phase_ = Phase::Idle;
    if (!ok) {
        round_ok_ = false;
    }
    if (++reg_idx_ >= REG_COUNT) {
        reg_idx_ = 0;
        // Round complete: publish state from whether every register read cleanly.
        info_.status.read_error = round_ok_ ? 0u : 1u;
        info_.status.data_valid = round_ok_ ? 1u : 0u;
        info_.state             = round_ok_ ? PowerMonitorState::Active : PowerMonitorState::Faulted;
        round_ok_      = true;                       // reset for the next round
        next_round_ms_ = now_ms + POLL_INTERVAL_MS;  // pace the next full round
    }
}

void Ina3221::service(uint32_t now_ms)
{
    if (cfg_.stubbed) {
        return;   // I2C disabled in hardware on this board; never drive the bus.
    }
    if (phase_ == Phase::InFlight) {
        harvestOrTimeout(now_ms);
        return;
    }
    // Idle: hold off at the start of a round until the poll interval has elapsed; the
    // registers within a round then run back-to-back.
    if (reg_idx_ == 0 && now_ms < next_round_ms_) {
        return;
    }
    startRead(now_ms);
}

// HAL I2C completion/error callbacks (override the weak HAL symbols, so keep C linkage;
// only one definition is allowed program-wide). Route to the live instance, which filters
// by its own handle.
extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (s_instance != nullptr) {
        s_instance->handleComplete(hi2c);
    }
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    if (s_instance != nullptr) {
        s_instance->handleError(hi2c);
    }
}

} // namespace platform::acquisition::power_monitor::ina3221
