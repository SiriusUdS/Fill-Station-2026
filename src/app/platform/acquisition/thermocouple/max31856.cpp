/**
 ******************************************************************************
 * @file    acquisition/thermocouple/max31856.cpp
 * @brief   MAX31856 thermocouple bank: one-time blocking register config, then a
 *          non-blocking interrupt-driven round-robin that reads each channel's
 *          linearized thermocouple + cold-junction temperatures and fault status
 *          over the shared SPI6 IT transport. Owns its 4 chip-selects (one
 *          asserted at a time) so the loop never blocks on the bus.
 ******************************************************************************
 */

#include "acquisition/thermocouple/max31856.hpp"

#include <span>

#include "communication/spi/spi_dil.hpp"   // platform SPI seam (SpiBus::Spi6, transfer/receive/init)

namespace spi = platform::communication::spi;

namespace platform::acquisition::thermocouple::max31856 {

namespace {

constexpr uint32_t CONFIG_WRITE_TIMEOUT_MS = 5;  // bounded blocking write, one-time at init

// Sign-extend the low @p bits of @p v (two's complement) to a full int32.
int32_t signExtend(uint32_t v, unsigned bits)
{
    const uint32_t mask = (1u << bits) - 1u;
    const uint32_t sign = 1u << (bits - 1);
    v &= mask;
    return static_cast<int32_t>((v ^ sign) - sign);
}

} // namespace

void Max31856Bank::select(std::size_t ch, bool on)
{
    // Chip-select is active-low: on (selected) drives the line low.
    HAL_GPIO_WritePin(cfg_.cs_ports[ch], cfg_.cs_pins[ch], on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Max31856Bank::writeRegister(std::size_t ch, uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { static_cast<uint8_t>(addr | WRITE_FLAG), value };
    select(ch, true);
    (void)HAL_SPI_Transmit(cfg_.hspi, tx, sizeof(tx), CONFIG_WRITE_TIMEOUT_MS);
    select(ch, false);
}

bool Max31856Bank::verifyConfig(std::size_t ch)
{
    // Read CR0 then CR1 back in one burst (address byte has bit7=0 for a read; the device
    // auto-increments). rx[0] is clocked out during the address byte; rx[1]=CR0, rx[2]=CR1.
    uint8_t tx[3] = { REG_CR0, 0x00, 0x00 };
    uint8_t rx[3] = {};
    select(ch, true);
    const HAL_StatusTypeDef ok =
        HAL_SPI_TransmitReceive(cfg_.hspi, tx, rx, sizeof(tx), CONFIG_WRITE_TIMEOUT_MS);
    select(ch, false);
    if (ok != HAL_OK) {
        return false;
    }
    // Match against exactly what init() wrote. A non-responding device returns 0x00 here.
    const uint8_t expect_cr0 = static_cast<uint8_t>(CR0_CMODE_AUTO | CR0_OCFAULT_1);
    const uint8_t expect_cr1 = static_cast<uint8_t>(cfg_.tc_type & 0x0F);
    return rx[1] == expect_cr0 && rx[2] == expect_cr1;
}

void Max31856Bank::init(const Config& config)
{
    cfg_ = config;

    for (std::size_t ch = 0; ch < THERMOCOUPLE_COUNT; ++ch) {
        info_[ch]       = {};
        info_[ch].state = ThermocoupleState::Unknown;
        select(ch, false);   // deassert every CS before touching the bus
    }

    // One-time configuration (short bounded blocking writes, before the loop runs):
    // mask the FAULT pin (we poll SR), enable automatic conversion + open-circuit
    // detection, and select the thermocouple type.
    for (std::size_t ch = 0; ch < THERMOCOUPLE_COUNT; ++ch) {
        writeRegister(ch, REG_MASK, 0xFF);
        writeRegister(ch, REG_CR0, static_cast<uint8_t>(CR0_CMODE_AUTO | CR0_OCFAULT_1));
        writeRegister(ch, REG_CR1, static_cast<uint8_t>(cfg_.tc_type & 0x0F));
    }

    // SPI sanity check: read the config back per channel. If a device does not echo what we
    // just wrote, the SPI link to it is dead (e.g. wrong clock phase, unpowered, MISO not
    // wired) — record that so parseFrame forces the channel Faulted rather than reporting the
    // all-zero reads as a valid 0 C. comms_ok rides ThermocoupleInfo into the extended state.
    for (std::size_t ch = 0; ch < THERMOCOUPLE_COUNT; ++ch) {
        config_ok_[ch]            = verifyConfig(ch);
        info_[ch].status.comms_ok = config_ok_[ch] ? 1u : 0u;
        if (!config_ok_[ch]) {
            info_[ch].state             = ThermocoupleState::Faulted;
            info_[ch].status.data_valid = 0u;
        }
    }

    // Arm the SPI6 interrupt transport for the per-loop reads: a fixed 7-byte frame,
    // CS managed by us (the bank multiplexes 4 chip-selects, which the single-CS
    // seam cannot), so manage_cs stays off.
    spi::BusConfig bus = {};
    bus.hspi         = cfg_.hspi;
    bus.frame_length = READ_FRAME_BYTES;
    bus.manage_cs    = false;
    spi::init(spi::SpiBus::Spi6, bus);
}

void Max31856Bank::startChannel(uint32_t now_ms)
{
    // Assert this channel's CS, then kick a non-blocking burst read from CJTH. If the
    // bus will not accept the frame (busy/error), deassert and retry on a later call.
    select(ch_, true);
    const uint8_t addr = REG_CJTH;
    if (!spi::transfer(spi::SpiBus::Spi6, std::span<const uint8_t>(&addr, 1))) {
        xfer_start_ms_ = now_ms;
        phase_         = Phase::InFlight;
    } else {
        select(ch_, false);
    }
}

void Max31856Bank::harvestOrTimeout(uint32_t now_ms)
{
    if (auto rx = spi::receive(spi::SpiBus::Spi6)) {
        select(ch_, false);
        parseFrame(ch_, rx->data());
        finishChannel(ch_, now_ms);
        return;
    }
    if ((now_ms - xfer_start_ms_) >= SAMPLE_TIMEOUT_MS) {
        // The frame never completed: abandon it, flag the channel faulted, move on.
        select(ch_, false);
        info_[ch_].state             = ThermocoupleState::Faulted;
        info_[ch_].status.data_valid = 0u;
        finishChannel(ch_, now_ms);
    }
}

void Max31856Bank::finishChannel(std::size_t ch, uint32_t now_ms)
{
    phase_ = Phase::Idle;
    if (++ch >= THERMOCOUPLE_COUNT) {
        ch_            = 0;
        next_round_ms_ = now_ms + POLL_INTERVAL_MS;   // pace the next full round
    } else {
        ch_ = static_cast<uint8_t>(ch);               // next channel, same round, immediately
    }
}

void Max31856Bank::parseFrame(std::size_t ch, const uint8_t* rx)
{
    // rx[0] is clocked out during the address byte; rx[1..6] are CJTH, CJTL, LTCBH,
    // LTCBM, LTCBL, SR (the contiguous burst from REG_CJTH).
    ThermocoupleInfo& info = info_[ch];

    const uint16_t cj_raw = static_cast<uint16_t>((rx[1] << 8) | rx[2]);
    const uint32_t tc_raw = (static_cast<uint32_t>(rx[3]) << 16)
                          | (static_cast<uint32_t>(rx[4]) << 8)
                          |  static_cast<uint32_t>(rx[5]);
    const uint8_t  sr     = rx[6];

    // Cold junction: 14-bit two's complement in bits 15:2 (LSB 2^-6 C). Thermocouple:
    // 19-bit in bits 23:5 (LSB 2^-7 C). Sign-extend the full register, then drop the
    // unused low bits to land on the documented LSB.
    info.cold_junction_code = signExtend(cj_raw, 16) >> 2;
    info.thermocouple_code  = signExtend(tc_raw, 24) >> 5;

    info.status.open_circuit = (sr & 0x01u) ? 1u : 0u;  // SR.OPEN
    info.status.over_under_v = (sr & 0x02u) ? 1u : 0u;  // SR.OVUV
    info.status.tc_out_range = (sr & 0x40u) ? 1u : 0u;  // SR.TCRANGE
    info.status.cj_out_range = (sr & 0x80u) ? 1u : 0u;  // SR.CJRANGE
    info.status.comms_ok     = config_ok_[ch] ? 1u : 0u;

    // A dead SPI link returns an all-zero frame, which has sr == 0 and would otherwise look
    // like a clean 0 C reading. Gate on the init-time readback so a non-responding device is
    // always Faulted, never a cheerful 0.
    const bool faulted       = (sr != 0u) || !config_ok_[ch];
    info.status.data_valid   = faulted ? 0u : 1u;
    info.state               = faulted ? ThermocoupleState::Faulted : ThermocoupleState::Active;
}

void Max31856Bank::service(uint32_t now_ms)
{
    if (phase_ == Phase::InFlight) {
        harvestOrTimeout(now_ms);
        return;
    }
    // Idle: hold off at the start of a round until the poll interval has elapsed; the
    // channels within a round then run back-to-back.
    if (ch_ == 0 && now_ms < next_round_ms_) {
        return;
    }
    startChannel(now_ms);
}

} // namespace platform::acquisition::thermocouple::max31856
