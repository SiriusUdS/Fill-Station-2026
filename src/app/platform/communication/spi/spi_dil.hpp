#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "stm32h7xx_hal.h"   // SPI_HandleTypeDef + GPIO types

/* ------------------------------------------------------------------------- *
 * Platform SPI DIL (Driver Interface Layer).
 *
 * SPI is a platform-internal transport here: its only consumer is the ADS131M08
 * ADC driver, which sits in the platform layer and reaches the bus through the
 * transfer()/receive() seam below. The logic layer never sees SPI - it depends
 * on the ADC interface (logic/communication/interfaces/adc.hpp) instead.
 *
 * This header is the shared spine of the platform SPI implementation. It owns
 * the bus identity/error types, the async transfer seam, the per-bus wiring type
 * (BusConfig), the driver-owned frame-buffer state (detail::BusState) and the
 * transport-agnostic mechanics (CS toggling, frame staging, completion
 * bookkeeping) that both transports reuse.
 *
 * The buses are split by transport into one translation unit each:
 *   - spi_dma.{hpp,cpp}  SPI4, HAL_SPI_TransmitReceive_DMA.
 *   - spi_irq.{hpp,cpp}  SPI6, HAL_SPI_TransmitReceive_IT (SPI6 is BDMA-only,
 *                        which the M7 path does not use).
 * spi_dil.cpp is the thin router: it dispatches init()/transfer()/receive() by
 * bus and owns the single HAL weak completion/error callbacks (which may have
 * only one definition program-wide), routing each by SPI instance.
 * ------------------------------------------------------------------------- */

namespace platform::communication::spi {

/** @brief The SPI buses exposed by the platform. */
enum class SpiBus {
    Spi4,  /**< DMA-driven bus (ADS131M08 ADC). */
    Spi6,  /**< Interrupt-driven bus (SPI6 is BDMA-only, so no DMA on the M7 path). */
};

/** @brief Errors the SPI interface can report. */
enum class SpiError {
    InternalError,  /**< Unspecified failure in the underlying HAL transfer. */
    Busy,           /**< A transfer is already in flight on this bus; retry later. */
};

/**
 * @brief  Start an asynchronous full-duplex frame on @p bus.
 *
 * The driver transmits exactly one fixed-length frame (the per-bus length set at
 * init). @p tx is copied into the front of the driver's TX buffer and the rest
 * of the frame is zero-filled, so callers may pass fewer bytes than the frame
 * length (e.g. a short command word) and may pass any buffer.
 *
 * @return std::nullopt once accepted, or a SpiError (Busy if a transfer is still
 *         in flight, InternalError on a HAL failure).
 */
[[nodiscard]] std::optional<SpiError> transfer(SpiBus bus, std::span<const uint8_t> tx);

/**
 * @brief  Take the bytes clocked back by the most recently completed transfer.
 *
 * @return A view over the driver-owned RX buffer (valid only until the next
 *         transfer() on the same bus), or std::nullopt if no transfer has
 *         completed since the last receive().
 */
[[nodiscard]] std::optional<std::span<const uint8_t>> receive(SpiBus bus);

/** @brief Largest frame, in bytes, any bus can transfer. */
inline constexpr uint16_t MAX_FRAME_BYTES = 64;

/**
 * @brief  Per-bus wiring captured at init.
 *
 * @c cs_ports / @c cs_pins are parallel arrays of length @c cs_num describing
 * the chip-select GPIOs; they must outlive the bus (point at storage with
 * static lifetime). The driver asserts/deasserts CS around each transfer only
 * when @c manage_cs is set. The transport (DMA vs IT) is fixed by which bus the
 * config is handed to, so it is not part of the config.
 */
struct BusConfig {
    SPI_HandleTypeDef* hspi   = nullptr;  /**< HAL SPI handle, e.g. &hspi4. */
    GPIO_TypeDef**     cs_ports = nullptr; /**< Chip-select GPIO ports (length cs_num). */
    uint16_t*          cs_pins  = nullptr; /**< Chip-select GPIO pins  (length cs_num). */
    uint16_t           cs_num   = 0;       /**< Number of chip selects. */
    uint16_t           frame_length = 0;   /**< Bytes per frame (1..MAX_FRAME_BYTES). */
    bool               manage_cs = false;  /**< Toggle CS around each transfer. */
};

/**
 * @brief  Bind @p bus to its HAL handle and chip-select GPIO, point it at the
 *         driver-owned buffers and arm it for the first transfer. Call once per
 *         bus at startup, after the CubeMX MX_SPIx_Init().
 * @param  bus     The bus to configure.
 * @param  config  Wiring for the bus (handle, CS GPIO, frame length, CS mode).
 */
void init(SpiBus bus, const BusConfig& config);

/* ------------------------------------------------------------------------- *
 * Transport-agnostic mechanics shared by spi_dma and spi_irq. Each transport
 * owns one BusState with static lifetime in its own translation unit; only the
 * HAL TransmitReceive variant differs between them, supplied to begin_transfer.
 * ------------------------------------------------------------------------- */
namespace detail {

/* done/new_data are written by the completion ISR and read by the main loop, so
   they are volatile; the pair is single-producer (ISR) / single-consumer
   (caller). The buffers are driver-owned so callers never hand DMA/IT a stack
   or caller buffer. */
struct BusState {
    SPI_HandleTypeDef* hspi     = nullptr;
    GPIO_TypeDef**     cs_ports = nullptr;
    uint16_t*          cs_pins  = nullptr;
    uint16_t           cs_num   = 0;
    uint16_t           frame_length = 0;
    bool               manage_cs = false;

    volatile bool done     = true;   // no transfer in flight; ready to start one
    volatile bool new_data = false;  // a completed frame is waiting in rx
    uint16_t      cs_counter = 0;
    uint32_t      last_timestamp = 0;

    std::array<uint8_t, MAX_FRAME_BYTES> tx{};
    std::array<uint8_t, MAX_FRAME_BYTES> rx{};
};

inline void assert_cs(const BusState& s)
{
    if (s.manage_cs && s.cs_num > 0) {
        HAL_GPIO_WritePin(s.cs_ports[0], s.cs_pins[0], GPIO_PIN_RESET);
    }
}

inline void deassert_cs(const BusState& s)
{
    if (s.manage_cs && s.cs_num > 0) {
        HAL_GPIO_WritePin(s.cs_ports[0], s.cs_pins[0], GPIO_PIN_SET);
    }
}

inline void apply_config(BusState& s, const BusConfig& config)
{
    s.hspi         = config.hspi;
    s.cs_ports     = config.cs_ports;
    s.cs_pins      = config.cs_pins;
    s.cs_num       = config.cs_num;
    s.frame_length = config.frame_length <= MAX_FRAME_BYTES ? config.frame_length : MAX_FRAME_BYTES;
    s.manage_cs    = config.manage_cs;

    s.done       = true;
    s.new_data   = false;
    s.cs_counter = 0;

    s.tx.fill(0);
    s.rx.fill(0);

    deassert_cs(s);
}

/* Stage the caller's bytes into the driver frame, assert CS and kick the
   peripheral. @p hal is the transport's HAL TransmitReceive variant (DMA or IT)
   - the only thing that differs between buses. */
template <typename HalCall>
inline std::optional<SpiError>
begin_transfer(BusState& s, std::span<const uint8_t> tx, HalCall hal)
{
    if (s.hspi == nullptr || s.frame_length == 0) {
        return SpiError::InternalError;  // bus not initialised
    }
    if (!s.done) {
        return SpiError::Busy;  // a transfer is still in flight
    }

    // Lay the caller's bytes at the front of the frame and zero-fill the rest.
    const std::size_t copy = tx.size() < s.frame_length ? tx.size() : s.frame_length;
    std::memcpy(s.tx.data(), tx.data(), copy);
    std::memset(s.tx.data() + copy, 0, s.frame_length - copy);

    s.done = false;
    assert_cs(s);

    if (hal(s.hspi, s.tx.data(), s.rx.data(), s.frame_length) != HAL_OK) {
        deassert_cs(s);
        s.done = true;  // re-arm; the frame never started
        return SpiError::InternalError;
    }
    return std::nullopt;
}

inline std::optional<std::span<const uint8_t>> take_received(BusState& s)
{
    if (!s.new_data) {
        return std::nullopt;
    }
    s.new_data = false;
    return std::span<const uint8_t>(s.rx.data(), s.frame_length);
}

/* Completion-ISR bookkeeping: deassert CS, re-arm and publish the frame. */
inline void on_complete(BusState& s)
{
    deassert_cs(s);
    s.done     = true;
    s.new_data = true;
    if (s.cs_num > 0) {
        s.cs_counter = (s.cs_counter + 1) % s.cs_num;
    }
    s.last_timestamp = HAL_GetTick();
}

inline void on_error(BusState& s, SPI_HandleTypeDef* hspi)
{
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    deassert_cs(s);
    s.done = true;  // re-arm so the next transfer can proceed
}

} // namespace detail

} // namespace platform::communication::spi
