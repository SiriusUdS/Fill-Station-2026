#pragma once

#include <optional>
#include <span>

#include "stm32h7xx_hal.h"                // SPI_HandleTypeDef
#include "communication/spi/spi_dil.hpp"  // BusConfig, SpiError, detail::BusState

/* ------------------------------------------------------------------------- *
 * DMA SPI transport: SPI4, driven by HAL_SPI_TransmitReceive_DMA.
 *
 * Self-contained - owns its BusState in the .cpp. The spi_dil router dispatches
 * the bus's init/transfer/receive here and routes the HAL completion/error
 * callbacks through on_complete/on_error.
 *
 * Each transfer is one fixed-length frame. Beyond the poll path (receive()), the
 * transport can invoke a per-frame callback from the completion ISR, so an
 * event-driven consumer (e.g. the DRDY-paced ADC: each DRDY edge kicks one
 * transfer, and this callback parses the frame) needs no polling.
 * ------------------------------------------------------------------------- */

namespace platform::communication::spi::dma {

/** @brief Bind SPI4's wiring and arm it. See platform::communication::spi::init. */
void init(const BusConfig& config);

/** @brief Start one DMA frame. See platform::communication::spi::transfer. */
std::optional<SpiError> transfer(std::span<const uint8_t> tx);

/** @brief Take the last completed frame. See platform::communication::spi::receive. */
std::optional<std::span<const uint8_t>> receive();

/* ----------------------- per-frame completion hook ----------------------- */

/**
 * @brief  Callback invoked from the completion ISR after each frame completes,
 *         with the bytes clocked back.
 *
 * @p frame views the driver-owned RX buffer and is valid only for the duration
 * of the call (the next transfer overwrites it). Runs in interrupt context -
 * keep it short.
 */
using FrameReadyCallback = void (*)(std::span<const uint8_t> frame);

/**
 * @brief  Register (or clear, with nullptr) the per-frame completion callback.
 *
 * Once set, every completed frame invokes @p cb from the completion ISR. The
 * DRDY-paced ADC uses this to parse each frame without polling receive().
 */
void set_frame_callback(FrameReadyCallback cb);

/* --------------------------- ISR routing hooks --------------------------- */

/** @brief Completion-ISR hook. Returns true iff @p hspi is this transport's bus. */
bool on_complete(SPI_HandleTypeDef* hspi);

/** @brief Error-ISR hook. Returns true iff @p hspi is this transport's bus. */
bool on_error(SPI_HandleTypeDef* hspi);

} // namespace platform::communication::spi::dma
