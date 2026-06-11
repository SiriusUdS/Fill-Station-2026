#pragma once

#include <optional>
#include <span>

#include "stm32h7xx_hal.h"                // SPI_HandleTypeDef
#include "communication/spi/spi_dil.hpp"  // BusConfig, SpiError, detail::BusState

/* ------------------------------------------------------------------------- *
 * Interrupt SPI transport: SPI6, driven by HAL_SPI_TransmitReceive_IT.
 *
 * SPI6 lives in a domain only BDMA can reach, which the M7 path does not use,
 * so this bus runs interrupt-driven rather than on DMA. Self-contained - owns
 * its BusState in the .cpp; the spi_dil router dispatches the bus's
 * init/transfer/receive here and routes the HAL callbacks through
 * on_complete/on_error.
 * ------------------------------------------------------------------------- */

namespace platform::communication::spi::irq {

/** @brief Bind SPI6's wiring and arm it. See platform::communication::spi::init. */
void init(const BusConfig& config);

/** @brief Start one interrupt-driven frame. See platform::communication::spi::transfer. */
std::optional<SpiError> transfer(std::span<const uint8_t> tx);

/** @brief Take the last completed frame. See platform::communication::spi::receive. */
std::optional<std::span<const uint8_t>> receive();

/** @brief Completion-ISR hook. Returns true iff @p hspi is this transport's bus. */
bool on_complete(SPI_HandleTypeDef* hspi);

/** @brief Error-ISR hook. Returns true iff @p hspi is this transport's bus. */
bool on_error(SPI_HandleTypeDef* hspi);

} // namespace platform::communication::spi::irq
