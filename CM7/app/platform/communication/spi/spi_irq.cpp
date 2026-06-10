/**
 ******************************************************************************
 * @file    communication/spi/spi_irq.cpp
 * @brief   Interrupt SPI transport (SPI6). Owns the driver-side frame buffers
 *          for the interrupt-driven bus and frames transfers through
 *          HAL_SPI_TransmitReceive_IT. The shared mechanics (CS toggling, frame
 *          staging, completion bookkeeping) live in spi_dil.hpp's detail
 *          namespace.
 ******************************************************************************
 */

#include "communication/spi/spi_irq.hpp"

namespace {

// This translation unit owns the interrupt bus's state for its whole lifetime.
platform::communication::spi::detail::BusState s_bus;

} // namespace

namespace platform::communication::spi::irq {

void init(const BusConfig& config)
{
    detail::apply_config(s_bus, config);
}

std::optional<SpiError> transfer(std::span<const uint8_t> tx)
{
    return detail::begin_transfer(s_bus, tx, HAL_SPI_TransmitReceive_IT);
}

std::optional<std::span<const uint8_t>> receive()
{
    return detail::take_received(s_bus);
}

bool on_complete(SPI_HandleTypeDef* hspi)
{
    if (s_bus.hspi == nullptr || hspi->Instance != s_bus.hspi->Instance) {
        return false;
    }
    detail::on_complete(s_bus);
    return true;
}

bool on_error(SPI_HandleTypeDef* hspi)
{
    if (s_bus.hspi == nullptr || hspi->Instance != s_bus.hspi->Instance) {
        return false;
    }
    detail::on_error(s_bus, hspi);
    return true;
}

} // namespace platform::communication::spi::irq
