/**
 ******************************************************************************
 * @file    communication/spi/spi_dil.cpp
 * @brief   Thin SPI router. Dispatches init() and the transfer/receive seam to
 *          the per-transport translation units by bus, and owns the single HAL
 *          completion/error/half-complete weak callbacks (only one definition
 *          is allowed program-wide), routing each by SPI instance.
 ******************************************************************************
 */

#include "communication/spi/spi_dil.hpp"
#include "communication/spi/spi_dma.hpp"
#include "communication/spi/spi_irq.hpp"

#include <optional>
#include <span>

namespace pdma = platform::communication::spi::dma;
namespace pirq = platform::communication::spi::irq;

namespace platform::communication::spi {

/* -------------------------------------------------------------------------- */
/* Bind a bus to its transport, and the transfer/receive seam                 */
/* -------------------------------------------------------------------------- */

void init(SpiBus bus, const BusConfig& config)
{
    if (bus == SpiBus::Spi4) {
        pdma::init(config);
    } else {
        pirq::init(config);
    }
}

std::optional<SpiError> transfer(SpiBus bus, std::span<const uint8_t> tx)
{
    return bus == SpiBus::Spi4 ? pdma::transfer(tx) : pirq::transfer(tx);
}

std::optional<std::span<const uint8_t>> receive(SpiBus bus)
{
    return bus == SpiBus::Spi4 ? pdma::receive() : pirq::receive();
}

} // namespace platform::communication::spi

/* -------------------------------------------------------------------------- */
/* Interrupt callbacks (override the HAL weak symbols, so keep C linkage).     */
/* Route to whichever transport owns the SPI instance.                        */
/* -------------------------------------------------------------------------- */

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (pdma::on_complete(hspi)) {
        return;
    }
    pirq::on_complete(hspi);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    if (pdma::on_error(hspi)) {
        return;
    }
    pirq::on_error(hspi);
}
