/**
 ******************************************************************************
 * @file    communication/spi/spi_dma.cpp
 * @brief   DMA SPI transport (SPI4). Owns the driver-side frame buffers for the
 *          DMA bus and frames transfers through HAL_SPI_TransmitReceive_DMA.
 *          Each completed frame can invoke a registered callback from the
 *          completion ISR, so an event-driven consumer (the DRDY-paced ADC) is
 *          driven entirely off interrupts. The shared single-shot mechanics live
 *          in spi_dil.hpp's detail namespace.
 *
 *          Buffer placement: SPI4's DMA1 streams (DMA1_Stream0/1) cannot reach
 *          the default .bss (DTCM, 0x20000000), so the driver buffers live in
 *          D1 AXI-SRAM via the linker's .axisram section. D-cache is currently
 *          disabled; if it is ever enabled these buffers need a non-cacheable
 *          MPU region (or invalidate-on-RX), as DMA bypasses the cache.
 ******************************************************************************
 */

#include "communication/spi/spi_dma.hpp"

#include <span>

namespace platform::communication::spi::dma {

namespace {

// SPI4's DMA1 streams cannot reach DTCM, so the buffers the DMA touches are
// pinned in D1 AXI-SRAM. BusState has a runtime constructor (default member
// initialisers), so it is initialised at boot regardless of .axisram's NOLOAD.
__attribute__((section(".axisram"))) detail::BusState s_bus;

FrameReadyCallback s_frame_cb = nullptr;

bool owns(const SPI_HandleTypeDef* hspi)
{
    return s_bus.hspi != nullptr && hspi->Instance == s_bus.hspi->Instance;
}

} // namespace

void init(const BusConfig& config)
{
    detail::apply_config(s_bus, config);
}

std::optional<SpiError> transfer(std::span<const uint8_t> tx)
{
    return detail::begin_transfer(s_bus, tx, HAL_SPI_TransmitReceive_DMA);
}

std::optional<std::span<const uint8_t>> receive()
{
    return detail::take_received(s_bus);
}

void set_frame_callback(FrameReadyCallback cb)
{
    s_frame_cb = cb;
}

bool on_complete(SPI_HandleTypeDef* hspi)
{
    if (!owns(hspi)) {
        return false;
    }
    detail::on_complete(s_bus);
    if (s_frame_cb != nullptr) {
        s_frame_cb(std::span<const uint8_t>(s_bus.rx.data(), s_bus.frame_length));
    }
    return true;
}

bool on_error(SPI_HandleTypeDef* hspi)
{
    if (!owns(hspi)) {
        return false;
    }
    detail::on_error(s_bus, hspi);
    return true;
}

} // namespace platform::communication::spi::dma
