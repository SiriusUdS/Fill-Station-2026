/**
  ******************************************************************************
  * @file    memory/backup_ram.cpp
  * @brief   STM32H7 backup-domain bring-up for the battery-backed Backup SRAM:
  *          write access (DBP), clock (BKPRAMEN) and the backup regulator
  *          (BREN), built directly on the STM32H7 HAL.
  ******************************************************************************
  */

#include "memory/backup_ram.hpp"

#include "stm32h7xx_hal.h"

namespace platform::memory::backup_ram {

std::optional<Error> init()
{
    /* Allow writes to the backup domain (RTC, backup registers, Backup SRAM):
       clears the write-protection bit PWR_CR1.DBP. Must come before touching
       the region. */
    HAL_PWR_EnableBkUpAccess();

    /* Clock the Backup SRAM (RCC_AHB4ENR.BKPRAMEN). Without this, any access to
       the 0x38800000 region faults. */
    __HAL_RCC_BKPRAM_CLK_ENABLE();

    /* Enable the backup regulator (PWR_CR2.BREN) and wait for it to stabilise
       (BRRDY). This is what keeps the Backup SRAM contents alive on VBAT once
       VDD is gone; without it the RTC still ticks but the SRAM is lost. */
    if (HAL_PWREx_EnableBkUpReg() != HAL_OK) {
        return Error::RegulatorTimeout;
    }

    return std::nullopt;
}

} // namespace platform::memory::backup_ram
