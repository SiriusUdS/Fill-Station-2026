#pragma once

#include <optional>

/* ------------------------------------------------------------------------- *
 * Platform Backup SRAM manager.
 *
 * Brings up the STM32H7 backup domain so the battery-backed Backup SRAM — the
 * `.backup_sram` linker section that holds logic::control::persistent_state —
 * is clocked, writable, and retained on VBAT. Pure platform code: it owns the
 * PWR/RCC register writes (via the HAL) and exposes only a HAL-free init() to
 * the rest of the firmware. Call init() once at boot, before any code touches
 * persistent state.
 * ------------------------------------------------------------------------- */

namespace platform::memory::backup_ram {

/** @brief Failure modes of init(). */
enum class Error {
    RegulatorTimeout,  /**< Backup regulator (BREN) never reported ready (BRRDY). */
};

/**
 * @brief  Bring up the backup domain for Backup SRAM use and VBAT retention.
 *
 * Performs, in order: enable backup-domain write access (PWR_CR1.DBP), enable
 * the Backup-SRAM clock (RCC_AHB4ENR.BKPRAMEN), and enable the backup regulator
 * (PWR_CR2.BREN), waiting for it to stabilise (BRRDY). Until BKPRAMEN is set
 * any access to the 0x38800000 region faults; until BREN is set the contents
 * are lost on VBAT-only power loss (the RTC keeps ticking, the SRAM does not).
 *
 * @return std::nullopt on success, or an Error describing the failure. A
 *         RegulatorTimeout is not fatal to *access* (the SRAM is already
 *         clocked and writable); it only means VBAT retention is unconfirmed.
 */
[[nodiscard]] std::optional<Error> init();

} // namespace platform::memory::backup_ram
