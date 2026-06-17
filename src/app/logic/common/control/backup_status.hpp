#pragma once

#include <cstdint>

/* ------------------------------------------------------------------------- *
 * Backup-domain retention health — runtime diagnostic state.
 *
 * The platform probes the backup domain at boot (backup_ram::init: DBP + BKPRAMEN
 * + the backup regulator BREN/BRRDY) and records the outcome here; the telemetry
 * pipeline surfaces it in the ExtendedSystemState so the ground station can see
 * whether the battery-backed Backup SRAM (which holds logic::control::
 * persistent_state) will actually survive a VBAT-only power loss. Process-wide and
 * NOT itself battery-backed (like the refused_* diagnostics): it is re-probed and
 * resets to Unknown on every boot until the platform sets it.
 *
 * A RegulatorTimeout does NOT mean the SRAM is unreadable this boot — it is already
 * clocked and writable — only that VBAT retention across the next power cycle is
 * unconfirmed.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/** @brief Backup-domain retention health, as probed by the platform at boot. */
enum class BackupStatus : uint8_t {
    Unknown   = 0,  /**< Not probed yet (the default until the platform reports). */
    Retained  = 1,  /**< Backup regulator ready (BRRDY) — VBAT retention confirmed. */
    Unretained = 2, /**< Regulator never reported ready — retention NOT confirmed this power cycle. */
};

/** @brief The backup-domain health the platform probed at boot. Surfaced in the
 *         ExtendedSystemState; defaults to Unknown until the platform sets it. */
inline BackupStatus backup_status = BackupStatus::Unknown;

} // namespace logic::control
