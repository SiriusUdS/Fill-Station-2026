#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "control/states.hpp"  // logic::control::State

/* ------------------------------------------------------------------------- *
 * Persistent state — a snapshot of the global filling-station state machine
 * that survives a reset (and battery-only power loss) by living in the MCU's
 * battery-backed Backup SRAM.
 *
 * `fill_state` is THE global state-machine state (the same value carried in
 * FillStation::fill_state and downlinked in telemetry): every state is saved,
 * so the ground station can see what the board was doing when it died. The
 * boot path only branches on a subset of states — e.g. an in-flight/launch
 * state where normal init must be skipped because driving the valves to their
 * init position mid-flight is dangerous — but the full state is persisted so
 * the value is meaningful for telemetry, not just for that one decision.
 *
 * The blob is self-validating: a magic word answers "is this memory ours, or
 * cold-boot garbage?" and a CRC over the payload answers "did it survive
 * intact?". validateLoadedState() must pass BOTH before any field is trusted.
 *
 * Fail-safe note: anything uncertain — bad magic, bad CRC, backup domain lost
 * power — makes validateLoadedState() return false, which the caller treats as
 * a normal boot. That is the safe default on the ground; it is only correct in
 * flight if the backup domain is kept alive by VBAT (battery/supercap) so the
 * saved state actually outlives the power event that reset the MCU.
 * ------------------------------------------------------------------------- */

namespace logic::control {

/**
 * @brief Battery-backed snapshot of the global state machine.
 *
 * Deliberately a trivial aggregate (no default member initializers, no user
 * constructors) so the platform can place the live instance in a no-init
 * Backup SRAM section without a constructor running at startup and wiping the
 * retained value. Set the payload fields, then call saveState() to commit.
 *
 * Layout is fixed (see static_assert below): magic + fill_state + reserved are
 * the payload; crc covers exactly that payload. Add new persisted fields in
 * the reserved space (shrinking it) and the CRC span follows automatically.
 */
struct PersistentState {
    uint32_t magic;        /**< PERSISTENT_STATE_MAGIC once written; else garbage. */
    State    fill_state;   /**< Global state-machine state (uint8_t underlying). */
    uint8_t  reserved[3];  /**< Explicit padding; keeps layout fixed and crc 4-byte aligned. */
    uint32_t crc;          /**< CRC-32 over every byte preceding this field. */

    /**
     * @brief  Is the loaded blob trustworthy?
     * @return true only if the magic word matches AND the CRC recomputes.
     *         A false result means: treat this as a cold/normal boot.
     */
    [[nodiscard]] bool validateLoadedState() const;

    /**
     * @brief  Load the persisted state, if this blob is trustworthy.
     * @return The saved State, or std::nullopt on a cold / corrupt boot — in
     *         which case the caller should fall back to a normal boot.
     */
    [[nodiscard]] std::optional<State> loadState() const;

    /**
     * @brief Set the state and commit it to this (Backup-SRAM-resident) blob.
     *
     * Writes the payload, zeroes the reserved padding, stamps the magic word,
     * and writes the CRC LAST — so a reset that interrupts the commit leaves
     * the CRC mismatching and loadState()/validateLoadedState() reject the
     * half-written blob rather than trusting it.
     */
    void saveState(State state);

  private:
    /** @brief CRC-32 over the payload (every byte preceding `crc`). */
    [[nodiscard]] uint32_t computeCrc() const;
};

// Storage layout guard: no implicit padding, crc sits right after the payload.
// If this fails, the CRC span and any reader on the ground would disagree about
// the bytes — fix the struct rather than the assert.
static_assert(sizeof(PersistentState) == 3 * sizeof(uint32_t),
              "PersistentState has implicit padding — adjust the reserved bytes");
static_assert(offsetof(PersistentState, crc) == 2 * sizeof(uint32_t),
              "crc must immediately follow the payload it covers");

/** @brief Sentinel proving the Backup SRAM holds our blob and not cold garbage. */
inline constexpr uint32_t PERSISTENT_STATE_MAGIC = 0xF111ED01u;

/**
 * @brief The single persistent-state instance.
 *
 * Defined in persistent_state.cpp and placed by the linker into the
 * `.backup_sram` section, which the CM7 linker script maps to the MCU's
 * battery-backed Backup SRAM — NOLOAD, so startup never copies or zeroes it and
 * the value survives a reset. For the value to also outlive a power cycle on
 * VBAT, the platform must first bring the backup domain up: enable the Backup
 * SRAM clock (BKPRAMEN), clear write protection (DBP) and turn on the backup
 * regulator (BREN). More persisted fields can be added to PersistentState over
 * time; they ride along in the same region and the CRC covers them automatically.
 */
extern PersistentState persistent_state;

} // namespace logic::control
