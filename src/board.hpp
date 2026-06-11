#pragma once

/* The board-support contract.
 *
 * Everything that differs between the two physical boards (system + peripheral
 * clocks, MPU, peripheral *instances*, pin assignments, and ISR vectors) is
 * implemented per board under boards/<board>/board.cpp. Shared application code
 * reaches the hardware ONLY through this interface: it names no pin macro, no HAL
 * handle (hspiN/htimN/…), no MX_*_Init, and no ISR vector symbol.
 *
 * This is the seam that lets one firmware build for boards with entirely different
 * pinouts — only the per-board board.cpp changes.
 */
namespace board {

/* Bring up the STM32 HAL, the system and peripheral clocks, the MPU, and every
 * CubeMX-configured peripheral (the MX_*_Init list) for THIS board. Pure HAL
 * bring-up only — constructs no application object and touches no driver state. */
void halInit();

}  // namespace board
