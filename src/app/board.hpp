#pragma once

/* The board-support contract.
 *
 * Everything that differs between the two physical boards (system + peripheral
 * clocks, MPU, peripheral *instances*, pin assignments, and ISR vectors) is
 * implemented per board under src/boards/<board>/board.cpp. The handle-free app
 * composition (main.cpp's object graph + tick loop) reaches the hardware ONLY
 * through this interface: it names no pin macro, no HAL handle (hspiN/htimN/…), no
 * MX_*_Init, and no ISR vector symbol.
 *
 * This is the seam that lets the firmware build for boards with entirely different
 * pinouts — only the per-board board.cpp changes.
 */
namespace board {

/* Bring up the STM32 HAL, the system and peripheral clocks, the MPU, and every
 * CubeMX-configured peripheral (the MX_*_Init list) for THIS board. Pure HAL
 * bring-up only — constructs no application object and touches no driver state. */
void halInit();

/* Bind each of THIS board's drivers to its HAL handles/pins, do the board-owned
 * timer/GPIO/NVIC config, and bring the board's logic up on top. Runs after
 * halInit(). Implemented per board, operating on that board's own object graph
 * (which main.cpp defines). */
void wireDrivers();

}  // namespace board
