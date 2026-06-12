/**
  ******************************************************************************
  * @file           : main.cpp  (ECU)
  * @brief          : ECU application entry — do-nothing stub.
  *
  * The ECU board is cloned from the FCU and not yet thinned to its real hardware.
  * It boots (board::halInit: clocks + GPIO), calls an empty board::wireDrivers(),
  * and idles. No drivers, no logic yet — those arrive once the ECU peripherals are
  * regenerated in CubeMX and src/app/logic/ecu is built. ECU's main.cpp will then
  * mirror the FCU's shape over its own (different) object set.
  ******************************************************************************
  */
#include "board.hpp"   // board::halInit / board::wireDrivers

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  board::halInit();      // chip, clocks, MPU, GPIO (board.cpp)
  board::wireDrivers();  // no-op for the ECU stub

  for (;;)
  {
    // idle — the ECU does nothing yet
  }
}
