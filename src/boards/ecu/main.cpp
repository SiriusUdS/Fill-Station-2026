/**
  ******************************************************************************
  * @file           : main.cpp  (ECU)
  * @brief          : ECU application composition + entry point (drivers-only).
  *
  * The handle-free half of the ECU board. It *defines* the driver object graph
  * (two ball valves, the streaming ADC, the SD card, the CAN node) and runs the
  * tick loop. It names no HAL handle/pin: bring-up is board::halInit() (chip) +
  * board::wireDrivers() (binds drivers to this board's hardware), both in board.cpp.
  *
  * There is no controller / logic::ecu yet ("wire drivers first, logic later"): the
  * loop ticks the valves while the ADC acquires and the SD/CAN are brought up, so
  * the hardware is exercised end-to-end before the control logic is written.
  ******************************************************************************
  */
#include "stm32h7xx_hal.h"   // HAL_GetTick

#include "board.hpp"         // board::halInit / board::wireDrivers
#include "ecu_objects.hpp"   // the ECU driver graph (declared extern, defined here)

/* The ECU's driver object graph. g_card is pinned in D1 AXI-SRAM (SDMMC DMA cannot
 * reach DTCM). The drivers are constructed unbound here; board::wireDrivers() binds
 * them to the board's HAL handles/pins. */
namespace ecu_app {

valve::BallValve     g_ipa_valve;
valve::BallValve     g_nos_valve;
ads131m08::Ads131m08 g_ads131;
can::Can             g_can;

__attribute__((section(".axisram"))) platform::storage::SdCard g_card;

}  // namespace ecu_app

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  board::halInit();      // chip, clocks, MPU, CubeMX peripherals (board.cpp)
  board::wireDrivers();  // bind drivers to this board's HAL handles/pins (board.cpp)

  for (;;)
  {
    const uint32_t now = HAL_GetTick();
    ecu_app::g_ipa_valve.tick(now);  // advance each valve's open/close + limit-switch state machine
    ecu_app::g_nos_valve.tick(now);
    // ADC acquires via the DRDY ISR; no controller drains it yet (logic later).
  }
}
