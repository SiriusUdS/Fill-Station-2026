/**
  ******************************************************************************
  * @file           : main.cpp  (FCU)
  * @brief          : FCU application composition + entry point.
  *
  * The handle-free half of the FCU board. It *defines* the application object
  * graph (the drivers + the controller built over them) and runs the tick loop.
  * It names no HAL handle, pin, or peripheral instance: bring-up is delegated to
  * board::halInit() (chip/clocks/peripherals) and board::wireDrivers() (binds each
  * driver to this board's HAL handles/pins) — both in board.cpp. ECU's main.cpp is
  * the same shape over a different object set.
  ******************************************************************************
  */
#include "stm32h7xx_hal.h"   // HAL_GetTick

#include "board.hpp"         // board::halInit / board::wireDrivers
#include "fcu_objects.hpp"   // the FCU object graph (declared extern, defined here)

/* The FCU's application object graph. g_card and the controller are pinned in D1
 * AXI-SRAM: the controller's telemetry double buffer is handed straight to the
 * SDMMC DMA, which cannot reach DTCM. g_card is defined (and so constructed)
 * before g_controller, which holds a reference to it. The drivers are constructed
 * unbound here; board::wireDrivers() binds them to the board's HAL handles. */
namespace fcu_app {

valve::BallValve     g_fill_valve;
valve::BallValve     g_dump_valve;
ads131m08::Ads131m08 g_ads131;
eth::Ethernet        g_eth;
can::Can             g_can;

__attribute__((section(".axisram"))) platform::storage::SdCard g_card;
__attribute__((section(".axisram")))
FcuController g_controller{g_card, g_fill_valve, g_dump_valve, g_ads131, g_eth, g_can};

}  // namespace fcu_app

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
    fcu_app::g_fill_valve.tick(now);  // advance each valve's open/close + limit-switch state machine
    fcu_app::g_dump_valve.tick(now);
    fcu_app::g_controller.tick(now);
  }
}
