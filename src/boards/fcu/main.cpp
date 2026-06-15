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

/* The FCU's application object graph. The SD log files (g_card_fast/slow/ext) and the
 * controller are pinned in D1 AXI-SRAM: the controller's telemetry double buffer is
 * handed straight to the SDMMC DMA, which cannot reach DTCM. The cards are defined
 * (and so constructed) before g_controller, which holds references to them. The drivers
 * are constructed unbound here; board::wireDrivers() binds them to the board's HAL handles. */
namespace fcu_app {

valve::BallValve      g_fill_valve;
valve::BallValve      g_dump_valve;
ads131m08::Ads131m08  g_ads131;
max31856::Max31856Bank g_thermocouples;
ina3221::Ina3221      g_power_monitor;
eth::Ethernet         g_eth;
can::Can              g_can;

/* Main-loop heartbeat: g_running_indicator blinks g_run_led from the for(;;) loop
 * below. Bound to its pin by board::wireDrivers(). */
indication::Led      g_run_led;
RunningIndicator     g_running_indicator{g_run_led};

__attribute__((section(".axisram"))) platform::storage::SdCard g_card_fast;
__attribute__((section(".axisram"))) platform::storage::SdCard g_card_slow;
__attribute__((section(".axisram"))) platform::storage::SdCard g_card_ext;
__attribute__((section(".axisram")))
FcuController g_controller{g_card_fast, g_card_slow, g_card_ext, g_fill_valve, g_dump_valve,
                           g_ads131, g_eth, g_can, g_thermocouples, g_power_monitor};

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
    fcu_app::g_running_indicator.tick(now);  // main-loop liveness blink (steady = loop alive)
    fcu_app::g_fill_valve.tick(now);  // advance each valve's open/close + limit-switch state machine
    fcu_app::g_dump_valve.tick(now);
    fcu_app::g_controller.tick(now);  // also services the INA3221 + folds it into the extended record
  }
}
