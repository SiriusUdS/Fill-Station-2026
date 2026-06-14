/**
  ******************************************************************************
  * @file           : main.cpp  (ECU)
  * @brief          : ECU application composition + entry point.
  *
  * The handle-free half of the ECU board. It *defines* the application object graph
  * (two ball valves, the streaming ADC, the SD card, the CAN node, and the engine
  * controller built over them) and runs the tick loop. It names no HAL handle/pin:
  * bring-up is board::halInit() (chip) + board::wireDrivers() (binds drivers to this
  * board's hardware + g_controller.init()), both in board.cpp.
  ******************************************************************************
  */
#include "stm32h7xx_hal.h"   // HAL_GetTick

#include "board.hpp"         // board::halInit / board::wireDrivers
#include "ecu_objects.hpp"   // the ECU object graph (declared extern, defined here)

/* The ECU's application object graph. The SD log files (g_card_fast/slow/ext) and
 * g_controller are pinned in D1 AXI-SRAM (the controller's telemetry double buffer is
 * handed straight to the SDMMC DMA, which cannot reach DTCM); the cards are defined
 * before g_controller, which holds references to them. The drivers are constructed
 * unbound; board::wireDrivers() binds them and brings the controller up. */
namespace ecu_app {

valve::BallValve     g_ipa_valve;
valve::BallValve     g_nos_valve;
ads131m08::Ads131m08 g_ads131;
can::Can             g_can;

__attribute__((section(".axisram"))) platform::storage::SdCard g_card_fast;
__attribute__((section(".axisram"))) platform::storage::SdCard g_card_slow;
__attribute__((section(".axisram"))) platform::storage::SdCard g_card_ext;
__attribute__((section(".axisram")))
EcuController g_controller{g_card_fast, g_card_slow, g_card_ext,
                           g_ipa_valve, g_nos_valve, g_ads131, g_can};

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
    ecu_app::g_controller.tick(now); // drain CAN, flush telemetry; produceRecord is on the timer
  }
}
