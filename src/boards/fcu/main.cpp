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
#include "control/persistent_state.hpp"   // fill_state — colours the status indicator

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

/* Main-loop heartbeat + state indicator: g_status_indicator blinks one of the three
 * status LEDs (by control state) from the for(;;) loop below. Bound to their pins by
 * board::wireDrivers(). */
indication::Led      g_led_green;
indication::Led      g_led_yellow;
indication::Led      g_led_red;
StatusIndicator      g_status_indicator{g_led_green, g_led_yellow, g_led_red};

/* The e-match: three GPIO lines + the GpioEmatch composed over them. Constructed before
 * g_controller (which holds a reference to it via Control/Telemetry). Bound to their pins
 * by board::wireDrivers(); plain RAM (no DMA), so no .axisram placement needed. */
gpio::DigitalOutput  g_ematch_fire;
gpio::DigitalInput   g_ematch_detect;
gpio::DigitalOutput  g_ematch_cont;
Ematch               g_ematch{g_ematch_fire, g_ematch_detect, g_ematch_cont};

/* The solenoid valve: same GPIO shape as the e-match. Constructed before g_controller
 * (which holds a reference to it via Control/Telemetry). Plain RAM (no DMA). */
gpio::DigitalOutput  g_solenoid_drive;
gpio::DigitalInput   g_solenoid_detect;
gpio::DigitalOutput  g_solenoid_cont;
Solenoid             g_solenoid{g_solenoid_drive, g_solenoid_detect, g_solenoid_cont};

/* The heater: a single GPIO output line + the GpioHeater over it. Constructed before
 * g_controller (which holds a reference to it via Control/Telemetry). Plain RAM (no DMA). */
gpio::DigitalOutput  g_heater_drive;
Heater               g_heater{g_heater_drive};

__attribute__((section(".axisram"))) platform::storage::SdCard g_card_fast;
__attribute__((section(".axisram"))) platform::storage::SdCard g_card_slow;
__attribute__((section(".axisram"))) platform::storage::SdCard g_card_ext;
__attribute__((section(".axisram")))
FcuController g_controller{g_card_fast, g_card_slow, g_card_ext, g_fill_valve, g_dump_valve,
                           g_ads131, g_eth, g_can, g_thermocouples, g_power_monitor,
                           g_ematch, g_solenoid, g_heater};

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
    // Main-loop liveness blink on the LED whose colour matches the current state
    // (steady blink = loop alive; the colour shows Safe / armed / fault).
    fcu_app::g_status_indicator.tick(now, logic::control::persistent_state.fill_state);
    fcu_app::g_fill_valve.tick(now);  // advance each valve's open/close + limit-switch state machine
    fcu_app::g_dump_valve.tick(now);
    fcu_app::g_controller.tick(now);  // also services the INA3221 + folds it into the extended record
    // Pace one staged SD block onto the card if the engine is idle and the card is ready. This is
    // the only SD work in the loop — a quick card-state poll + a DMA kick, never an f_write/f_sync
    // (writes are fire-and-forget raw-sector DMA into each file's pre-allocated extent). Runs after
    // the valve + control ticks so SD never delays actuation.
    platform::storage::sd_write_engine().tick();
  }
}
