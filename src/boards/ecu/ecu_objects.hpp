#pragma once

/* The ECU board's application object graph: the drivers + the engine controller
 * built over them. They are *defined* in main.cpp (the handle-free composition) and
 * *bound* to HAL handles/pins in board.cpp (board::wireDrivers); the controller is
 * brought up there too (g_controller.init()) and ticked from the main loop.
 */

#include "communication/can/can_dil.hpp"
#include "acquisition/adc/ads131m08.hpp"
#include "acquisition/power_monitor/ina3221.hpp"
#include "storage/sd_card.hpp"
#include "actuation/valve/ball_valve.hpp"
#include "ecu_controller.hpp"

namespace ecu_app {

namespace can       = platform::communication::can;
namespace ads131m08 = platform::acquisition::adc::ads131m08;
namespace ina3221   = platform::acquisition::power_monitor::ina3221;
namespace valve     = platform::actuation::valve;

/* The ECU's concrete controller type, instantiated over its real drivers. */
using EcuController = logic::ecu::Controller<platform::storage::SdCard, valve::BallValve,
                                             ads131m08::Ads131m08, can::Can, ina3221::Ina3221>;

/* Two propellant ball valves (IPA + NOS) on TIM15, the streaming ADC on SPI1, the
 * CAN node (telemetry -> FCU), the SD card, and the controller built over them all.
 * Defined in main.cpp. */
extern valve::BallValve          g_ipa_valve;
extern valve::BallValve          g_nos_valve;
extern ads131m08::Ads131m08      g_ads131;
extern ina3221::Ina3221          g_power_monitor;  // INA3221 on I2C4 (serviced by the controller; rides the extended record)
extern can::Can                  g_can;
extern platform::storage::SdCard g_card_fast;  // data_fast.bin (raw 2 kHz SystemState)
extern platform::storage::SdCard g_card_slow;  // data_slow.bin (125 Hz averaged SystemState)
extern platform::storage::SdCard g_card_ext;   // data_ext.bin  (ExtendedSystemState)
extern EcuController             g_controller;

}  // namespace ecu_app
