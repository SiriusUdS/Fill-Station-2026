#pragma once

/* The ECU board's driver object graph (no controller yet — "wire drivers first").
 *
 * Declares the driver instances that make up the ECU's hardware bring-up. They are
 * *defined* in main.cpp (the handle-free composition) and *bound* to HAL handles/pins
 * in board.cpp (board::wireDrivers). A controller / logic::ecu is added later; for now
 * the loop just ticks the valves while the ADC/SD/CAN are exercised.
 */

#include "communication/can/can_dil.hpp"
#include "acquisition/adc/ads131m08.hpp"
#include "storage/sd_card.hpp"
#include "actuation/valve/ball_valve.hpp"

namespace ecu_app {

namespace can       = platform::communication::can;
namespace ads131m08 = platform::acquisition::adc::ads131m08;
namespace valve     = platform::actuation::valve;

/* Two propellant ball valves (IPA + NOS) on TIM15, the streaming ADC on SPI1, the
 * CAN node (telemetry -> FCU), and the SD card. Defined in main.cpp. */
extern valve::BallValve          g_ipa_valve;
extern valve::BallValve          g_nos_valve;
extern ads131m08::Ads131m08      g_ads131;
extern can::Can                  g_can;
extern platform::storage::SdCard g_card;

}  // namespace ecu_app
