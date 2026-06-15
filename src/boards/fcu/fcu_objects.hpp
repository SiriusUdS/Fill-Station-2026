#pragma once

/* The FCU board's application object graph.
 *
 * Declares the driver + controller instances that make up the FCU. They are
 * *defined* in main.cpp (the handle-free app composition) and *bound* to HAL
 * handles/pins in board.cpp (board::wireDrivers). This header is what lets those
 * two board-side translation units share the same objects without either one
 * having to name the other's internals.
 */

#include "communication/ethernet/ethernet.hpp"
#include "communication/can/can_dil.hpp"
#include "acquisition/adc/ads131m08.hpp"
#include "acquisition/thermocouple/max31856.hpp"
#include "acquisition/power_monitor/ina3221.hpp"
#include "storage/sd_card.hpp"
#include "actuation/valve/ball_valve.hpp"
#include "gpio/digital_output.hpp"
#include "gpio/digital_input.hpp"
#include "indication/led.hpp"
#include "indication/running_indicator.hpp"
#include "gpio_ematch.hpp"
#include "gpio_solenoid.hpp"
#include "fcu_controller.hpp"

namespace fcu_app {

namespace eth        = platform::communication::ethernet;
namespace can        = platform::communication::can;
namespace ads131m08  = platform::acquisition::adc::ads131m08;
namespace max31856   = platform::acquisition::thermocouple::max31856;
namespace ina3221    = platform::acquisition::power_monitor::ina3221;
namespace valve      = platform::actuation::valve;
namespace indication = platform::indication;
namespace gpio       = platform::gpio;

/* The main-loop liveness indicator, blinking a status LED straight from the
 * for(;;) loop (a frozen LED means the loop stalled). */
using RunningIndicator = logic::indication::RunningIndicator<indication::Led>;

/* The FCU e-match: firing output + present-detect input + continuity LED, all plain
 * GPIO behind the logic seams (EMATCH_STATE / EMATCH_DET / EMATCH_CONT on GPIOD). */
using Ematch = logic::fcu::GpioEmatch<gpio::DigitalOutput, gpio::DigitalInput, gpio::DigitalOutput>;

/* The FCU solenoid valve: coil output + present-detect input + continuity LED, same
 * GPIO shape as the e-match (SOL_VALVE_STATE / SOL_VALVE_DET / SOL_VALVE_CONT on GPIOD). */
using Solenoid = logic::fcu::GpioSolenoid<gpio::DigitalOutput, gpio::DigitalInput, gpio::DigitalOutput>;

/* The FCU's concrete controller type, instantiated over its real drivers. */
using FcuController = logic::fcu::Controller<platform::storage::SdCard, valve::BallValve,
                                             ads131m08::Ads131m08, eth::Ethernet, can::Can,
                                             max31856::Max31856Bank, ina3221::Ina3221, Ematch, Solenoid>;

/* Fill/Dump ball valves, the streaming ADC, the Ethernet link and CAN node, the
 * SD card, and the controller built over them all. Defined in main.cpp. */
extern valve::BallValve       g_fill_valve;
extern valve::BallValve       g_dump_valve;
extern ads131m08::Ads131m08   g_ads131;
extern max31856::Max31856Bank g_thermocouples;
extern ina3221::Ina3221       g_power_monitor;  // INA3221 on I2C4 (serviced by the controller; rides the extended record)
extern eth::Ethernet          g_eth;
extern can::Can               g_can;
extern platform::storage::SdCard g_card_fast;  // data_fast.bin (raw 2 kHz SystemState)
extern platform::storage::SdCard g_card_slow;  // data_slow.bin (125 Hz averaged SystemState)
extern platform::storage::SdCard g_card_ext;   // data_ext.bin  (ExtendedSystemState)
extern indication::Led        g_run_led;
extern RunningIndicator       g_running_indicator;

/* The e-match GPIO lines (EMATCH_STATE / EMATCH_DET / EMATCH_CONT on GPIOD) and the
 * GpioEmatch composed over them. Bound to their pins by board::wireDrivers(). */
extern gpio::DigitalOutput    g_ematch_fire;    // EMATCH_STATE  (firing output)
extern gpio::DigitalInput     g_ematch_detect;  // EMATCH_DET    (e-match-present input)
extern gpio::DigitalOutput    g_ematch_cont;    // EMATCH_CONT   (continuity LED)
extern Ematch                 g_ematch;

/* The solenoid-valve GPIO lines (SOL_VALVE_STATE / SOL_VALVE_DET / SOL_VALVE_CONT on
 * GPIOD) and the GpioSolenoid composed over them. Bound to their pins by wireDrivers(). */
extern gpio::DigitalOutput    g_solenoid_drive;   // SOL_VALVE_STATE (coil output)
extern gpio::DigitalInput     g_solenoid_detect;  // SOL_VALVE_DET   (present input)
extern gpio::DigitalOutput    g_solenoid_cont;    // SOL_VALVE_CONT  (continuity LED)
extern Solenoid               g_solenoid;

extern FcuController          g_controller;

}  // namespace fcu_app
