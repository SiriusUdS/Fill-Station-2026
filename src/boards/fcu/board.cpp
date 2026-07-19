/**
  ******************************************************************************
  * @file           : board.cpp  (FCU)
  * @brief          : FCU board-support: HAL bring-up + driver wiring + ISRs.
  *
  * Implements the board:: contract (src/app/board.hpp) for the FCU. This is the
  * board-specific half of the firmware and the ONLY place that names HAL handles,
  * pin macros, peripheral instances, and ISR vectors. The handle-free application
  * composition (the object graph + tick loop) lives in main.cpp; here we bring up
  * the chip (halInit) and bind each driver to this board's hardware (wireDrivers).
  ******************************************************************************
  */
#include "board.hpp"

/* CubeMX-generated peripheral init declarations + HAL handles (CM7/Core/Inc). */
#include "main.h"
#include "dma.h"
#include "eth.h"
#include "fatfs.h"
#include "fdcan.h"
#include "i2c.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "crc.h"
#include "iwdg.h"

/* The FCU object graph (defined in main.cpp) + the bits wireDrivers() needs. */
#include "fcu_objects.hpp"
#include "memory/backup_ram.hpp"
#include "control/backup_status.hpp"   // logic::control::backup_status (boot retention probe)
#include "data_integrity/crc/crc.hpp"
#include "system/watchdog/watchdog.hpp"  // watchdog::init — binds the IWDG behind logic::control::watchdog::kick
#include "framing/can_header.hpp"
#include "system/board_id.hpp"   // BoardId::FillingStation

using namespace fcu_app;
namespace backup_ram = platform::memory::backup_ram;
namespace crc        = platform::data_integrity::crc;
namespace watchdog   = platform::system::watchdog;

static void SystemClock_Config(void);
static void PeriphCommonClock_Config(void);
static void MPU_Config(void);

namespace board {

void halInit(void)
{
  /* Enable the CPU instruction cache. CubeMX has CPU_ICache=Enabled, but it emits
     SCB_EnableICache() into main.c, which is excluded from this build (we own the entry
     point), so the call is carried here. I-cache needs no MPU/coherency work — nothing
     DMAs instructions, and code lives in flash written only at programming time. Done first,
     matching CubeMX's placement at the top of main(). */
  SCB_EnableICache();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();    // hardware CRC unit (telemetry frame CRC; configured by crc::init)
  MX_ETH_Init();
  /* SD card on SDMMC2 — NON-FATAL bring-up. The generated MX_SDMMC2_SD_Init() calls
     Error_Handler() (a hard __disable_irq + while(1) lock) when HAL_SD_Init fails, so a board with
     no card seated would wedge here at boot, before the main loop. Instead fill the same per-board
     config (mirrored from CM7/Core/Src/sdmmc.c — keep in sync if CubeMX regenerates it) and bring
     the card up through the non-fatal tryInitSd(): a missing/dead card just leaves sdPresent()==
     false and logging off (SdCard::init() short-circuits) while everything else runs normally.
     HAL_SD_Init still invokes HAL_SD_MspInit (GPIO/clock/NVIC) internally. */
  hsd2.Instance                 = SDMMC2;
  hsd2.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide             = SDMMC_BUS_WIDE_4B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd2.Init.ClockDiv            = 2;
  platform::storage::tryInitSd(&hsd2);
  MX_FATFS_Init();
  MX_FDCAN1_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_I2C4_Init();   // INA3221 power monitor bus (PF14/PF15)
  MX_TIM1_Init();   // Fill valve servo PWM (PE9 / TIM1_CH1)
  MX_TIM15_Init();  // Dump valve servo PWM (PE6 / TIM15_CH2)
  MX_TIM6_Init();   // record-production cadence (drives g_controller.produceRecord)
  /* Independent watchdog LAST: MX_IWDG_Init() starts the ~30 s counter, so the boot window to the
     first serviced ping starts here. The FCU feeds it only from a GS ping (handlePing ->
     watchdog::kick); a board that cannot service a ping for the timeout is reset as dead. */
  MX_IWDG1_Init();
}

void wireDrivers(void)
{
  /* Start the main-loop heartbeat first, so a steady blink confirms the loop is alive
     through the rest of bring-up (a frozen LED means it stalled). The indicator blinks
     ONE of the three status LEDs by control state: green = Safe, yellow = any other
     non-Error/Abort state, red = Error/Abort. The colour<->pin mapping is a best guess
     (LED1/2/3 = PF1/2/3 = green/yellow/red) — swap the pins HERE if the board's physical
     colours differ. The GPIOF clock is already enabled by MX_GPIO_Init. */
  g_led_green.init({.port = LED1_STATE_GPIO_Port, .pin = LED1_STATE_Pin});
  g_led_yellow.init({.port = LED2_STATE_GPIO_Port, .pin = LED2_STATE_Pin});
  g_led_red.init({.port = LED3_STATE_GPIO_Port, .pin = LED3_STATE_Pin});
  g_status_indicator.init();

  /* Bring up the e-match (igniter) lines on GPIOD (clock enabled by MX_GPIO_Init):
       - EMATCH_STATE (PD12) firing output  — driven high ONLY during the Ignite state
         (Control::onTransition); init() drives it low (safe).
       - EMATCH_DET  (PD13) present input   — active-high "an e-match is plugged in"; flip
         active_high here if the board's polarity changes.
       - EMATCH_CONT (PD8)  continuity LED  — lit while an e-match is detected.
     The GpioEmatch composes these; the controller energises/polls it. */
  g_ematch_fire.init({.port = EMATCH_STATE_GPIO_Port, .pin = EMATCH_STATE_Pin, .active_high = true});
  g_ematch_cont.init({.port = EMATCH_CONT_GPIO_Port,  .pin = EMATCH_CONT_Pin,  .active_high = true});
  g_ematch_detect.init({.port = EMATCH_DET_GPIO_Port, .pin = EMATCH_DET_Pin,
                        .active_high = true, .pull = GPIO_NOPULL});

  /* Bring up the solenoid-valve coil output on GPIOB (clock enabled by MX_GPIO_Init):
       - SOLENOID_VALVE_STATE (PB7) coil output — driven open ONLY while the SolenoidValve flag
         is set AND the board is in Unsafe (Control::serviceSolenoid); init() drives it low
         (closed). */
  g_solenoid_drive.init({.port = SOLENOID_VALVE_STATE_GPIO_Port, .pin = SOLENOID_VALVE_STATE_Pin,
                         .active_high = true});

  /* Bring up the two heater outputs (clocks enabled by MX_GPIO_Init). Each is driven on/off
     straight from its own control flag (Control::serviceHeaters) in any state; init() drives
     both low (off):
       - HEATER_STATE      (PD1) main heater, from the Heater flag.
       - HEATER_TANK_STATE (PB6) tank heater, from the HeaterTank flag. */
  g_heater_drive.init({.port = HEATER_STATE_GPIO_Port, .pin = HEATER_STATE_Pin, .active_high = true});
  g_heater_tank_drive.init({.port = HEATER_TANK_STATE_GPIO_Port, .pin = HEATER_TANK_STATE_Pin,
                            .active_high = true});

  /* Bring up the backup domain first, so the battery-backed Backup SRAM that
     holds the persistent state is clocked, writable and retained on VBAT before
     the FCU logic reads it. A regulator-timeout only means VBAT retention is
     unconfirmed; the SRAM is still accessible, so we proceed — but record the
     outcome so telemetry surfaces whether the saved state will survive a power loss. */
  logic::control::backup_status =
      backup_ram::init() ? logic::control::BackupStatus::Unretained
                         : logic::control::BackupStatus::Retained;

  /* Configure the CRC peripheral for the zlib/reflected variant behind the
     logic data-integrity seam (logic::data_integrity::crc32), used to stamp the
     telemetry frame CRC. Must precede the first downlink (drainTick in tick()). */
  crc::init(&hcrc);

  /* Bind the IWDG (started by MX_IWDG_Init in halInit) behind the logic watchdog seam
     (logic::control::watchdog::kick), so a serviced ping in handlePing can refresh it. */
  watchdog::init(&hiwdg1);

  /* Bring up the CAN and Ethernet drivers, then the FCU logic. The board now
     answers ARP and ICMP echo (ping) requests and exchanges UDP/CAN traffic
     through the logic interfaces. */
  (void)g_can.init(&hfdcan1, static_cast<uint8_t>(BoardId::FillingStation));
  g_eth.init();

  /* Bring up the ADS131M08 ADC. The board owns the DRDY (PE7) pin: configure it
     as a falling-edge EXTI input here, then let the driver arm it. CS is PE15.
     Each DRDY-paced conversion is pushed into the driver's ring; the controller's
     record timer drains it (see produceRecord). */
  GPIO_InitTypeDef drdy = {};
  drdy.Pin  = GPIO_PIN_7;
  drdy.Mode = GPIO_MODE_IT_FALLING;
  drdy.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &drdy);

  /* Per-channel ADC calibration — one constant per channel, tune in place. pga_gain is the
     coarse analog PGA gain (PgaGain::x1..x128); ocal_offset is a 24-bit signed offset in ADC
     counts (0 = none); gcal_gain is the fine digital gain trim (GCAL_UNITY = no trim). The
     values below reproduce the historical fixed setting (all x32 / no offset / unity). */
  using ads131m08::PgaGain;
  g_ads131.init({
      .hspi      = &hspi4,
      .cs_port   = GPIOE,
      .cs_pin    = GPIO_PIN_15,
      .drdy_pin  = GPIO_PIN_7,
      .drdy_irqn = EXTI9_5_IRQn,
      //              ch0          ch1          ch2          ch3          ch4          ch5          ch6          ch7
      .pga_gain    = { PgaGain::x128, PgaGain::x1, PgaGain::x64, PgaGain::x1, PgaGain::x1, PgaGain::x1, PgaGain::x1, PgaGain::x1 },
      .ocal_offset = {            0,            0,            0,            0,            0,            0,            0,            0 },
      .gcal_gain   = { ads131m08::GCAL_UNITY, ads131m08::GCAL_UNITY, ads131m08::GCAL_UNITY, ads131m08::GCAL_UNITY,
                       ads131m08::GCAL_UNITY, ads131m08::GCAL_UNITY, ads131m08::GCAL_UNITY, ads131m08::GCAL_UNITY },
  });
  g_ads131.start();

  /* Bring up the 2 MAX31856 thermocouples on SPI6. The board owns both chip-selects:
     CubeMX left TC1_CS asserted (low) and TC2_CS as a floating input, which is wrong
     for a shared SPI bus — every CS must be a push-pull output idling deasserted (high).
     Configure both as outputs here; the driver then deasserts each, configures the
     devices, and polls them NON-BLOCKING from the main loop over interrupt-driven SPI6
     (so a plugged-in board never stalls the loop). Channel order TC1..TC2 maps to
     thermocouple_info[0..1]. NOTE: MX_SPI6_Init must use CPHA = 1 (SPI mode 1/3). */
  GPIO_InitTypeDef tc_cs = {};
  tc_cs.Mode  = GPIO_MODE_OUTPUT_PP;
  tc_cs.Pull  = GPIO_NOPULL;
  tc_cs.Speed = GPIO_SPEED_FREQ_LOW;
  tc_cs.Pin = TC1_CS_Pin; HAL_GPIO_Init(TC1_CS_GPIO_Port, &tc_cs);
  tc_cs.Pin = TC2_CS_Pin; HAL_GPIO_Init(TC2_CS_GPIO_Port, &tc_cs);

  g_thermocouples.init({
      .hspi     = &hspi6,
      .cs_ports = { TC1_CS_GPIO_Port, TC2_CS_GPIO_Port },
      .cs_pins  = { TC1_CS_Pin, TC2_CS_Pin },
  });

  /* Bring up the INA3221 power monitor on I2C4 (PF14 SCL / PF15 SDA). init() writes the
     config (also a presence check) and arms the I2C4 event/error interrupts; the controller
     then services it non-blocking from the loop and folds it into the extended record. The
     EV/ER vector handlers are below. Default 7-bit address 0x40 (A0 = GND). */
  g_power_monitor.init({
      .hi2c    = &hi2c4,
      .address = ina3221::DEFAULT_ADDRESS,
      .ev_irqn = I2C4_EV_IRQn,
      .er_irqn = I2C4_ER_IRQn,
  });

  /* Bring up the shared async SD write engine FIRST: it arbitrates the one SDMMC peripheral
     across all three files and is driven by the SDMMC2 completion ISR, so it must be online
     before any file opens (and before that IRQ can fire). One engine per physical card. Also
     hand it the SD_DETECT socket switch (PD4) so card-present rides the extended telemetry. */
  platform::storage::sd_write_engine().init(&hsd2, SD_DETECT_GPIO_Port, SD_DETECT_Pin);

  /* Bind the three SD log files to the HAL handle + FatFs drive + file name (the app
     composition left them unbound). They share one card: g_controller.init() mounts the
     volume + creates this boot's session folder once, then opens all three files inside it
     (fcu_data_fast.bin / fcu_data_slow.bin / fcu_data_ext.bin) and f_expands each to a fixed CONTIGUOUS
     region. The pre-alloc sizes are this run's space budget per stream (the run stops writing a
     stream, rather than growing it, once full); finalize() reclaims each file's unused tail on a
     clean shutdown. Each pre-alloc is computed from its mode's configured duration (the knobs live
     in sd_recorder.hpp: FAST/SLOW/EXT_LOG_DURATION_S) via logBytesForSeconds(), then the
     static_assert re-checks the rounded pre-alloc still covers that duration. fast (2 kHz) and slow
     (125 Hz) are mutually exclusive in time, so each holds its own mode; ext (~10 Hz) is written in
     BOTH, so it spans fast + slow (1 h + 10 h = 11 h). FcuSystemState = 60 B, FcuExtendedSystemState
     the low-rate record. ~718 MiB total per boot session. */
  namespace tlm = logic::telemetry;

  static constexpr uint32_t FAST_PREALLOC_BYTES =
      tlm::logBytesForSeconds<FcuSystemState>(tlm::FAST_RECORD_RATE_HZ, tlm::FAST_LOG_DURATION_S);
  static_assert(tlm::logCapacitySeconds<FcuSystemState>(tlm::FAST_RECORD_RATE_HZ, FAST_PREALLOC_BYTES)
                    >= tlm::FAST_LOG_DURATION_S,
                "fcu_data_fast.bin pre-alloc must cover FAST_LOG_DURATION_S of 2 kHz FcuSystemState");

  static constexpr uint32_t SLOW_PREALLOC_BYTES =
      tlm::logBytesForSeconds<FcuSystemState>(tlm::SLOW_RECORD_RATE_HZ, tlm::SLOW_LOG_DURATION_S);
  static_assert(tlm::logCapacitySeconds<FcuSystemState>(tlm::SLOW_RECORD_RATE_HZ, SLOW_PREALLOC_BYTES)
                    >= tlm::SLOW_LOG_DURATION_S,
                "fcu_data_slow.bin pre-alloc must cover SLOW_LOG_DURATION_S of 125 Hz FcuSystemState");

  static constexpr uint32_t EXT_PREALLOC_BYTES =
      tlm::logBytesForSeconds<FcuExtendedSystemState>(tlm::EXT_RECORD_RATE_HZ, tlm::EXT_LOG_DURATION_S);
  // ext logs in both fast and slow, so its budget must span both (EXT_LOG_DURATION_S = fast + slow).
  static_assert(tlm::logCapacitySeconds<FcuExtendedSystemState>(tlm::EXT_RECORD_RATE_HZ, EXT_PREALLOC_BYTES)
                    >= tlm::FAST_LOG_DURATION_S + tlm::SLOW_LOG_DURATION_S,
                "fcu_data_ext.bin pre-alloc must cover fast + slow of ~10 Hz FcuExtendedSystemState (it logs in both modes)");

  g_card_fast.bind(&hsd2, "0:/", "fcu_data_fast.bin", FAST_PREALLOC_BYTES);   // 2 kHz  -> FAST_LOG_DURATION_S (1 h)
  g_card_slow.bind(&hsd2, "0:/", "fcu_data_slow.bin", SLOW_PREALLOC_BYTES);   // 125 Hz -> SLOW_LOG_DURATION_S (10 h)
  g_card_ext.bind (&hsd2, "0:/", "fcu_data_ext.bin",  EXT_PREALLOC_BYTES);    // ~10 Hz -> fast + slow (11 h)

  /* Bring up the two local ball valves. The 333 Hz (3 ms) servo PWM frequency is
     owned HERE, not in CubeMX: leave the generated timer at its defaults and set
     the period in code so it lives in one place. 100 MHz APB2 clock / (PSC 99 + 1)
     = 1 MHz tick; ARR 2999 -> 3 ms. The UG event reloads the prescaler immediately
     so the first period is already correct. Servo pulse 1-2 ms = 1000-2000 ticks;
     tune min/max for the valve's actual travel. */
  __HAL_TIM_SET_PRESCALER(&htim1,   99);
  __HAL_TIM_SET_AUTORELOAD(&htim1,  2999);
  htim1.Instance->EGR = TIM_EGR_UG;   // reload PSC/ARR now (no first-period glitch)
  __HAL_TIM_SET_PRESCALER(&htim15,  99);
  __HAL_TIM_SET_AUTORELOAD(&htim15, 2999);
  htim15.Instance->EGR = TIM_EGR_UG;

  g_fill_valve.init({
      .servo       = {.htim = &htim1, .channel = TIM_CHANNEL_1},
      .open_limit  = {.port = FILL_SWITCH_OPENED_GPIO_Port, .pin = FILL_SWITCH_OPENED_Pin},
      .close_limit = {.port = FILL_SWITCH_CLOSED_GPIO_Port, .pin = FILL_SWITCH_CLOSED_Pin},
      .max_transit_timeout_ms = 5000,
      .duty_closed_percent = 26.0F,   // Fill calibration: 26 % duty = fully closed
      .duty_open_percent   = 54.0F,   //                   54 % duty = fully open
      .inverted            = true,    // The fill valve's servo is installed in the opposite orientation to the dump, so invert the drive direction here (instead of swapping min/max pulse) to keep the duty<->position mapping consistent between them: 26 % duty = closed, 54 % duty = open, for both valves.
  });
  g_dump_valve.init({
      .servo       = {.htim = &htim15, .channel = TIM_CHANNEL_2},
      .open_limit  = {.port = DUMP_SWITCH_OPENED_GPIO_Port, .pin = DUMP_SWITCH_OPENED_Pin},
      .close_limit = {.port = DUMP_SWITCH_CLOSED_GPIO_Port, .pin = DUMP_SWITCH_CLOSED_Pin},
      .max_transit_timeout_ms = 5000,
      .duty_closed_percent = 26.0F,   // Dump calibration: 26 % duty = fully closed
      .duty_open_percent   = 54.0F,   //                   54 % duty = fully open
      .inverted            = true,    // The dump valve's servo is installed in the opposite orientation to the fill, so invert the drive direction here (instead of swapping min/max pulse) to keep the duty<->position mapping consistent between them: 26 % duty = closed, 54 % duty = open, for both valves.
  });

  /* Bring up the FCU controller. It resumes the persisted state and then safes the
     local valves (drives them closed) through the control layer — actuation is the
     control layer's authority, so boot-safing lives there, not in board bring-up.
     This also mounts the SD card via g_card.init(). */
  g_controller.init();

  /* Start the record-production timer last, so records only flow once the logic
     and SD are ready. The 2 kHz (0.5 ms) cadence is owned HERE, not in CubeMX:
     leave the generated TIM6 at its defaults and set the period in code so it
     lives in one place. 100 MHz APB1 timer clock / (PSC 99 + 1) = 1 MHz tick;
     ARR 499 -> 0.5 ms = 2 kHz. The UG event reloads PSC/ARR immediately. This is
     the comms/save cadence, decoupled from the ADC's DRDY rate: each tick drains
     the ADC ring and produces one record per queued conversion. */
  __HAL_TIM_SET_PRESCALER(&htim6,  99);
  __HAL_TIM_SET_AUTORELOAD(&htim6, 499);
  htim6.Instance->EGR = TIM_EGR_UG;          // reload PSC/ARR now (no first-period glitch)
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);  // UG set the flag; clear so we don't fire immediately
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);
}

}  // namespace board

/* ---- Board ISR vectors (symbols fixed by the chosen timer / pin group) ---- */

/* Record-production timer (TIM6) ISR -> controller. TIM6 fires at a fixed cadence
   (set in wireDrivers), decoupled from the ADC's DRDY rate. */
extern "C" void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM6) {
    fcu_app::g_controller.produceRecord(HAL_GetTick());
  }
}

/* DRDY (PE7) data-ready EXTI vector. The handler symbol is fixed by the pin group
   (EXTI lines 5-9); it dispatches through the HAL to the ADC driver's
   HAL_GPIO_EXTI_Callback. */
extern "C" void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}

/* INA3221 I2C4 event + error vectors. The driver runs the per-loop reads in interrupt mode
   (I2C4 is D3/BDMA-only, same as SPI6), so these drive the HAL I2C state machine, which
   dispatches to the driver's HAL_I2C_MemRxCpltCallback / HAL_I2C_ErrorCallback. */
extern "C" void I2C4_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c4);
}

extern "C" void I2C4_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c4);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
extern "C" void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
static void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_SPI4;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 300;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 16;
  PeriphClkInitStruct.PLL2.PLL2R = 3;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 12;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL3;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* MPU Configuration */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
