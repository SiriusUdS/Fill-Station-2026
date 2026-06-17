/**
  ******************************************************************************
  * @file           : board.cpp  (ECU)
  * @brief          : ECU board-support: HAL bring-up + driver wiring + ISRs.
  *
  * Implements the board:: contract (src/app/board.hpp) for the ECU. The ONLY place
  * that names HAL handles, pin macros, peripheral instances, and ISR vectors. The
  * handle-free composition (the object graph + tick loop) lives in main.cpp; here we
  * bring up the chip (halInit) and bind each driver to this board's hardware
  * (wireDrivers). No controller yet — "wire drivers first, logic later".
  ******************************************************************************
  */
#include "board.hpp"

/* CubeMX-generated peripheral init declarations + HAL handles (CM7/Core/Inc). */
#include "main.h"
#include "gpio.h"
#include "crc.h"
#include "fdcan.h"
#include "dma.h"
#include "spi.h"
#include "i2c.h"
#include "sdmmc.h"
#include "fatfs.h"
#include "tim.h"

/* The ECU object graph (defined in main.cpp) + the bits wireDrivers() needs. */
#include "ecu_objects.hpp"
#include "memory/backup_ram.hpp"
#include "control/backup_status.hpp"   // logic::control::backup_status (boot retention probe)
#include "data_integrity/crc/crc.hpp"  // crc::init — binds the HW CRC behind logic::data_integrity::crc32
#include "framing/can_header.hpp"
#include "system/board_id.hpp"   // BoardId::Engine

using namespace ecu_app;
namespace backup_ram = platform::memory::backup_ram;
namespace crc        = platform::data_integrity::crc;

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

  /* MPU + HAL + system clock, then the shared peripheral kernel clocks (SPI1 + I2C4 off
     PLL3 — CubeMX moved these out of the per-peripheral MspInit into PeriphCommonClock_Config
     when I2C4 was added; main.c is excluded from the build, so we call it here), then every
     CubeMX-configured peripheral. */
  MPU_Config();
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();

  MX_GPIO_Init();
  MX_CRC_Init();
  MX_FDCAN1_Init();
  MX_DMA_Init();         // must precede MX_SPI1_Init: SPI1 MspInit calls HAL_DMA_Init + arms the stream NVICs
  MX_SPI1_Init();        // ADS131M08 ADC bus
  MX_I2C4_Init();        // INA3221 power monitor bus
  MX_SDMMC1_SD_Init();   // SD card
  MX_FATFS_Init();
  MX_TIM15_Init();       // valve servo PWM (CH1=IPA/PE5, CH2=NOS/PE6)
  MX_TIM6_Init();        // record-production cadence (unused until the controller lands)
}

void wireDrivers(void)
{
  /* Start the main-loop heartbeat first, so a steady blink confirms the loop is alive
     through the rest of bring-up (a frozen LED means it stalled). Same three-LED
     indicator as the FCU: it blinks ONE status LED by control state — green = Safe,
     yellow = any other non-Error/Abort state, red = Error/Abort. Pin<->colour mapping
     for this board: green = LED3 (PF9), yellow = LED2 (PF8), red = LED1 (PF7) — swap the
     pins HERE if the board's physical colours differ. GPIOF clock is enabled by MX_GPIO_Init. */
  g_led_green.init({.port = LED3_STATE_GPIO_Port, .pin = LED3_STATE_Pin});
  g_led_yellow.init({.port = LED2_STATE_GPIO_Port, .pin = LED2_STATE_Pin});
  g_led_red.init({.port = LED1_STATE_GPIO_Port, .pin = LED1_STATE_Pin});
  g_status_indicator.init();

  /* Bring up the backup domain first so the battery-backed Backup SRAM that holds
     the persistent state is clocked + writable before the controller reads it. Record
     the outcome so telemetry can surface whether VBAT retention is confirmed: a
     RegulatorTimeout means the SRAM is usable this boot but may NOT survive the next
     VBAT-only power loss. */
  logic::control::backup_status =
      backup_ram::init() ? logic::control::BackupStatus::Unretained
                         : logic::control::BackupStatus::Retained;

  /* Configure the CRC peripheral for the zlib/reflected variant behind the logic
     data-integrity seam (logic::data_integrity::crc32), used to checksum every SD
     record. MX_CRC_Init() created the unit in halInit(); this binds + reconfigures it.
     Must precede the controller (its SdRecorder stamps record CRCs). */
  crc::init(&hcrc);

  /* CAN node — the ECU downlinks telemetry over CAN to the FCU. */
  (void)g_can.init(&hfdcan1, static_cast<uint8_t>(BoardId::Engine));

  /* ADS131M08 ADC on SPI1. The board owns the DRDY (PA4) pin: configure it as a
     falling-edge EXTI input, then let the driver arm it. CS is PC5.
     DRDY is active-low (idles high, pulses low when a conversion is ready). Enable the
     internal pull-up so the falling-edge EXTI line rests at its correct idle-high level
     whenever the device is not driving it (boot / reset / no CLKIN) instead of floating
     and firing on noise — this board has no hardware pull-up on DRDY (the FCU does). */
  GPIO_InitTypeDef drdy = {};
  drdy.Pin  = ADS_DRDY_Pin;     // PA4
  drdy.Mode = GPIO_MODE_IT_FALLING;
  drdy.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADS_DRDY_GPIO_Port, &drdy);   // GPIOA

  g_ads131.init({
      .hspi      = &hspi1,
      .cs_port   = ADS_CS_GPIO_Port,   // GPIOC
      .cs_pin    = ADS_CS_Pin,         // PC5
      .drdy_pin  = ADS_DRDY_Pin,       // PA4
      .drdy_irqn = EXTI4_IRQn,
  });
  g_ads131.start();

  /* Bring up the INA3221 power monitor on I2C4. init() writes the config (also a presence
     check) and arms the I2C4 event/error interrupts; the controller then services it
     non-blocking from the loop and folds it into the extended record. The EV/ER vector
     handlers are below. Default 7-bit address 0x40 (A0 = GND). */
  g_power_monitor.init({
      .hi2c    = &hi2c4,
      .address = ina3221::DEFAULT_ADDRESS,
      .ev_irqn = I2C4_EV_IRQn,
      .er_irqn = I2C4_ER_IRQn,
  });

  /* Bring up the single shared async write engine on SDMMC1 BEFORE any file opens (and before
     its completion IRQ can fire): it arbitrates the one SDMMC peripheral across all three files
     via raw-sector DMA. One engine per physical card (mirrors the FCU, which uses SDMMC2). */
  platform::storage::sd_write_engine().init(&hsd1);

  /* Bind the three SD log files on SDMMC1 (the app composition left them unbound). They
     share one card: g_controller.init() mounts the volume + creates this boot's session
     folder once, then opens all three files inside it and f_expands each to a fixed contiguous
     region (same recording policy as the FCU). The reserved tail is reclaimed by finalize() on a
     clean shutdown (the DisableLogging flag). Each pre-alloc is computed from its mode's configured
     duration (the knobs live in sd_recorder.hpp: FAST/SLOW/EXT_LOG_DURATION_S) via
     logBytesForSeconds(), then the static_assert re-checks the rounded pre-alloc still covers that
     duration. fast (2 kHz) and slow (125 Hz) are mutually exclusive in time, so each holds its own
     mode; ext (~10 Hz) is written in BOTH, so it spans fast + slow (11 h). The ECU's records are
     smaller than the FCU's (EcuSystemState = 56 B), so the same durations need slightly less space.
     Files carry an ecu_ prefix to tell them apart from the FCU's once collected. ~640 MiB per boot
     session. */
  namespace tlm = logic::telemetry;

  static constexpr uint32_t FAST_PREALLOC_BYTES =
      tlm::logBytesForSeconds<EcuSystemState>(tlm::FAST_RECORD_RATE_HZ, tlm::FAST_LOG_DURATION_S);
  static_assert(tlm::logCapacitySeconds<EcuSystemState>(tlm::FAST_RECORD_RATE_HZ, FAST_PREALLOC_BYTES)
                    >= tlm::FAST_LOG_DURATION_S,
                "ecu_data_fast.bin pre-alloc must cover FAST_LOG_DURATION_S of 2 kHz EcuSystemState");

  static constexpr uint32_t SLOW_PREALLOC_BYTES =
      tlm::logBytesForSeconds<EcuSystemState>(tlm::SLOW_RECORD_RATE_HZ, tlm::SLOW_LOG_DURATION_S);
  static_assert(tlm::logCapacitySeconds<EcuSystemState>(tlm::SLOW_RECORD_RATE_HZ, SLOW_PREALLOC_BYTES)
                    >= tlm::SLOW_LOG_DURATION_S,
                "ecu_data_slow.bin pre-alloc must cover SLOW_LOG_DURATION_S of 125 Hz EcuSystemState");

  static constexpr uint32_t EXT_PREALLOC_BYTES =
      tlm::logBytesForSeconds<EcuExtendedSystemState>(tlm::EXT_RECORD_RATE_HZ, tlm::EXT_LOG_DURATION_S);
  // ext logs in both fast and slow, so its budget must span both (EXT_LOG_DURATION_S = fast + slow).
  static_assert(tlm::logCapacitySeconds<EcuExtendedSystemState>(tlm::EXT_RECORD_RATE_HZ, EXT_PREALLOC_BYTES)
                    >= tlm::FAST_LOG_DURATION_S + tlm::SLOW_LOG_DURATION_S,
                "ecu_data_ext.bin pre-alloc must cover fast + slow of ~10 Hz EcuExtendedSystemState (it logs in both modes)");

  g_card_fast.bind(&hsd1, "0:/", "ecu_data_fast.bin", FAST_PREALLOC_BYTES);   // 2 kHz  -> FAST_LOG_DURATION_S (1 h)
  g_card_slow.bind(&hsd1, "0:/", "ecu_data_slow.bin", SLOW_PREALLOC_BYTES);   // 125 Hz -> SLOW_LOG_DURATION_S (10 h)
  g_card_ext.bind (&hsd1, "0:/", "ecu_data_ext.bin",  EXT_PREALLOC_BYTES);    // ~10 Hz -> fast + slow (11 h)

  /* Two propellant ball valves on TIM15: IPA = CH1 (PE5), NOS = CH2 (PE6). 333 Hz
     (3 ms) servo PWM owned here: 100 MHz / (PSC 99 + 1) = 1 MHz tick; ARR 2999 -> 3 ms.
     The closed/open endpoints are each valve's PWM duty-cycle calibration
     (BallValveConfig::duty_{closed,open}_percent, default 26 %/54 %); BallValve::init()
     converts them to pulse ticks against this period, so no pulse ticks are set here. */
  __HAL_TIM_SET_PRESCALER(&htim15,  99);
  __HAL_TIM_SET_AUTORELOAD(&htim15, 2999);
  htim15.Instance->EGR = TIM_EGR_UG;   // reload PSC/ARR now (no first-period glitch)

  g_ipa_valve.init({
      .servo       = {.htim = &htim15, .channel = TIM_CHANNEL_1},
      .open_limit  = {.port = SWITCH_VALVE_IPA_OPENED_GPIO_Port, .pin = SWITCH_VALVE_IPA_OPENED_Pin},
      .close_limit = {.port = SWITCH_VALVE_IPA_CLOSED_GPIO_Port, .pin = SWITCH_VALVE_IPA_CLOSED_Pin},
      .max_transit_timeout_ms = 5000,
  });
  g_nos_valve.init({
      .servo       = {.htim = &htim15, .channel = TIM_CHANNEL_2},
      .open_limit  = {.port = SWITCH_VALVE_NOS_OPENED_GPIO_Port, .pin = SWITCH_VALVE_NOS_OPENED_Pin},
      .close_limit = {.port = SWITCH_VALVE_NOS_CLOSED_GPIO_Port, .pin = SWITCH_VALVE_NOS_CLOSED_Pin},
      .max_transit_timeout_ms = 5000,
  });

  /* Bring up the engine controller (this also mounts the SD card via g_card.init()). The
     controller owns boot valve positioning: g_controller.init() resumes persistent_state and
     drives Control::init, which on a cold boot enters Safe (closing both valves) and on a
     reload re-executes the resumed state's transition (e.g. Launch OPENS both valves). So the
     board no longer pre-closes the valves here — doing so would fight a reload that must reopen
     them, and is redundant with the cold-boot Safe transition. */
  g_controller.init();

  /* Start the record-production timer last, so records only flow once the logic and
     SD are ready. The 2 kHz (0.5 ms) cadence is owned HERE: 100 MHz APB1 timer clock
     / (PSC 99 + 1) = 1 MHz tick; ARR 499 -> 0.5 ms = 2 kHz. Decoupled from the ADC's
     DRDY rate: each tick drains the ADC ring and produces one record per conversion. */
  __HAL_TIM_SET_PRESCALER(&htim6,  99);
  __HAL_TIM_SET_AUTORELOAD(&htim6, 499);
  htim6.Instance->EGR = TIM_EGR_UG;               // reload PSC/ARR now (no first-period glitch)
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
    ecu_app::g_controller.produceRecord(HAL_GetTick());
  }
}

/* DRDY (PA4) data-ready EXTI vector. PA4 is EXTI line 4 -> EXTI4_IRQHandler; it
   dispatches through the HAL to the ADC driver's HAL_GPIO_EXTI_Callback. */
extern "C" void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ADS_DRDY_Pin);   // GPIO_PIN_4
}

/* INA3221 I2C4 event + error vectors: forward to the HAL so the driver's non-blocking
   (interrupt-driven) reads complete. */
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
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */

/**
  * @brief Shared peripheral kernel-clock configuration (SPI1 + I2C4 off PLL3).
  *        Mirrors the CubeMX-generated PeriphCommonClock_Config() in main.c, which is
  *        excluded from the build — so halInit() calls this copy instead. Keep in sync
  *        with the .ioc / main.c on regen.
  * @retval None
  */
static void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C4 | RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 12;
  PeriphClkInitStruct.PLL3.PLL3Q = 12;
  PeriphClkInitStruct.PLL3.PLL3R = 4;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

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
