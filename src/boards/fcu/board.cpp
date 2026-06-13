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
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "crc.h"

/* The FCU object graph (defined in main.cpp) + the bits wireDrivers() needs. */
#include "fcu_objects.hpp"
#include "memory/backup_ram.hpp"
#include "data_integrity/crc/crc.hpp"
#include "framing/can_header.hpp"
#include "system/board_id.hpp"   // BoardId::FillingStation

using namespace fcu_app;
namespace backup_ram = platform::memory::backup_ram;
namespace crc        = platform::data_integrity::crc;

static void SystemClock_Config(void);
static void PeriphCommonClock_Config(void);
static void MPU_Config(void);

namespace board {

void halInit(void)
{
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
  MX_SDMMC2_SD_Init();
  MX_FATFS_Init();
  MX_FDCAN1_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_TIM1_Init();   // Fill valve servo PWM (PE9 / TIM1_CH1)
  MX_TIM15_Init();  // Dump valve servo PWM (PE6 / TIM15_CH2)
  MX_TIM6_Init();   // record-production cadence (drives g_controller.produceRecord)
}

void wireDrivers(void)
{
  /* Bring up the backup domain first, so the battery-backed Backup SRAM that
     holds the persistent state is clocked, writable and retained on VBAT before
     the FCU logic reads it. A regulator-timeout only means VBAT retention is
     unconfirmed; the SRAM is still accessible, so we proceed. */
  (void)backup_ram::init();

  /* Configure the CRC peripheral for the zlib/reflected variant behind the
     logic data-integrity seam (logic::data_integrity::crc32), used to stamp the
     telemetry frame CRC. Must precede the first downlink (drainTick in tick()). */
  crc::init(&hcrc);

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

  g_ads131.init({
      .hspi      = &hspi4,
      .cs_port   = GPIOE,
      .cs_pin    = GPIO_PIN_15,
      .drdy_pin  = GPIO_PIN_7,
      .drdy_irqn = EXTI9_5_IRQn,
  });
  g_ads131.start();

  /* Bind the SD card to its HAL handle + FatFs drive (the app composition left it
     unbound). It is mounted later by g_controller.init(). */
  g_card.bind(&hsd2, "0:/");

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
      .servo       = {.htim = &htim1, .channel = TIM_CHANNEL_1,
                      .min_pulse_ticks = 1000.0F, .max_pulse_ticks = 2000.0F},
      .open_limit  = {.port = FILL_SWITCH_OPENED_GPIO_Port, .pin = FILL_SWITCH_OPENED_Pin},
      .close_limit = {.port = FILL_SWITCH_CLOSED_GPIO_Port, .pin = FILL_SWITCH_CLOSED_Pin},
      .max_transit_timeout_ms = 5000,
  });
  g_dump_valve.init({
      .servo       = {.htim = &htim15, .channel = TIM_CHANNEL_2,
                      .min_pulse_ticks = 1000.0F, .max_pulse_ticks = 2000.0F},
      // No physical open switch on the dump valve (no DUMP_SWITCH_OPENED pin): the
      // descriptor is null and tick() never reads it (opened_switch_ignored).
      .open_limit  = {.port = nullptr, .pin = 0},
      .close_limit = {.port = DUMP_SWITCH_CLOSED_GPIO_Port, .pin = DUMP_SWITCH_CLOSED_Pin},
      .max_transit_timeout_ms = 5000,
      .opened_switch_ignored  = true,
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
