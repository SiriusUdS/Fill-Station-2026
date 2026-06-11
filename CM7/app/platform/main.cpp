/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body (C++ entry point)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "eth.h"
#include "fatfs.h"
#include "fdcan.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "communication/ethernet/ethernet.hpp"
#include "communication/can/can_dil.hpp"
#include "acquisition/adc/ads131m08.hpp"
#include "storage/sd_card.hpp"
#include "actuation/valve/ball_valve.hpp"
#include "memory/backup_ram.hpp"
#include "fcu_controller.hpp"
#include "dil/can_types.h"
#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"   // FILLING_STATION_BOARD_ID

namespace eth        = platform::communication::ethernet;
namespace can        = platform::communication::can;
namespace ads131m08  = platform::acquisition::adc::ads131m08;
namespace valve      = platform::actuation::valve;
namespace backup_ram = platform::memory::backup_ram;

/* The FCU's two local ball valves: Fill (servo on TIM1_CH1/PE9) and Dump (servo
   on TIM15_CH2/PE6), each with opened/closed limit switches on PE2-PE5. Brought
   up in fcuInit() and ticked every loop so their open/close state machines run. */
static valve::BallValve g_fill_valve;
static valve::BallValve g_dump_valve;

/* The board's single streaming ADC. Static lifetime: it backs the SPI CS arrays
   and is reached from the ADC ISRs while acquiring. */
static ads131m08::Ads131m08 g_ads131;

/* The board's single SD card and the FCU controller built over it. Both pinned in
   D1 AXI-SRAM: the controller's telemetry double buffer is handed straight to the
   SDMMC DMA, which cannot reach DTCM. g_card is declared (and so constructed)
   before g_controller, which holds a reference to it. */
__attribute__((section(".axisram"))) static platform::storage::SdCard g_card{&hsd2, "0:/"};
__attribute__((section(".axisram")))
static logic::fcu::Controller<platform::storage::SdCard, valve::BallValve, ads131m08::Ads131m08>
    g_controller{g_card, g_fill_valve, g_dump_valve, g_ads131};

/* Record-production timer (TIM6) ISR -> controller. TIM6 fires at a fixed cadence
   (set in fcuInit), decoupled from the ADC's DRDY rate: each tick drains every
   conversion the ADC has queued in its ring and emits one telemetry record per
   conversion (or a filler when the ADC is silent). */
extern "C" void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM6) {
    g_controller.produceRecord(HAL_GetTick());
  }
}

/* DRDY (PE7) data-ready EXTI vector. The handler symbol is fixed by the pin
   group (EXTI lines 5-9), so it is board-specific and lives here; it dispatches
   through the HAL to the ADC driver's HAL_GPIO_EXTI_Callback. */
extern "C" void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);

/* Application init — defined below main(), forward declared so main reads first. */
static void init(void);        /* full bring-up: HAL first, then our logic */
static void stmHalInit(void);  /* STM32 HAL + CubeMX peripheral init */
static void fcuInit(void);     /* our drivers (backup RAM, CAN, Ethernet) + FCU logic */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  init();

  for (;;)
  {
    const uint32_t now = HAL_GetTick();
    g_fill_valve.tick(now);   // advance each valve's open/close + limit-switch state machine
    g_dump_valve.tick(now);
    g_controller.tick(now);
  }
}

/**
  * @brief  Full system bring-up: STM32 HAL/peripherals first, then our logic.
  */
static void init(void)
{
  stmHalInit();
  fcuInit();
}

/**
  * @brief  STM32 HAL and CubeMX peripheral initialization (MPU, clocks,
  *         peripherals). Generated bring-up only — no application logic here.
  */
static void stmHalInit(void)
{
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
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

/**
  * @brief  Bring up our drivers and the FCU logic, on top of the HAL.
  */
static void fcuInit(void)
{
  /* Bring up the backup domain first, so the battery-backed Backup SRAM that
     holds the persistent state is clocked, writable and retained on VBAT before
     the FCU logic reads it. A regulator-timeout only means VBAT retention is
     unconfirmed; the SRAM is still accessible, so we proceed. */
  (void)backup_ram::init();

  /* Bring up the CAN and Ethernet drivers, then the FCU logic. The board now
     answers ARP and ICMP echo (ping) requests and exchanges UDP/CAN traffic
     through the logic interfaces. */
  (void)can::init(&hfdcan1, FILLING_STATION_BOARD_ID);
  eth::init();

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

  /* Safe boot state: drive both valves closed (no flow) before logic starts. */
  (void)g_fill_valve.close();
  (void)g_dump_valve.close();

  /* Bring up the FCU controller (this also mounts the SD card via g_card.init()). */
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
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
void PeriphCommonClock_Config(void)
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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
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
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
