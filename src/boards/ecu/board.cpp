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
#include "sdmmc.h"
#include "fatfs.h"
#include "tim.h"

/* The ECU object graph (defined in main.cpp) + the bits wireDrivers() needs. */
#include "ecu_objects.hpp"
#include "memory/backup_ram.hpp"
#include "framing/can_header.hpp"
#include "system/board_id.hpp"   // BoardId::Engine

using namespace ecu_app;
namespace backup_ram = platform::memory::backup_ram;

static void SystemClock_Config(void);
static void MPU_Config(void);

namespace board {

void halInit(void)
{
  /* MPU + HAL + system clock (the ECU uses default kernel clocks — no
     PeriphCommonClock_Config), then every CubeMX-configured peripheral. */
  MPU_Config();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CRC_Init();
  MX_FDCAN1_Init();
  MX_DMA_Init();         // must precede MX_SPI1_Init: SPI1 MspInit calls HAL_DMA_Init + arms the stream NVICs
  MX_SPI1_Init();        // ADS131M08 ADC bus
  MX_SDMMC1_SD_Init();   // SD card
  MX_FATFS_Init();
  MX_TIM15_Init();       // valve servo PWM (CH1=IPA/PE5, CH2=NOS/PE6)
  MX_TIM6_Init();        // record-production cadence (unused until the controller lands)
}

void wireDrivers(void)
{
  /* Bring up the backup domain first so the battery-backed Backup SRAM that holds
     the persistent state is clocked + writable before the controller reads it. */
  (void)backup_ram::init();

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

  /* Bind the three SD log files on SDMMC1 (the app composition left them unbound). They
     share one card: g_controller.init() mounts the volume + creates this boot's session
     folder once, then opens all three files inside it (same recording policy as the FCU). */
  g_card_fast.bind(&hsd1, "0:/", "data_fast.bin");
  g_card_slow.bind(&hsd1, "0:/", "data_slow.bin");
  g_card_ext.bind(&hsd1, "0:/", "data_ext.bin");

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

  /* Safe boot state: drive both valves closed (no flow). */
  (void)g_ipa_valve.close();
  (void)g_nos_valve.close();

  /* Bring up the engine controller (this also mounts the SD card via g_card.init()). */
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
