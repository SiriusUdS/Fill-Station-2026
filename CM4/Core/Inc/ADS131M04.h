/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADS131M04_H
#define __ADS131M04_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define MSB_uint16_mask 0xFF00
#define LSB_uint16_mask 0xFF

#define WREG_CMD 0b011
#define RREG_CMD 0b101
#define NULL_CMD 0

void ADS131M04_write_register(uint8_t address, uint16_t value, SPI_HandleTypeDef *hspi);
void ADS131M04_get_data(SPI_HandleTypeDef *hspi);


// ---------------------------
// ADS131M04 Register Addresses
// ---------------------------

// DEVICE SETTINGS AND INDICATORS
#define REG_ADDR_ID             0x00
#define REG_ADDR_STATUS         0x01

// GLOBAL SETTINGS ACROSS CHANNELS
#define REG_ADDR_MODE           0x02
#define REG_ADDR_CLOCK          0x03
#define REG_ADDR_GAIN           0x04
//0x05 is reserved
#define REG_ADDR_CFG            0x06
#define REG_ADDR_THRSHLD_MSB    0x07
#define REG_ADDR_THRSHLD_LSB    0x08

// CHANNEL 0
#define REG_ADDR_CH0_CFG        0x09
#define REG_ADDR_CH0_OCAL_MSB   0x0A
#define REG_ADDR_CH0_OCAL_LSB   0x0B
#define REG_ADDR_CH0_GCAL_MSB   0x0C
#define REG_ADDR_CH0_GCAL_LSB   0x0D

// CHANNEL 1
#define REG_ADDR_CH1_CFG        0x0E
#define REG_ADDR_CH1_OCAL_MSB   0x0F
#define REG_ADDR_CH1_OCAL_LSB   0x10
#define REG_ADDR_CH1_GCAL_MSB   0x11
#define REG_ADDR_CH1_GCAL_LSB   0x12

// CHANNEL 2
#define REG_ADDR_CH2_CFG        0x13
#define REG_ADDR_CH2_OCAL_MSB   0x14
#define REG_ADDR_CH2_OCAL_LSB   0x15
#define REG_ADDR_CH2_GCAL_MSB   0x16
#define REG_ADDR_CH2_GCAL_LSB   0x17

// CHANNEL 3
#define REG_ADDR_CH3_CFG        0x18
#define REG_ADDR_CH3_OCAL_MSB   0x19
#define REG_ADDR_CH3_OCAL_LSB   0x1A
#define REG_ADDR_CH3_GCAL_MSB   0x1B
#define REG_ADDR_CH3_GCAL_LSB   0x1C

// REGISTER MAP CRC AND RESERVED
#define REG_ADDR_REGMAP_CRC     0x3E
#define REG_ADDR_RESERVED       0x3F



// ---------------------------
// 0x02 MODE register
// ---------------------------
typedef union {
    uint16_t all;
    struct {
        uint16_t DRDY_FMT   : 1;  // Bit 0
        uint16_t DRDY_HiZ   : 1;  // Bit 1
        uint16_t DRDY_SEL   : 2;  // Bits 3:2
        uint16_t TIMEOUT    : 1;  // Bit 4
        uint16_t RESERVED0  : 3;  // Bits 7:5
        uint16_t WLENGTH    : 2;  // Bits 9:8
        uint16_t RESET      : 1;  // Bit 10
        uint16_t CRC_TYPE   : 1;  // Bit 11
        uint16_t RX_CRC_EN  : 1;  // Bit 12
        uint16_t REG_CRC_EN : 1;  // Bit 13
        uint16_t RESERVED1  : 2;  // Bits 15:14
    } bits;
} REG_MODE_t;

// ---------------------------
// 0x03 CLOCK register
// ---------------------------
typedef union {
    uint16_t all;
    struct {
        uint16_t PWR_       : 2;  // Bits 1:0
        uint16_t OSR        : 3;  // Bits 4:2
        uint16_t TBM        : 1;  // Bit 5
        uint16_t RESERVED0  : 2;  // Bits 7:6
        uint16_t CH0_EN     : 1;  // Bit 8
        uint16_t CH1_EN     : 1;  // Bit 9
        uint16_t CH2_EN     : 1;  // Bit 10
        uint16_t CH3_EN     : 1;  // Bit 11
        uint16_t RESERVED1  : 4;  // Bits 15:12
    } bits;
} REG_CLOCK_t;

// ---------------------------
// 0x04 GAIN register
// ---------------------------
typedef union {
    uint16_t all;
    struct {
        uint16_t PGAGAIN0   : 3;  // Bits 2:0
        uint16_t RESERVED0  : 1;  // Bit 3
        uint16_t PGAGAIN1   : 3;  // Bits 6:4
        uint16_t RESERVED1  : 1;  // Bit 7
        uint16_t PGAGAIN2   : 3;  // Bits 10:8
        uint16_t RESERVED2  : 1;  // Bit 11
        uint16_t PGAGAIN3   : 3;  // Bits 14:12
        uint16_t RESERVED3  : 1;  // Bit 15
    } bits;
} REG_GAIN_t;

// ---------------------------
// 0x05 GAIN register
// ---------------------------

typedef union {
    uint16_t all;
    struct {
        uint16_t PGAGAIN4   : 3;  // Bits 2:0
        uint16_t RESERVED0  : 1;  // Bit 3
        uint16_t PGAGAIN5   : 3;  // Bits 6:4
        uint16_t RESERVED1  : 1;  // Bit 7
        uint16_t PGAGAIN6   : 3;  // Bits 10:8
        uint16_t RESERVED2  : 1;  // Bit 11
        uint16_t PGAGAIN7   : 3;  // Bits 14:12
        uint16_t RESERVED3  : 1;  // Bit 15
    } bits;
} REG_GAIN2_t;




// ---------------------------
// 0x06 CFG register
// ---------------------------
typedef union {
    uint16_t all;
    struct {
        uint16_t CD_EN      : 1;  // Bit 0
        uint16_t CD_LEN     : 3;  // Bits 3:1
        uint16_t CD_NUM     : 3;  // Bits 6:4
        uint16_t CD_ALLCH   : 1;  // Bit 7
        uint16_t GC_EN      : 1;  // Bit 8
        uint16_t GC_DLY     : 4;  // Bits 12:9
        uint16_t RESERVED   : 3;  // Bits 15:13
    } bits;
} REG_CFG_t;

// ---------------------------
// 0x07 THRSHLD_MSB register
// ---------------------------
typedef union {
    uint16_t all;
    struct {
        uint16_t CD_TH_MSB  : 16; // Bits 15:0
    } bits;
} REG_THRSHLD_MSB_t;

// ---------------------------
// 0x08 THRSHLD_LSB register
// ---------------------------
typedef union {
    uint16_t all;
    struct {
        uint16_t DCBLOCK    : 4;  // Bits 3:0
        uint16_t RESERVED0  : 4;  // Bits 7:4
        uint16_t CD_TH_LSB  : 8;  // Bits 15:8
    } bits;
} REG_THRSHLD_LSB_t;

/* ============================================================
 * ADS131M04 CHANNEL REGISTER BITFIELD STRUCTURES
 * ============================================================ */

//Channel 0

/* ===================== CH0_CFG (0x09) ===================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t MUX0        : 2;   // Bits 1:0
        uint16_t DCBLK0_DIS  : 1;   // Bit 2
        uint16_t RESERVED    : 3;   // Bits 5:3 (write 000)
        uint16_t PHASE0      : 10;  // Bits 15:6 (two's complement)
    } bit;

} ADS131_CH0_CFG_t;


/* ================== CH0_OCAL_MSB (0x0A) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t OCAL0_MSB : 16;   // Bits 23:8 of offset calibration
    } bit;

} ADS131_CH0_OCAL_MSB_t;


/* ================== CH0_OCAL_LSB (0x0B) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t RESERVED    : 8;   // Bits 7:0 (always 0)
        uint16_t OCAL0_LSB   : 8;   // Bits 15:8
    } bit;

} ADS131_CH0_OCAL_LSB_t;


/* ================== CH0_GCAL_MSB (0x0C) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t GCAL0_MSB : 16;   // Bits 23:8 of gain calibration
    } bit;

} ADS131_CH0_GCAL_MSB_t;


/* ================== CH0_GCAL_LSB (0x0D) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t RESERVED    : 8;   // Bits 7:0 (always 0)
        uint16_t GCAL0_LSB   : 8;   // Bits 15:8
    } bit;

} ADS131_CH0_GCAL_LSB_t;

//Channel 1

/* ===================== CH1_CFG (0x0E) ===================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t MUX1        : 2;   // Bits 1:0
        uint16_t DCBLK1_DIS  : 1;   // Bit 2
        uint16_t RESERVED    : 3;   // Bits 5:3
        uint16_t PHASE1      : 10;  // Bits 15:6
    } bit;

} ADS131_CH1_CFG_t;


/* ================== CH1_OCAL_MSB (0x0F) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t OCAL1_MSB : 16;   // Bits 23:8
    } bit;

} ADS131_CH1_OCAL_MSB_t;


/* ================== CH1_OCAL_LSB (0x10) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t RESERVED    : 8;   // Bits 7:0
        uint16_t OCAL1_LSB   : 8;   // Bits 15:8
    } bit;

} ADS131_CH1_OCAL_LSB_t;

/* ================== CH1_GCAL_MSB (0x11) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t GCAL1_MSB : 16;   // Bits 15:0
                                   // Gain calibration bits [23:8]
                                   // Reset = 0x8000
    } bit;

} ADS131_CH1_GCAL_MSB_t;


/* ================== CH1_GCAL_LSB (0x12) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t RESERVED    : 8;  // Bits 7:0 (always reads 0)
        uint16_t GCAL1_LSB   : 8;  // Bits 15:8
                                   // Gain calibration bits [7:0]
    } bit;

} ADS131_CH1_GCAL_LSB_t;

//Channel 2

/* ===================== CH2_CFG (0x13) ===================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t MUX2        : 2;   // Bits 1:0
        uint16_t DCBLK2_DIS  : 1;   // Bit 2
        uint16_t RESERVED    : 3;   // Bits 5:3 (always 000)
        uint16_t PHASE2      : 10;  // Bits 15:6 (two's complement)
    } bit;

} ADS131_CH2_CFG_t;


/* ================== CH2_OCAL_MSB (0x14) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t OCAL2_MSB : 16;   // Bits 15:0
                                   // Offset calibration bits [23:8]
    } bit;

} ADS131_CH2_OCAL_MSB_t;


/* ================== CH2_OCAL_LSB (0x15) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t RESERVED   : 8;   // Bits 7:0 (always reads 0)
        uint16_t OCAL2_LSB  : 8;   // Bits 15:8
                                   // Offset calibration bits [7:0]
    } bit;

} ADS131_CH2_OCAL_LSB_t;


/* ================== CH2_GCAL_MSB (0x16) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t GCAL2_MSB : 16;   // Bits 15:0
                                   // Gain calibration bits [23:8]
                                   // Reset = 0x8000
    } bit;

} ADS131_CH2_GCAL_MSB_t;


/* ================== CH2_GCAL_LSB (0x17) =================== */
typedef union
{
    uint16_t reg;

    struct
    {
        uint16_t RESERVED   : 8;   // Bits 7:0 (always reads 0)
        uint16_t GCAL2_LSB  : 8;   // Bits 15:8
                                   // Gain calibration bits [7:0]
    } bit;

} ADS131_CH2_GCAL_LSB_t;

//Channel 3

// ---------------------------
// Channel 3 Configuration Register (CH3_CFG, Address 0x18)
// ---------------------------

typedef union {
    uint16_t all; // Full 16-bit access
    struct {
        uint16_t MUX3      : 2;  // Bits 1:0 - Input selection
                                   // 00 = AIN3P/AIN3N
                                   // 01 = ADC inputs shorted
                                   // 10 = Positive DC test signal
                                   // 11 = Negative DC test signal
        uint16_t DCBLK3_DIS0 : 1; // Bit 2 - DC block filter disable
                                   // 0 = Controlled by DCBLOCK[3:0] (default)
                                   // 1 = Disabled for this channel
        uint16_t RESERVED     : 3; // Bits 5:3 - Reserved, always reads 000
        int16_t PHASE3        : 10; // Bits 15:6 - Phase delay in modulator clock cycles (two's complement)
    } bits;
} CH3_CFG_Reg;

// ---------------------------
// Channel 3 Offset Calibration
// ---------------------------

// MSB register (0x19)
typedef union {
    uint16_t all; // full 16-bit access
    struct {
        uint16_t OCAL3_MSB : 8;  // bits 15:8 - offset MSB
        uint16_t unused_MSB : 8; // bits 7:0 (not used for calibration)
    } bits;
} CH3_OCAL_MSB_Reg;

// LSB register (0x1A)
typedef union {
    uint16_t all;
    struct {
        uint16_t OCAL3_LSB : 8;  // bits 15:8 - offset LSB
        uint16_t RESERVED  : 8;  // bits 7:0 - always reads 0
    } bits;
} CH3_OCAL_LSB_Reg;

// ---------------------------
// Channel 3 Gain Calibration
// ---------------------------

// MSB register (0x1B)
typedef union {
    uint16_t all;
    struct {
        uint16_t GCAL3_MSB : 16;  // bits 15:8 - gain MSB
    } bits;
} CH3_GCAL_MSB_Reg;

// LSB register (0x1C)
typedef union {
    uint16_t all;
    struct {
        uint16_t GCAL3_LSB : 8;  // bits 15:8 - gain LSB
        uint16_t unused_LSB : 8; // bits 7:0 (not used for calibration)
    } bits;
} CH3_GCAL_LSB_Reg;





#endif /* __ADS131M04_H */
