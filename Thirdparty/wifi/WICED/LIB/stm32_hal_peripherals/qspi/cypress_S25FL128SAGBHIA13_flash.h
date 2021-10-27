/*
 * Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#ifndef __CYPRESS_S25FL128SAGBHIA13_FLASH_H
#define __CYPRESS_S25FL128SAGBHIA13_FLASH_H

#include "spi_flash.h"
#include "platform_peripheral.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef DEBUG
#define DEBUG_PRINTF_S25FL(x) printf x
#else
#define DEBUG_PRINTF_S25FL(x)
#endif /* ifdef DEBUG */

/* S25FL128SAGBHIA13 Cypress memory */

/* Size of the flash */
#define KBYTE   ( 1024 )
#define MBYTE   ( 1024*KBYTE )
#define SFLASH_SIZE_16MBYTE ( 16 * MBYTE )

#define SFLASH_MANUFACTURER_ID               0x01
#define SFLASH_DEVICE_ID_MSB                 0x20
#define SFLASH_DEVICE_ID_LSB                 0x18

#define QSPI_FLASH_SIZE                      23       /* 2 ^ (23+1) = 16MB */
#define QSPI_PAGE_SIZE                       512
#define QSPI_SECTOR_SIZE                     ( 256 * KBYTE )

/* Reset Operations */
#define SW_RESET                             0xF0

/* Identification Operations */
#define READ_ID_CMD                          0x9F

/* Read Operations */
#define READ_CMD                             0x03
#define READ_4_BYTE_ADDR_CMD                 0x13

#define FAST_READ_CMD                        0x0B
#define FAST_READ_4_BYTE_ADDR_CMD            0x0C

#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x3C

#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define DUAL_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xBC

#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x6C

#define QUAD_INOUT_FAST_READ_CMD             0xEB
#define QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xEC

#define READ_CONFIG_REG_CMD                  0x35

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define PAGE_PROG_4_BYTE_ADDR_CMD            0x12

#define QUAD_IN_FAST_PROG_CMD                0x32
#define QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD    0x34

/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x20
#define SUBSECTOR_ERASE_4_BYTE_ADDR_CMD      0x21

#define SECTOR_ERASE_CMD                     0xD8
#define SECTOR_ERASE_4_BYTE_ADDR_CMD         0xDC

#define BULK_ERASE_CMD                       0xC7
#define BULK_ERASE_TIME_MS                   ( 35 * 1000 )

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_READ              8
#define DUMMY_CLOCK_CYCLES_READ_QUAD         10

/* End address of the QSPI memory */
#define QSPI_END_ADDR                        ( 1 << QSPI_FLASH_SIZE )
/* Cypress FLash Quad Mode bit in COnfiguration register */
#define QUAD_MODE_BIT                        ( 1 << 1 )
#define QUAD_MODE_BIT_MASK                   ( 0xFF ^ QUAD_MODE_BIT )
/*
 * Cypress Flash specific functions
 */

typedef enum {
   HAL_CR1_QSPI_NO_ERR  = 0,
   HAL_CR1_QSPI_CMD_ERR = 1,
   HAL_CR1_QSPI_RCV_ERR = 2,
   HAL_CR1_QSPI_XMT_ERR = 3,
   HAL_CR1_QSPI_WRT_ERR = 4,
} platform_hal_qspi_err_t;

/* Function Prototypes */

extern int sflash_write_enable(QSPI_HandleTypeDef *hqspi);

/* static inline function specifically for Cypress Flash Quad mode configuration */
/**
  * @brief  This function sets configuration register to Quad mode
  * @param  hqspi: QSPI handle
  * @retval error code
  */
static inline int QSPI_set_Quadmode( QSPI_HandleTypeDef *hqspi )
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg_cfg;
  uint8_t reg_status;
  uint8_t reg2[3] = {'\0'};

  /* Read Volatile Configuration register --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_CONFIG_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if (HAL_QSPI_Command( hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return ( HAL_CR1_QSPI_CMD_ERR );
  }

  if ( HAL_QSPI_Receive( hqspi, &reg_cfg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return ( HAL_CR1_QSPI_RCV_ERR );
  }

  if ( reg_cfg & QUAD_MODE_BIT ) /* already in Quad mode */
  {
#if !defined(BOOTLOADER)
    /* DEBUG_PRINTF_S25FL( (" Already in Quad mode!!\n") ); */
#endif /* if !defined(BOOTLOADER) */
    return ( HAL_CR1_QSPI_NO_ERR );
  }

  sCommand.Instruction       = READ_STATUS_REG_CMD;
  if ( HAL_QSPI_Command( hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return ( HAL_CR1_QSPI_CMD_ERR );
  }

  if ( HAL_QSPI_Receive( hqspi, &reg_status, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return ( HAL_CR1_QSPI_RCV_ERR );
  }

  /* Enable write operations ---------------------------------------- */
  sflash_write_enable(hqspi);

  /* Write Volatile Configuration register (with new dummy cycles) -- */
  sCommand.Instruction = WRITE_STATUS_REG_CMD;
  reg2[0] = ( uint8_t ) ( reg_status << 8 );
  reg_cfg = reg_cfg & QUAD_MODE_BIT_MASK;
  reg_cfg = reg_cfg | QUAD_MODE_BIT;
  reg2[1] |= ( uint8_t ) ( reg_cfg );
#if !defined(BOOTLOADER)
  DEBUG_PRINTF_S25FL(( "QUAD_MODE_BIT_MASK: 0x%02X, QUAD_MODE_BIT : 0x%02X\n", QUAD_MODE_BIT_MASK, QUAD_MODE_BIT ));
  DEBUG_PRINTF_S25FL(( "reg_status: 0x%02X, reg_cfg : 0x%02X\n", reg_status, reg_cfg ));
#endif /* if !defined(BOOTLOADER) */
  sCommand.NbData            = 2;

  if ( HAL_QSPI_Command( hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return ( HAL_CR1_QSPI_WRT_ERR );
  }

  if ( HAL_QSPI_Transmit( hqspi, reg2, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return ( HAL_CR1_QSPI_XMT_ERR );
  }

  /* Verify written Value */
  sCommand.Instruction       = READ_CONFIG_REG_CMD;
  sCommand.NbData            = 1;
  if ( HAL_QSPI_Command( hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return ( HAL_CR1_QSPI_CMD_ERR );
  }

  if ( HAL_QSPI_Receive( hqspi, &reg_cfg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK )
  {
    return ( HAL_CR1_QSPI_RCV_ERR );
  }
  return ( HAL_CR1_QSPI_NO_ERR );
}
#endif /* __CYPRESS_S25FL128SAGBHIA13_FLASH_H */
