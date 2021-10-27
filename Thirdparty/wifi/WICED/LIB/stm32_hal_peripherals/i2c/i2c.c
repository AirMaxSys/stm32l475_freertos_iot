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

/** @file
 * STM32 HAL based I2C implementation
 */
#include <stdint.h>
#include <string.h>
#include "platform.h"
#include "platform_config.h"
#include "platform_peripheral.h"
#include "wwd_rtos.h"
#include "wwd_assert.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define I2C_TRANSMIT_TIMEOUT ( 1000 )
#define I2C_RECEIVE_TIMEOUT  ( 1000 )
#define I2C_DMA_TIMEOUT_MS   ( 500 )

/* I2C bus frequency in Hz based on speed mode */
#define I2C_LOW_SPEED_MODE_FREQ_HZ      ( 10000 )
#define I2C_STANDARD_SPEED_MODE_FREQ_HZ ( 100000 )
#define I2C_HIGH_SPEED_MODE_FREQ_HZ     ( 400000 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static platform_result_t i2c_transmit_data( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint8_t* buffer, uint16_t length, uint32_t retries, uint32_t use_dma );
static platform_result_t i2c_receive_data( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint8_t* buffer, uint16_t length, uint32_t retries, uint32_t use_dma );

/******************************************************
 *               Variable Definitions
 ******************************************************/

extern const platform_i2c_clock_enable_function_t i2c_clock_enable_function[ ];
extern const platform_i2c_clock_disable_function_t i2c_clock_disable_function[ ];
extern const uint8_t i2c_alternate_functions[ ];
extern const IRQn_Type i2c_error_irq_vectors[ ];
extern const IRQn_Type i2c_event_irq_vectors[ ];
extern const platform_i2c_t platform_i2c_peripherals[ ];
extern platform_i2c_driver_t platform_i2c_driver[WICED_I2C_MAX];

/* Mapping of I2C port number to I2C_Handle */
static I2C_HandleTypeDef i2c_handles[ NUMBER_OF_I2C_PORTS ];
static DMA_HandleTypeDef dma_handles[ NUMBER_OF_I2C_PORTS * 2 ]; /* For Tx and Rx */

/******************************************************
 *               Function Definitions
 ******************************************************/

static platform_result_t platform_i2c_pin_init( const platform_i2c_t* i2c, FunctionalState state )
{
    uint8_t i2c_number;

    i2c_number = platform_i2c_get_port_number( i2c->port );

    if ( state == ENABLE )
    {
        if ( i2c->pin_scl != NULL )
        {
            platform_gpio_set_alternate_function( i2c->pin_scl->port, i2c->pin_scl->pin_number, GPIO_MODE_AF_OD, GPIO_NOPULL, i2c_alternate_functions[ i2c_number ] );
        }

        if ( i2c->pin_sda != NULL )
        {
            platform_gpio_set_alternate_function( i2c->pin_sda->port, i2c->pin_sda->pin_number, GPIO_MODE_AF_OD, GPIO_NOPULL, i2c_alternate_functions[ i2c_number ] );
        }
    }
    else
    {
        if ( i2c->pin_scl != NULL )
        {
            HAL_GPIO_DeInit( i2c->pin_scl->port, ( uint32_t )( 1 << i2c->pin_scl->pin_number ) );
        }

        if ( i2c->pin_sda != NULL )
        {
            HAL_GPIO_DeInit( i2c->pin_sda->port, ( uint32_t )( 1 << i2c->pin_sda->pin_number ) );
        }
    }

    return PLATFORM_SUCCESS;
}

platform_result_t platform_i2c_init( const platform_i2c_t* i2c, const platform_i2c_config_t* config )
{
    uint8_t i2c_number;
    I2C_HandleTypeDef* i2c_handle;
    DMA_HandleTypeDef* dma_handle_rx;
    DMA_HandleTypeDef* dma_handle_tx;
    platform_i2c_driver_t *i2c_driver;

    wiced_assert( "bad argument", ( i2c != NULL ) && ( config != NULL ) );

    platform_mcu_powersave_disable( );

    i2c_number      = platform_i2c_get_port_number( i2c->port );
    i2c_handle      = &i2c_handles[ i2c_number ];
    i2c_driver      = &platform_i2c_driver[ i2c_number ];
    i2c_driver->i2c = ( platform_i2c_t* )i2c;

    host_rtos_init_semaphore( &( i2c_driver->rw_complete ) );

    /* Enable I2C peripheral pins */
    platform_i2c_pin_init( i2c, ENABLE );

    /* Enable I2C peripheral clock */
    i2c_clock_enable_function[ i2c_number ]( );

    /* Initialize the I2C InitStruct */
    i2c_handle->Instance             = i2c->port;
    i2c_handle->Init.OwnAddress1     = 0x00;
    i2c_handle->Init.OwnAddress2     = 0x00;
    i2c_handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_handle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if ( config->speed_mode == I2C_LOW_SPEED_MODE )
    {
        i2c_handle->Init.Timing = platform_i2c_get_port_timing( i2c->port, I2C_LOW_SPEED_MODE_FREQ_HZ );
    }
    else if ( config->speed_mode == I2C_STANDARD_SPEED_MODE )
    {
        i2c_handle->Init.Timing = platform_i2c_get_port_timing( i2c->port, I2C_STANDARD_SPEED_MODE_FREQ_HZ );
    }
    else if ( config->speed_mode == I2C_HIGH_SPEED_MODE )
    {
        i2c_handle->Init.Timing = platform_i2c_get_port_timing( i2c->port, I2C_HIGH_SPEED_MODE_FREQ_HZ );
    }

    if ( config->address_width == I2C_ADDRESS_WIDTH_7BIT )
    {
        i2c_handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    }
    else if ( config->address_width == I2C_ADDRESS_WIDTH_10BIT )
    {
        i2c_handle->Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
    }

    /* Enable and initialize the I2C bus */
    if ( HAL_I2C_Init( i2c_handle ) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    /* Enable RX DMA */
    if ( i2c->rx_dma_config.controller != NULL )
    {
        if ( i2c->rx_dma_config.controller == DMA1 )
        {
            __HAL_RCC_DMA1_CLK_ENABLE( );
        }
        else
        {
            __HAL_RCC_DMA2_CLK_ENABLE( );
        }

        dma_handle_rx = &dma_handles[ i2c_number * 2 + 1 ];
        platform_i2c_rx_dma_init( dma_handle_rx, i2c_handle, i2c );
        NVIC_EnableIRQ( i2c->rx_dma_config.irq_vector );
    }

    /* Enable TX DMA */
    if ( i2c->tx_dma_config.controller != NULL )
    {
        if ( i2c->tx_dma_config.controller == DMA1 )
        {
            __HAL_RCC_DMA1_CLK_ENABLE( );
        }
        else
        {
            __HAL_RCC_DMA2_CLK_ENABLE( );
        }

        dma_handle_tx = &dma_handles[ i2c_number * 2 ];
        platform_i2c_tx_dma_init( dma_handle_tx, i2c_handle, i2c );
        NVIC_EnableIRQ( i2c->tx_dma_config.irq_vector );
    }

    NVIC_EnableIRQ( i2c_event_irq_vectors[ i2c_number ] );
    NVIC_EnableIRQ( i2c_error_irq_vectors[ i2c_number ] );

    platform_mcu_powersave_enable( );

    return PLATFORM_SUCCESS;
}

platform_result_t platform_i2c_deinit( const platform_i2c_t* i2c, const platform_i2c_config_t* config )
{
    uint8_t i2c_number;
    I2C_HandleTypeDef* i2c_handle;
    platform_i2c_driver_t* i2c_driver;

    wiced_assert( "bad argument", ( i2c != NULL ) && ( config != NULL ) );

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[ i2c_number ];
    i2c_driver = &platform_i2c_driver[ i2c_number ];

    host_rtos_deinit_semaphore( &( i2c_driver->rw_complete ) );

    platform_mcu_powersave_disable( );

    /* Disable RX DMA */
    if ( i2c->rx_dma_config.controller != NULL )
    {
        DMA_HandleTypeDef* dma_handle_rx;

        dma_handle_rx = &dma_handles[ i2c_number * 2 + 1 ];
        HAL_DMA_DeInit( dma_handle_rx );
        NVIC_DisableIRQ( i2c->rx_dma_config.irq_vector );
    }

    /* Disable TX DMA */
    if ( i2c->tx_dma_config.controller != NULL )
    {
        DMA_HandleTypeDef* dma_handle_tx;

        dma_handle_tx = &dma_handles[ i2c_number * 2 ];
        HAL_DMA_DeInit( dma_handle_tx );
        NVIC_DisableIRQ( i2c->tx_dma_config.irq_vector );
    }
    HAL_I2C_DeInit( i2c_handle );

    /* Disable I2C peripheral clock */
    i2c_clock_disable_function[ i2c_number ]( );

    /* Disable I2C peripheral pins */
    platform_i2c_pin_init( i2c, DISABLE );

    platform_mcu_powersave_enable( );

    return PLATFORM_SUCCESS;
}

static platform_result_t i2c_transmit_data( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint8_t* buffer, uint16_t length, uint32_t retries, uint32_t use_dma )
{
    uint16_t address;
    uint8_t i2c_number;
    I2C_HandleTypeDef* i2c_handle;
    HAL_StatusTypeDef hal_ret;
    platform_i2c_driver_t* i2c_driver;

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[ i2c_number ];
    i2c_driver = &platform_i2c_driver[ i2c_number ];

    address = config->address;

    if ( config->address_width == I2C_ADDRESS_WIDTH_7BIT )
    {
        address <<= 1;
    }

    do
    {
        i2c_driver->last_result = PLATFORM_ERROR;
        if ( use_dma && ( i2c->tx_dma_config.controller != NULL ) )
        {
            /* Assert if the buffer address and size is not cache-line aligned */

            /* Ensure buffer is aligned to PLATFORM_L1_CACHE_BYTES using
             * memalign - if dynamically allocating the buffer
             * DEFINE_CACHE_LINE_ALIGNED_ARRAY(type, name, size) - For array
             * For variables use ALIGNED(PLATFORM_L1_CACHE_BYTES)
             */
            wiced_assert("i2c transmit buffer is not cache line aligned", ( ( PLATFORM_L1_CACHE_LINE_MASK & (uint32_t)(buffer) ) == 0 ) );

            /* Flush cache contents to RAM */
            platform_dcache_clean_range( (const volatile void *)buffer, length );

            hal_ret = HAL_I2C_Master_Transmit_DMA( i2c_handle, address, buffer, length );
        }
        else
        {
            hal_ret = HAL_I2C_Master_Transmit_IT( i2c_handle, address, buffer, length );
        }

        if (HAL_OK == hal_ret)
        {
            /* Wait for transmission complete */
            if ( WWD_SUCCESS == host_rtos_get_semaphore( &( i2c_driver->rw_complete ), I2C_DMA_TIMEOUT_MS, WICED_TRUE ) )
            {
                if ( PLATFORM_SUCCESS == i2c_driver->last_result )
                {
                    return PLATFORM_SUCCESS;
                }
            }
        }

    } while ( retries-- != 0 );

    return PLATFORM_ERROR;

}

static platform_result_t i2c_receive_data( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint8_t* buffer, uint16_t length, uint32_t retries, uint32_t use_dma )
{
    uint16_t address;
    uint8_t i2c_number;
    I2C_HandleTypeDef* i2c_handle;
    HAL_StatusTypeDef hal_ret;
    platform_i2c_driver_t* i2c_driver;

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[ i2c_number ];
    i2c_driver = &platform_i2c_driver[ i2c_number ];


    address = config->address;

    if ( config->address_width == I2C_ADDRESS_WIDTH_7BIT )
    {
        address <<= 1;
    }

    do
    {
        i2c_driver->last_result = PLATFORM_ERROR;
        if ( use_dma && ( i2c->rx_dma_config.controller != NULL ) )
        {
            /* Assert if the buffer address and size is not cache-line aligned */

            /* Ensure buffer is aligned to PLATFORM_L1_CACHE_BYTES using
             * memalign - if dynamically allocating the buffer
             * DEFINE_CACHE_LINE_ALIGNED_ARRAY(type, name, size) - For array
             * For variables use ALIGNED(PLATFORM_L1_CACHE_BYTES)
             */
            wiced_assert("i2c receive buffer is not cache line aligned", ( ( PLATFORM_L1_CACHE_LINE_MASK & (uint32_t)(buffer) ) == 0 ) );

            /* Cache invalidate RX buffer */
            platform_dcache_inv_range( (volatile void *) ( buffer ), length );

            hal_ret = HAL_I2C_Master_Receive_DMA( i2c_handle, address, buffer, length );
        }
        else
        {
            hal_ret = HAL_I2C_Master_Receive_IT( i2c_handle, address, buffer, length );
        }

        if ( HAL_OK == hal_ret)
        {
            /* Wait for transmission complete */
            if ( WWD_SUCCESS == host_rtos_get_semaphore( &( i2c_driver->rw_complete ), I2C_DMA_TIMEOUT_MS, WICED_TRUE ) )
            {
                if ( PLATFORM_SUCCESS == i2c_driver->last_result )
                {
                    return PLATFORM_SUCCESS;
                }
            }
        }

    } while ( retries-- != 0 );

    return PLATFORM_ERROR;
}

platform_result_t platform_i2c_init_tx_message( platform_i2c_message_t* message, const void* tx_buffer, uint16_t tx_buffer_length, uint16_t retries, wiced_bool_t disable_dma )
{
    wiced_assert( "bad argument", ( message != NULL ) && ( tx_buffer != NULL ) && ( tx_buffer_length != 0 ) );

    if ( disable_dma == WICED_FALSE )
    {
        return PLATFORM_UNSUPPORTED;
    }

    memset( message, 0x00, sizeof( *message ) );

    message->tx_buffer = tx_buffer;
    message->retries   = retries;
    message->tx_length = tx_buffer_length;
    message->flags     = 0;

    return PLATFORM_SUCCESS;
}

platform_result_t platform_i2c_init_rx_message( platform_i2c_message_t* message, void* rx_buffer, uint16_t rx_buffer_length, uint16_t retries, wiced_bool_t disable_dma )
{
    wiced_assert( "bad argument", ( message != NULL ) && ( rx_buffer != NULL ) && ( rx_buffer_length != 0 ) );

    if ( disable_dma == WICED_FALSE )
    {
        return PLATFORM_UNSUPPORTED;
    }

    memset( message, 0x00, sizeof( *message ) );

    message->rx_buffer = rx_buffer;
    message->retries   = retries;
    message->rx_length = rx_buffer_length;
    message->flags     = 0;

    return PLATFORM_SUCCESS;
}

platform_result_t platform_i2c_init_combined_message( platform_i2c_message_t* message, const void* tx_buffer, void* rx_buffer, uint16_t tx_buffer_length, uint16_t rx_buffer_length, uint16_t retries, wiced_bool_t disable_dma )
{
    wiced_assert( "Not implemented", 0 );
    return PLATFORM_UNSUPPORTED;
}

wiced_bool_t platform_i2c_probe_device( const platform_i2c_t* i2c, const platform_i2c_config_t* config, int retries )
{
    uint8_t dummy[ 2 ];
    platform_result_t result;

    /* Read two bytes from the addressed-slave. The slave-address won't be
     * acknowledged if it isn't on the I2C bus. The read won't trigger
     * a NACK from the slave (unless of error), since only the receiver can do that.
     */
    result = i2c_receive_data( i2c, config, dummy, sizeof dummy, retries, 0 );

    return ( result == PLATFORM_SUCCESS ) ? WICED_TRUE : WICED_FALSE;
}

platform_result_t platform_i2c_transfer( const platform_i2c_t* i2c, const platform_i2c_config_t* config, platform_i2c_message_t* messages, uint16_t number_of_messages )
{
    platform_result_t result = PLATFORM_SUCCESS;
    uint32_t message_count;
    uint32_t use_dma = ( config->flags & I2C_DEVICE_NO_DMA ) ? 0 : 1;

    /* Check for message validity. Combo message is unsupported */
    for ( message_count = 0; message_count < number_of_messages; message_count++ )
    {
        if ( messages[ message_count ].rx_buffer != NULL && messages[ message_count ].tx_buffer != NULL )
        {
            return PLATFORM_UNSUPPORTED;
        }

        if ( ( messages[ message_count ].tx_buffer == NULL && messages[ message_count ].tx_length != 0 ) || ( messages[ message_count ].tx_buffer != NULL && messages[ message_count ].tx_length == 0 ) || ( messages[ message_count ].rx_buffer == NULL && messages[ message_count ].rx_length != 0 ) || ( messages[ message_count ].rx_buffer != NULL && messages[ message_count ].rx_length == 0 ) )
        {
            return PLATFORM_BADARG;
        }

        if ( messages[ message_count ].tx_buffer == NULL && messages[ message_count ].rx_buffer == NULL )
        {
            return PLATFORM_BADARG;
        }
    }

    platform_mcu_powersave_disable( );

    for ( message_count = 0; message_count < number_of_messages; message_count++ )
    {
        if ( messages[ message_count ].tx_length != 0 )
        {
            result = i2c_transmit_data( i2c, config, (uint8_t*) messages[ message_count ].tx_buffer, messages[ message_count ].tx_length, messages[ message_count ].retries, use_dma );

        }
        else if ( messages[ message_count ].rx_length != 0 )
        {

            result = i2c_receive_data( i2c, config, (uint8_t*) messages[ message_count ].rx_buffer, messages[ message_count ].rx_length, messages[ message_count ].retries, use_dma );
        }
    }

    platform_mcu_powersave_enable( );

    return result;
}

platform_result_t platform_i2c_write( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint16_t flags, const void* buffer, uint16_t buffer_length )
{
    platform_result_t result;

    result = i2c_transmit_data( i2c, config, (uint8_t*) buffer, buffer_length, 0, 0 );

    return result;
}

platform_result_t platform_i2c_read( const platform_i2c_t* i2c, const platform_i2c_config_t* config, uint16_t flags, void* buffer, uint16_t buffer_length )
{
    platform_result_t result;

    result = i2c_receive_data( i2c, config, (uint8_t*) buffer, buffer_length, 0, 0 );

    return result;
}

platform_i2c_driver_t* platform_i2c_get_driver( I2C_HandleTypeDef* i2c_handle )
{
    int i = 0;
    platform_i2c_driver_t* driver;

    for ( i = 0; i < WICED_I2C_MAX; i++ )
    {
        driver = &platform_i2c_driver[ i ];

        if ( driver->i2c->port == i2c_handle->Instance )
        {
            return driver;
        }
    }

    return NULL;
}

void platform_i2c_dma_tx_irq( const platform_i2c_t* i2c )
{
    I2C_HandleTypeDef* i2c_handle;
    uint8_t i2c_number;

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[ i2c_number ];

    HAL_DMA_IRQHandler( i2c_handle->hdmatx );
}

void platform_i2c_dma_rx_irq( const platform_i2c_t* i2c )
{
    I2C_HandleTypeDef* i2c_handle;
    uint8_t i2c_number;

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[ i2c_number ];

    HAL_DMA_IRQHandler( i2c_handle->hdmarx );
}

void platform_i2c_er_irq( const platform_i2c_t* i2c )
{
    I2C_HandleTypeDef* i2c_handle;
    uint8_t i2c_number;

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[ i2c_number ];

    HAL_I2C_EV_IRQHandler( i2c_handle );
}

void platform_i2c_ev_irq( const platform_i2c_t* i2c )
{
    I2C_HandleTypeDef* i2c_handle;
    uint8_t i2c_number;

    i2c_number = platform_i2c_get_port_number( i2c->port );
    i2c_handle = &i2c_handles[i2c_number];

    HAL_I2C_EV_IRQHandler( i2c_handle );
}

void HAL_I2C_MasterTxCpltCallback( I2C_HandleTypeDef* i2c_handle )
{
    platform_i2c_driver_t* i2c_driver = NULL;

    i2c_driver = platform_i2c_get_driver( i2c_handle );
    if ( ( i2c_driver != NULL ) && ( i2c_driver->i2c != NULL ) )
    {
        i2c_driver->last_result = PLATFORM_SUCCESS;
        host_rtos_set_semaphore( &( i2c_driver->rw_complete ), WICED_TRUE );
    }
}

void HAL_I2C_MasterRxCpltCallback( I2C_HandleTypeDef* i2c_handle )
{
    platform_i2c_driver_t* i2c_driver = NULL;

    i2c_driver = platform_i2c_get_driver( i2c_handle );

    if ( ( i2c_driver != NULL ) && ( i2c_driver->i2c != NULL ) )
    {
        i2c_driver->last_result = PLATFORM_SUCCESS;
        host_rtos_set_semaphore( &( i2c_driver->rw_complete ), WICED_TRUE );
    }
}

void HAL_I2C_ErrorCallback( I2C_HandleTypeDef* i2c_handle )
{
    platform_i2c_driver_t* i2c_driver = NULL;

    i2c_driver = platform_i2c_get_driver( i2c_handle );


    if ( ( i2c_driver != NULL ) && ( i2c_driver->i2c != NULL ) )
    {
        i2c_driver->last_result = PLATFORM_ERROR;
        host_rtos_set_semaphore( &( i2c_driver->rw_complete ), WICED_TRUE );
    }
}
