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
 * STM32 HAL based UART implementation
 */
#include <stdint.h>
#include <string.h>
#include "platform.h"
#include "platform_config.h"
#include "platform_peripheral.h"
#include "platform_sleep.h"
#include "platform_assert.h"
#include "wwd_assert.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define UART_TX_MAX_WAIT_TIME (1000)

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

static platform_result_t platform_uart_pin_init( platform_uart_driver_t* driver, FunctionalState state );
static platform_result_t init_circular_dma( platform_uart_driver_t* driver, void* data, uint32_t size, uint32_t timeout );
static platform_result_t get_dma_rx_data_from_circular_buffer( platform_uart_driver_t* driver, uint8_t* data_in, uint32_t* expected_data_size, uint32_t timeout_ms );

/******************************************************
 *               Variable Definitions
 ******************************************************/

extern const platform_uart_clock_enable_function_t uart_clock_enable_function[ ];
extern const platform_uart_clock_disable_function_t uart_clock_disable_function[ ];
extern const uint8_t uart_alternate_functions[ ];
extern const IRQn_Type uart_irq_vectors[ ];
extern platform_uart_driver_t platform_uart_drivers[ WICED_UART_MAX ];

/* Mapping of UART port number to UART_Handle */
static UART_HandleTypeDef uart_handles[ NUMBER_OF_UART_PORTS ];
static DMA_HandleTypeDef dma_handles[ NUMBER_OF_UART_PORTS * 2 ]; /* For Tx and Rx */

/******************************************************
 *               Function Definitions
 ******************************************************/

platform_result_t platform_uart_init( platform_uart_driver_t* driver, const platform_uart_t* peripheral, const platform_uart_config_t* config, wiced_ring_buffer_t* optional_ring_buffer )
{
    UART_HandleTypeDef* uart_handle;
    DMA_HandleTypeDef* dma_handle_rx;
    DMA_HandleTypeDef* dma_handle_tx;
    uint32_t uart_number;

    wiced_assert( "bad argument", ( driver != NULL ) && ( peripheral != NULL ) && ( config != NULL ) );
    wiced_assert( "Bad ring buffer", (optional_ring_buffer == NULL) || ((optional_ring_buffer->buffer != NULL ) && (optional_ring_buffer->size != 0)) );
#ifdef PLATFORM_L1_CACHE_SHIFT
    wiced_assert("Ring buffer is not cache line aligned", ( ( PLATFORM_L1_CACHE_LINE_MASK & (uint32_t)(optional_ring_buffer->buffer) ) == 0 ) );
#endif

    platform_mcu_powersave_disable( );

    uart_number     = platform_uart_get_port_number( peripheral->port );
    uart_handle     = &uart_handles[ uart_number ];
    driver->rx_size = 0;
    driver->tx_size = 0;
    driver->last_transmit_result = PLATFORM_SUCCESS;
    driver->last_receive_result  = PLATFORM_SUCCESS;
    driver->peripheral = (platform_uart_t*) peripheral;
    host_rtos_init_semaphore( &driver->tx_complete );
    host_rtos_init_semaphore( &driver->rx_complete );

    /* Enable UART peripheral pins */
    platform_uart_pin_init( driver, ENABLE );

    /* Enable UART peripheral clock */
    uart_clock_enable_function[ uart_number ]( );

    /* Initialise USART peripheral */
    uart_handle->Instance        = peripheral->port;
    uart_handle->Init.BaudRate   = config->baud_rate;
    uart_handle->Init.Mode       = UART_MODE_TX_RX;
    uart_handle->Init.WordLength = ( ( config->data_width == DATA_WIDTH_9BIT ) || ( ( config->data_width == DATA_WIDTH_8BIT ) && ( config->parity != NO_PARITY ) ) ) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    uart_handle->Init.StopBits   = ( config->stop_bits == STOP_BITS_1 ) ? UART_STOPBITS_1 : UART_STOPBITS_2;

    switch ( config->parity )
    {
        case NO_PARITY:
            uart_handle->Init.Parity = UART_PARITY_NONE;
            break;

        case EVEN_PARITY:
            uart_handle->Init.Parity = UART_PARITY_EVEN;
            break;

        case ODD_PARITY:
            uart_handle->Init.Parity = UART_PARITY_ODD;
            break;

        default:
            return PLATFORM_BADARG;
    }

    switch ( config->flow_control )
    {
        case FLOW_CONTROL_DISABLED:
            uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
            break;

        case FLOW_CONTROL_CTS:
            uart_handle->Init.HwFlowCtl = UART_HWCONTROL_CTS;
            break;

        case FLOW_CONTROL_RTS:
            uart_handle->Init.HwFlowCtl = UART_HWCONTROL_RTS;
            break;

        case FLOW_CONTROL_CTS_RTS:
            uart_handle->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
            break;

        default:
            return PLATFORM_BADARG;
    }

    if ( HAL_UART_Init( uart_handle ) != HAL_OK )
    {
        return PLATFORM_ERROR;
    }

    if ( peripheral->rx_dma_config.controller != NULL )
    {
        if ( peripheral->rx_dma_config.controller == DMA1 )
        {
            __HAL_RCC_DMA1_CLK_ENABLE( );
        }
        else
        {
            __HAL_RCC_DMA2_CLK_ENABLE( );
        }

        dma_handle_rx = &dma_handles[ uart_number * 2 + 1 ];
        platform_uart_rx_dma_init( dma_handle_rx, uart_handle, peripheral );

        /* Setup ring buffer */
        if ( optional_ring_buffer != NULL )
        {
            /* Note that the ring_buffer should've been initialised first */
            driver->rx_buffer = optional_ring_buffer;
            driver->rx_size   = 0;
            /* initialize circular rx dma
             * the dma continues to run even after this function returns,
             * target is auto-incremented, and size is re-used because of
             * dma_handle_rx->Init.Mode = DMA_CIRCULAR
             * and
             * dma_handle_rx->Init.MemInc = DMA_MINC_ENABLE
             */
            init_circular_dma( driver, optional_ring_buffer->buffer, optional_ring_buffer->size, 0 );
        }

        NVIC_EnableIRQ( peripheral->rx_dma_config.irq_vector );
    }

    if ( peripheral->tx_dma_config.controller != NULL )
    {
        if ( peripheral->tx_dma_config.controller == DMA1 )
        {
            __HAL_RCC_DMA1_CLK_ENABLE( );
        }
        else
        {
            __HAL_RCC_DMA2_CLK_ENABLE( );
        }

        dma_handle_tx = &dma_handles[ uart_number * 2 ];
        platform_uart_tx_dma_init( dma_handle_tx, uart_handle, peripheral );
        NVIC_EnableIRQ( peripheral->tx_dma_config.irq_vector );
    }

    NVIC_EnableIRQ( uart_irq_vectors[ uart_number ] );
    platform_mcu_powersave_enable( );
    return PLATFORM_SUCCESS;
}

/* initialize circular rx dma
 * the dma continues to run even after this function returns,
 * target is auto-incremented, and size is re-used because of
 * dma_handle_rx->Init.Mode = DMA_CIRCULAR
 * and
 * dma_handle_rx->Init.MemInc = DMA_MINC_ENABLE
 */
static platform_result_t init_circular_dma( platform_uart_driver_t* driver, void* data, uint32_t size, uint32_t timeout )
{
    UART_HandleTypeDef * uart_handle;
    uint8_t uart_number;
    platform_result_t result = PLATFORM_SUCCESS;

    uart_number = platform_uart_get_port_number( driver->peripheral->port );
    uart_handle = &uart_handles[ uart_number ];

    __HAL_UART_ENABLE_IT( uart_handle, UART_IT_RXNE );

    /* Starts circular dma */
    if ( HAL_OK != HAL_UART_Receive_DMA( uart_handle, data, size ) )
    {
        result = PLATFORM_ERROR;
    }

    return result;
}

platform_result_t platform_uart_deinit( platform_uart_driver_t* driver )
{
    UART_HandleTypeDef * uart_handle;
    uint8_t uart_number;

    wiced_assert( "bad argument", ( driver != NULL ) );

    platform_mcu_powersave_disable( );
    uart_number = platform_uart_get_port_number( driver->peripheral->port );
    uart_handle = &uart_handles[ uart_number ];
    NVIC_DisableIRQ( uart_irq_vectors[ uart_number ] );

    /* Deinitialize USART */
    HAL_UART_DeInit( uart_handle );

    /* Disable UART peripheral clock */
    uart_clock_disable_function[ uart_number ]( );

    /* Disable UART peripheral pins */
    platform_uart_pin_init( driver, DISABLE );

    host_rtos_deinit_semaphore( &driver->rx_complete );
    host_rtos_deinit_semaphore( &driver->tx_complete );
    driver->rx_size = 0;
    driver->tx_size = 0;
    driver->last_transmit_result = PLATFORM_SUCCESS;
    driver->last_receive_result  = PLATFORM_SUCCESS;
    platform_mcu_powersave_enable( );
    return PLATFORM_SUCCESS;
}

platform_result_t platform_uart_transmit_bytes( platform_uart_driver_t* driver, const uint8_t* data_out, uint32_t size )
{
    UART_HandleTypeDef * uart_handle;
    uint8_t uart_number;
    HAL_StatusTypeDef hal_ret;

    wiced_assert( "bad argument", ( driver != NULL ) && ( data_out != NULL ) && ( size != 0 ) );

    platform_mcu_powersave_disable( );
    uart_number = platform_uart_get_port_number( driver->peripheral->port );
    uart_handle = &uart_handles[ uart_number ];

    /* Use DMA only if UART port is configured and
     * the buffer is cache line aligned */
    if ( ( driver->peripheral->tx_dma_config.controller != NULL ) &&
       ( ( (uint32_t)data_out & PLATFORM_L1_CACHE_LINE_MASK ) == 0 ) )
    {
            /* Flush cache contents to RAM */
            platform_dcache_clean_range( (const volatile void *)data_out, size );

            hal_ret = HAL_UART_Transmit_DMA( uart_handle, (uint8_t *) data_out, size );

            /* Wait for transmission complete */
            if ( hal_ret == HAL_OK )
            {
                if (WWD_SUCCESS != host_rtos_get_semaphore( &driver->tx_complete, UART_TX_MAX_WAIT_TIME, WICED_TRUE ) )
                {
                    hal_ret = HAL_TIMEOUT;
                }
            }
    }
    else
    {
        hal_ret = HAL_UART_Transmit( uart_handle, (uint8_t *) data_out, size, UART_TX_MAX_WAIT_TIME  );
    }

    if (hal_ret == HAL_OK )
    {
        driver->last_transmit_result = PLATFORM_SUCCESS;
    }
    else
    {
        driver->last_transmit_result = PLATFORM_ERROR;
    }

    driver->tx_size = 0;
    platform_mcu_powersave_enable( );
    return driver->last_transmit_result;
}

static platform_result_t get_dma_rx_data_from_circular_buffer( platform_uart_driver_t* driver, uint8_t* data_in, uint32_t* expected_data_size, uint32_t timeout_ms )
{
    uint32_t bytes_left      = *expected_data_size;
    platform_result_t result = PLATFORM_SUCCESS;
    wwd_result_t wwd_result  = WWD_SUCCESS;

    while ( bytes_left != 0 )
    {
        /* Read half of the ring buffer at any time, to leave room for circular DMA to update
         * the remaining buffer
         */
        uint32_t transfer_size = MIN( driver->rx_buffer->size / 2, bytes_left );

        if ( transfer_size > ring_buffer_used_space( driver->rx_buffer ) )
        {
            /* Set rx_size and wait in rx_complete semaphore until data reaches rx_size or timeout occurs */
            driver->last_receive_result = PLATFORM_SUCCESS;
            driver->rx_size = transfer_size;

            wwd_result = host_rtos_get_semaphore( &driver->rx_complete, timeout_ms, WICED_TRUE );

            /* Reset rx_size to prevent semaphore being set while nothing waits for the data */
            driver->rx_size = 0;

            if ( wwd_result == WWD_TIMEOUT )
            {
                /* Semaphore timeout, break from the while loop */
                result = PLATFORM_TIMEOUT;
                break;
            }
            else
            {
                /* No timeout. retrieve result */
                result = driver->last_receive_result;
            }
        }

        bytes_left -= transfer_size;

        /* Grab data from the buffer */
        do
        {
            uint8_t* available_data;
            uint32_t bytes_available;

            /* Cache invalidate ring buffer as contents in cache may be older than the one in RAM updated by DMA */
            platform_dcache_inv_range( (volatile void *) ( driver->rx_buffer ), driver->rx_buffer->size );

            ring_buffer_get_data( driver->rx_buffer, &available_data, &bytes_available );
            bytes_available = MIN( bytes_available, transfer_size );

            if ( bytes_available > 0 )
            {
              memcpy( data_in, available_data, bytes_available );
            }
            transfer_size -= bytes_available;
            data_in        = ( (uint8_t*) data_in + bytes_available );
            ring_buffer_consume( driver->rx_buffer, bytes_available );

        } while ( transfer_size != 0 );
    }

    /* Update actual amount of data rx */
    *expected_data_size -= bytes_left;
    return result;
}

platform_result_t platform_uart_receive_bytes( platform_uart_driver_t* driver, uint8_t* data_in, uint32_t* expected_data_size, uint32_t timeout_ms )
{
    platform_result_t result = PLATFORM_SUCCESS;
    wwd_result_t wwd_result = WICED_SUCCESS;
    UART_HandleTypeDef* uart_handle;
    uint8_t uart_number;
    HAL_StatusTypeDef hal_ret;

    wiced_assert( "bad argument", ( driver != NULL ) && ( data_in != NULL ) && ( expected_data_size != NULL ) && ( *expected_data_size != 0 ) );

    platform_mcu_powersave_disable( );
    uart_number = platform_uart_get_port_number( driver->peripheral->port );
    uart_handle = &uart_handles[ uart_number ];

    if ( ( driver->rx_buffer != NULL ) && ( driver->peripheral->rx_dma_config.controller != NULL ) )
    {
        /* Get RX data from buffer that is filled by DMA */
        result = get_dma_rx_data_from_circular_buffer( driver, data_in, expected_data_size, timeout_ms );
    }
    else
    {
        /* Setup RX and wait for interrupt for RX to complete */
        hal_ret = HAL_UART_Receive_IT( uart_handle, data_in, (uint16_t) ( *expected_data_size ) );
        if ( ( hal_ret == HAL_OK ) || ( hal_ret == HAL_BUSY ) )
        {
            wwd_result = host_rtos_get_semaphore( &driver->rx_complete, timeout_ms, WICED_TRUE );

            driver->rx_size = 0;

            if ( wwd_result == WWD_TIMEOUT )
            {
                result = PLATFORM_TIMEOUT;
            }
            else
            {
                result = driver->last_receive_result;
            }
        }
        else
        {
            result = PLATFORM_ERROR;
        }
    }

    if ( result != PLATFORM_SUCCESS )
    {
        *expected_data_size = 0;
    }

    platform_mcu_powersave_enable( );
    return result;
}

static platform_result_t platform_uart_pin_init( platform_uart_driver_t* driver, FunctionalState state )
{
    platform_uart_t* peripheral = driver->peripheral;
    uint32_t uart_number = platform_uart_get_port_number( driver->peripheral->port );

    if ( state == ENABLE )
    {
        if ( peripheral->tx_pin != NULL )
        {
            platform_gpio_set_alternate_function( peripheral->tx_pin->port, peripheral->tx_pin->pin_number, GPIO_MODE_AF_PP, GPIO_PULLUP, uart_alternate_functions[ uart_number ] );
        }

        if ( peripheral->rx_pin != NULL )
        {
            platform_gpio_set_alternate_function( peripheral->rx_pin->port, peripheral->rx_pin->pin_number, GPIO_MODE_AF_PP, GPIO_PULLUP, uart_alternate_functions[ uart_number ] );
        }

        if ( peripheral->cts_pin != NULL )
        {
            platform_gpio_set_alternate_function( peripheral->cts_pin->port, peripheral->cts_pin->pin_number, GPIO_MODE_AF_PP, GPIO_PULLUP, uart_alternate_functions[ uart_number ] );
        }

        if ( peripheral->rts_pin != NULL )
        {
            platform_gpio_set_alternate_function( peripheral->rts_pin->port, peripheral->rts_pin->pin_number, GPIO_MODE_AF_PP, GPIO_PULLUP, uart_alternate_functions[ uart_number ] );
        }
    }
    else
    {
        if ( peripheral->tx_pin != NULL )
        {
            HAL_GPIO_DeInit( peripheral->tx_pin->port, (uint32_t) ( 1 << peripheral->tx_pin->pin_number ) );
        }

        if ( peripheral->rx_pin != NULL )
        {
            HAL_GPIO_DeInit( peripheral->rx_pin->port, (uint32_t) ( 1 << peripheral->rx_pin->pin_number ) );
        }

        if ( peripheral->cts_pin != NULL )
        {
            HAL_GPIO_DeInit( peripheral->cts_pin->port, (uint32_t) ( 1 << peripheral->cts_pin->pin_number ) );
        }

        if ( peripheral->rts_pin != NULL )
        {
            HAL_GPIO_DeInit( peripheral->rts_pin->port, (uint32_t) ( 1 << peripheral->rts_pin->pin_number ) );
        }
    }

    return PLATFORM_SUCCESS;
}

/******************************************************
 *            IRQ Handlers Definition
 ******************************************************/

void platform_uart_irq( platform_uart_driver_t* driver )
{
    UART_HandleTypeDef* uart_handle;
    DMA_HandleTypeDef* dma_handle_rx;
    uint8_t uart_number;
    wiced_bool_t rx_ring_buffer_used;
    uint32_t ring_buffer_space = 0;

    rx_ring_buffer_used = ( ( driver->rx_buffer != NULL ) && ( driver->peripheral->rx_dma_config.controller != NULL ) ) ? WICED_TRUE : WICED_FALSE;
    uart_number      = platform_uart_get_port_number( driver->peripheral->port );
    uart_handle      = &uart_handles[ uart_number ];
    dma_handle_rx    = &dma_handles[ uart_number * 2 + 1 ];

    HAL_UART_IRQHandler( uart_handle );

    /* Update tail */
    if ( rx_ring_buffer_used )
    {
        platform_uart_update_rx_ring_buffer_tail(driver, dma_handle_rx);
    }

    ring_buffer_space = ring_buffer_used_space( driver->rx_buffer );

    /* Notify thread if sufficient data are available */
    if ( ( rx_ring_buffer_used ) && (( driver->rx_size > 0 ) && (ring_buffer_space >= driver->rx_size ) ) )
    {
        host_rtos_set_semaphore( &driver->rx_complete, WICED_TRUE );
        driver->rx_size = 0;
    }
}

platform_uart_driver_t* platform_uart_get_driver( UART_HandleTypeDef *huart )
{
    int i = 0;
    platform_uart_driver_t* cur_driver;

    for ( i = 0; i < WICED_UART_MAX; i++ )
    {
        cur_driver = &platform_uart_drivers[ i ];

        if ( cur_driver->peripheral->port == huart->Instance )
        {
            return cur_driver;
        }
    }

    return NULL;
}

void platform_uart_dma_tx_irq( platform_uart_driver_t* driver )
{
    UART_HandleTypeDef* uart_handle;
    uint8_t uart_number;

    uart_number = platform_uart_get_port_number( driver->peripheral->port );
    uart_handle = &uart_handles[ uart_number ];
    HAL_DMA_IRQHandler( uart_handle->hdmatx );
}

void platform_uart_dma_rx_irq( platform_uart_driver_t* driver )
{
    UART_HandleTypeDef* uart_handle;
    uint8_t uart_number;

    uart_number = platform_uart_get_port_number( driver->peripheral->port );
    uart_handle = &uart_handles[ uart_number ];
    HAL_DMA_IRQHandler( uart_handle->hdmarx );
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
    platform_uart_driver_t* driver;

    driver = platform_uart_get_driver( huart );

    if ( driver != NULL )
    {
        driver->last_transmit_result = PLATFORM_SUCCESS;

        /* Set semaphore regardless of result to prevent waiting thread from locking up */
        host_rtos_set_semaphore( &driver->tx_complete, WICED_TRUE );

        driver->tx_size = 0;
    }
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
    platform_uart_driver_t * driver;
    driver = platform_uart_get_driver( huart );

    if ( driver != NULL )
    {
        driver->last_receive_result = PLATFORM_SUCCESS;

        /* Set semaphore regardless of result to prevent waiting thread from locking up */
        host_rtos_set_semaphore( &driver->rx_complete, WICED_TRUE );

        driver->rx_size = 0;
    }
}

void HAL_UART_ErrorCallback( UART_HandleTypeDef *huart )
{
    platform_uart_driver_t * driver;
    driver = platform_uart_get_driver( huart );

    if ( driver != NULL )
    {
        driver->last_receive_result = PLATFORM_ERROR;

        /* Set semaphore regardless of result to prevent waiting thread from locking up */
        /* All the below errors are RX errors */
        if (huart->ErrorCode & ( HAL_UART_ERROR_PE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE | HAL_UART_ERROR_NE ) )
        {
            host_rtos_set_semaphore( &driver->rx_complete, WICED_TRUE );
        }

        driver->rx_size = 0;
    }
}
