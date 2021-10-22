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
#pragma once

#include "dtls_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *          DTLS -> Host Function Declarations
 ******************************************************/

extern dtls_result_t dtls_host_create_buffer( dtls_context_t* dtls, uint16_t buffer_size, uint8_t** buffer );
extern dtls_result_t dtls_host_free_packet( uint32_t* packet );
extern dtls_result_t dtls_host_send_tcp_packet( void* context, uint32_t* packet );
extern dtls_result_t dtls_host_get_packet_data( dtls_context_t* dtls, uint32_t* packet, uint32_t offset, uint8_t** data, uint16_t* data_length, uint16_t* available_data_length );
extern dtls_result_t dtls_host_set_packet_start( uint32_t* packet, uint8_t* start );
extern dtls_result_t dtls_host_packet_get_info( uint32_t* packet, dtls_session_t* session );

/*
 * This should wait for a specified amount of time to receive a packet.
 * If the DTLS context already has a received packet stored, it should append it to the previous packet either contiguously or via a linked list.
 */
extern dtls_result_t dtls_host_receive_packet( dtls_context_t* ssl, uint32_t** packet, uint32_t timeout );
extern uint64_t      dtls_host_get_time_ms( void );
extern uint32_t      dtls_host_get_time( void );

extern void* dtls_host_malloc( const char* name, uint32_t size );
extern void  dtls_host_free( void* p );

extern dtls_result_t dtls_flush_output( dtls_context_t* dtls, dtls_session_t* session, uint8_t* buffer, uint32_t length );

/******************************************************
 *           Host -> DTLS Function Declarations
 ******************************************************/
extern dtls_peer_t* dtls_new_peer();

#ifdef __cplusplus
} /*extern "C" */
#endif
