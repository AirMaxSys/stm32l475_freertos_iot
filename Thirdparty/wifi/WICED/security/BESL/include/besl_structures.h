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

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define BESL_ETHERNET_ADDRESS_LENGTH      (6)

/******************************************************
 *                   Enumerations
 ******************************************************/

#ifndef RESULT_ENUM
#define RESULT_ENUM( prefix, name, value )  prefix ## name = (value)
#endif /* ifndef RESULT_ENUM */

/*
 * Values: 3000 - 3999
 */
#define WPS_BESL_RESULT_LIST( prefix ) \
    RESULT_ENUM( prefix, SUCCESS,                                    0 ),   /**< Success */          \
    RESULT_ENUM( prefix, TIMEOUT,                                    2 ),   /**< Timeout */          \
    RESULT_ENUM( prefix, ERROR,                                      4 ),   /**<   */ \
    RESULT_ENUM( prefix, BADARG,                                     5 ),   /**< Bad Arguments */    \
    RESULT_ENUM( prefix, UNPROCESSED,                             3001 ),   /**<  */                 \
    RESULT_ENUM( prefix, IN_PROGRESS,                             3002 ),   /**< In progress */      \
    RESULT_ENUM( prefix, COMPLETE,                                3003 ),   /**<   */ \
    RESULT_ENUM( prefix, PBC_OVERLAP,                             3004 ),   /**<   */ \
    RESULT_ENUM( prefix, ABORTED,                                 3005 ),   /**<   */ \
    RESULT_ENUM( prefix, NOT_STARTED,                             3006 ),   /**<   */ \
    RESULT_ENUM( prefix, REGISTRAR_NOT_READY,                     3007 ),   /**<   */ \
    RESULT_ENUM( prefix, ATTEMPTED_EXTERNAL_REGISTRAR_DISCOVERY,  3008 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_JOIN_FAILED,                       3009 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_ENCRYPTED_TLV_HMAC_FAIL,           3010 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_SECRET_NONCE_MISMATCH,             3011 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_MESSAGE_HMAC_FAIL,                 3012 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_MESSAGE_TLV_MASK_MISMATCH,         3013 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_ENCRYPTION_TYPE_ERROR,             3014 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_AUTHENTICATION_TYPE_ERROR,         3015 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_REGISTRAR_NONCE_MISMATCH,          3016 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_ENROLLEE_NONCE_MISMATCH,           3017 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_VERSION_MISMATCH,                  3018 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CRYPTO_ERROR,                      3019 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_MESSAGE_MISSING_TLV,               3020 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INCORRECT_MESSAGE,                 3021 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_NO_RESPONSE,                       3022 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_RECEIVED_EAP_FAIL,                 3023 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_RECEIVED_WEP_CREDENTIALS,          3024 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_OUT_OF_MEMORY,                     3025 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_QUEUE_PUSH,                        3026 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_SCAN_START_FAIL,                   3027 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_WPS_STACK_MALLOC_FAIL,             3028 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CREATING_EAPOL_PACKET,             3029 ),   /**<   */ \
    RESULT_ENUM( prefix, UNKNOWN_EVENT,                           3033 ),   /**<   */ \
    RESULT_ENUM( prefix, OUT_OF_HEAP_SPACE,                       3034 ),   /**<   */ \
    RESULT_ENUM( prefix, CERT_PARSE_FAIL,                         3035 ),   /**<   */ \
    RESULT_ENUM( prefix, KEY_PARSE_FAIL,                          3036 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_DEVICE_LIST_FIND,                  3037 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_NO_P2P_TLV,                        3038 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_ALREADY_STARTED,                   3039 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_HANDLER_ALREADY_REGISTERED,        3040 ),   /**<   */ \
    RESULT_ENUM( prefix, BUFFER_ALLOC_FAIL,                       3041 ),   /**<   */ \
    RESULT_ENUM( prefix, OTHER_ENROLLEE,                          3042 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_RECEIVED_INVALID_CREDENTIALS,      3043 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_HMAC_CHECK_FAIL,                   3044 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNABLE_TO_SET_WLAN_SECURITY,       3045 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_RUNT_WPS_PACKET,                   3046 ),   /**<   */

#define TLS_RESULT_LIST( prefix ) \
    RESULT_ENUM( prefix, SUCCESS,                                0 ),   /**<   */ \
    RESULT_ENUM( prefix, TIMEOUT,                                2 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR,                                  4 ),   /**<   */ \
    RESULT_ENUM( prefix, RECEIVE_FAILED,                      5001 ),   /**<   */ \
    RESULT_ENUM( prefix, ALERT_NO_CERTIFICATE,                5002 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_OUT_OF_MEMORY,                 5003 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_FEATURE_UNAVAILABLE,           5004 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_INPUT_DATA,                5005 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_MAC,                   5006 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_RECORD,                5007 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_MODULUS_SIZE,          5008 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNKNOWN_CIPHER,                5009 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_NO_CIPHER_CHOSEN,              5010 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_NO_SESSION_FOUND,              5011 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_NO_CLIENT_CERTIFICATE,         5012 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CERTIFICATE_TOO_LARGE,         5013 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CERTIFICATE_REQUIRED,          5014 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_PRIVATE_KEY_REQUIRED,          5015 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CA_CHAIN_REQUIRED,             5016 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNEXPECTED_MESSAGE,            5017 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_FATAL_ALERT_MESSAGE,           5018 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_PEER_VERIFY_FAILED,            5019 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_PEER_CLOSE_NOTIFY,             5020 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CLIENT_HELLO,           5021 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_SERVER_HELLO,           5022 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CERTIFICATE,            5023 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CERTIFICATE_REQUEST,    5024 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_SERVER_KEY_EXCHANGE,    5025 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_SERVER_HELLO_DONE,      5026 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CLIENT_KEY_EXCHANGE,    5027 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CERTIFICATE_VERIFY,     5028 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CHANGE_CIPHER_SPEC,     5029 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_FINISHED,               5030 ),   /**<   */ \
    RESULT_ENUM( prefix, HANDSHAKE_TIMEOUT,                   5031 ),   /**<   */ \
    RESULT_ENUM( prefix, HANDSHAKE_ERROR,                     5032 ),   /**<   */ \
    RESULT_ENUM( prefix, INIT_FAIL,                           5033 ),   /**<   */ \
    RESULT_ENUM( prefix, BAD_MESSAGE,                         5034 ),   /**<   */ \
    RESULT_ENUM( prefix, UNTRUSTED_CERTIFICATE,               5035 ),   /**<   */ \
    RESULT_ENUM( prefix, EXPIRED_CERTIFICATE,                 5036 ),   /**<   */ \
    RESULT_ENUM( prefix, CERTIFICATE_NAME_MISMATCH,           5037 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_DECRYPTION_FAIL,               5038 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_ENCRYPTION_FAIL,               5039 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_HMAC_CHECK_FAIL,               5040 ),   /**<   */ \
    RESULT_ENUM( prefix, CERTIFICATE_REVOKED,                 5041 ),   /**<   */ \
    RESULT_ENUM( prefix, NO_DATA,                             5042 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNSUPPORTED_EXTENSION,         5043 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNITIALIZED_CONTEXT,           5044 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNRECOGNIZED_SERVER_NAME,      5045 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_ILLEGAL_PARAMETER,             5046 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_RECORD_OVERFLOW,               5047 ),   /**<   */

#define DTLS_RESULT_LIST( prefix ) \
    RESULT_ENUM( prefix, SUCCESS,                                0 ),   /**<   */ \
    RESULT_ENUM( prefix, TIMEOUT,                                2 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR,                                  4 ),   /**<   */ \
    RESULT_ENUM( prefix, RECEIVE_FAILED,                      11001 ),   /**<   */ \
    RESULT_ENUM( prefix, ALERT_NO_CERTIFICATE,                11002 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_OUT_OF_MEMORY,                 11003 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_FEATURE_UNAVAILABLE,           11004 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_INPUT_DATA,                11005 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_MAC,                   11006 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_RECORD,                11007 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_MODULUS_SIZE,          11008 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNKNOWN_CIPHER,                11009 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_NO_CIPHER_CHOSEN,              11010 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_NO_SESSION_FOUND,              11011 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_NO_CLIENT_CERTIFICATE,         11012 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CERTIFICATE_TOO_LARGE,         11013 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CERTIFICATE_REQUIRED,          11014 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_PRIVATE_KEY_REQUIRED,          11015 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CA_CHAIN_REQUIRED,             11016 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNEXPECTED_MESSAGE,            11017 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_FATAL_ALERT_MESSAGE,           11018 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_PEER_VERIFY_FAILED,            11019 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_PEER_CLOSE_NOTIFY,             11020 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CLIENT_HELLO,           11021 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_SERVER_HELLO,           11022 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CERTIFICATE,            11023 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CERTIFICATE_REQUEST,    11024 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_SERVER_KEY_EXCHANGE,    11025 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_SERVER_HELLO_DONE,      11026 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CLIENT_KEY_EXCHANGE,    11027 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CERTIFICATE_VERIFY,     11028 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_CHANGE_CIPHER_SPEC,     11029 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_BAD_HS_FINISHED,               11030 ),   /**<   */ \
    RESULT_ENUM( prefix, HANDSHAKE_TIMEOUT,                   11031 ),   /**<   */ \
    RESULT_ENUM( prefix, HANDSHAKE_ERROR,                     11032 ),   /**<   */ \
    RESULT_ENUM( prefix, INIT_FAIL,                           11033 ),   /**<   */ \
    RESULT_ENUM( prefix, BAD_MESSAGE,                         11034 ),   /**<   */ \
    RESULT_ENUM( prefix, UNTRUSTED_CERTIFICATE,               11035 ),   /**<   */ \
    RESULT_ENUM( prefix, EXPIRED_CERTIFICATE,                 11036 ),   /**<   */ \
    RESULT_ENUM( prefix, CERTIFICATE_NAME_MISMATCH,           11037 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_DECRYPTION_FAIL,               11038 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_ENCRYPTION_FAIL,               11039 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_HMAC_CHECK_FAIL,               11040 ),   /**<   */ \
    RESULT_ENUM( prefix, CERTIFICATE_REVOKED,                 11041 ),   /**<   */ \
    RESULT_ENUM( prefix, NO_DATA,                             11042 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_UNSUPPORTED_EXTENSION,         11043 ),   /**<   */


    #define SUPPLICANT_RESULT_LIST( prefix ) \
    RESULT_ENUM( prefix, SUCCESS,                                0 ),   /**<   */ \
    RESULT_ENUM( prefix, TIMEOUT,                                2 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR,                                  4 ),   /**<   */ \
    RESULT_ENUM( prefix, IN_PROGRESS,                         6001 ),   /**<   */ \
    RESULT_ENUM( prefix, ABORTED,                             6002 ),   /**<   */ \
    RESULT_ENUM( prefix, NOT_STARTED,                         6003 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_STACK_MALLOC_FAIL,             6004 ),   /**<   */ \
    RESULT_ENUM( prefix, OUT_OF_HEAP_SPACE,                   6005 ),   /**<   */ \
    RESULT_ENUM( prefix, COMPLETE,                            6006 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_AT_THREAD_START,               6007 ),   /**<   */ \
    RESULT_ENUM( prefix, UNPROCESSED,                         6008 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_CREATING_EAPOL_PACKET,         6009 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_READING_BSSID,                 6010 ),   /**<   */ \
    RESULT_ENUM( prefix, RECEIVED_EAP_FAIL,                   6011 ),   /**<   */

#define P2P_RESULT_LIST( prefix ) \
    RESULT_ENUM( prefix, SUCCESS,                                0 ),   /**<   */ \
    RESULT_ENUM( prefix, TIMEOUT,                                2 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_FAIL,                          9001 ),   /**<   */ \
    RESULT_ENUM( prefix, ACCEPT_CONNECTION,                   9002 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INCORRECT_SSID,                9003 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_DEVICE,                9004 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_MISSING_RSN_IE,                9005 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_SCAN_RESULT,           9006 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_MISSING_TLV,                   9007 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_INVALID_MAC_ADDRESS,           9008 ),   /**<   */ \
    RESULT_ENUM( prefix, ERROR_ASSOCIATED_DEVICE_NOT_FOUND,   9009 ),   /**<   */ \
    RESULT_ENUM( prefix, GROUP_OWNER_WAITING_FOR_CONNECTION,  9010 ),   /**<   */ \
    RESULT_ENUM( prefix, GROUP_OWNER_WPS_IN_PROGRESS,         9011 ),   /**<   */ \
    RESULT_ENUM( prefix, GROUP_OWNER_WPS_FAILED,              9012 ),   /**<   */ \
    RESULT_ENUM( prefix, GROUP_OWNER_WPS_COMPLETE,            9013 ),   /**<   */ \
    RESULT_ENUM( prefix, GROUP_OWNER_WPS_PBC_OVERLAP,         9014 ),   /**<   */ \
    RESULT_ENUM( prefix, GROUP_OWNER_WPS_TIMED_OUT,           9015 ),   /**<   */ \
    RESULT_ENUM( prefix, GROUP_OWNER_WPS_ABORTED,             9016 ),   /**<   */ \
    RESULT_ENUM( prefix, UNKNOWN,                             9017 ),   /**<   */

typedef enum
{
    WPS_BESL_RESULT_LIST( BESL_ )
    TLS_RESULT_LIST( BESL_TLS_ )
    SUPPLICANT_RESULT_LIST( SUPPLICANT_ )
    P2P_RESULT_LIST( BESL_P2P_ )
} besl_result_t;

#define AVP_LENGTH_MASK 0x00FFFFFF
/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef uint8_t   besl_bool_t;
typedef uint32_t  besl_time_t;

/******************************************************
 *                 Packed Structures
 ******************************************************/

#pragma pack(1)

typedef struct
{
    uint8_t octet[BESL_ETHERNET_ADDRESS_LENGTH];
} besl_mac_t;

typedef struct
{
    uint8_t* data;
    uint16_t length;
    uint32_t packet_mask;
} besl_ie_t;

typedef struct
{
    uint8_t   ether_dhost[BESL_ETHERNET_ADDRESS_LENGTH];
    uint8_t   ether_shost[BESL_ETHERNET_ADDRESS_LENGTH];
    uint16_t  ether_type;
} ether_header_t;

typedef struct
{
    uint8_t  version;
    uint8_t  type;
    uint16_t length;
} eapol_header_t;

typedef struct
{
    ether_header_t  ethernet;
    eapol_header_t  eapol;
} eapol_packet_header_t;

typedef struct
{
    ether_header_t  ethernet;
    eapol_header_t  eapol;
    uint8_t         data[1];
} eapol_packet_t;

typedef struct
{
    uint8_t  code;
    uint8_t  id;
    uint16_t length;
    uint8_t  type;
} eap_header_t;

typedef struct
{
    uint8_t  type;
} peap_header_t;

typedef struct
{
    uint8_t  type;
    uint8_t  data[1];
} peap_packet_t;

typedef struct
{
    uint32_t avp_code;
    uint8_t  flags;
    uint8_t  avp_length[3];
}avp_header_t;

typedef struct
{
        uint8_t Version;
        uint8_t reserved;
        uint8_t count;
}leap_header;

typedef struct
{
    avp_header_t avp_header;
    uint8_t data[1];
}avp_packet_t;

typedef struct
{
    avp_header_t avp_header;
    uint32_t vendor_id;
    uint8_t data[1];
}avp_vendor_packet_t;


typedef struct
{
    ether_header_t  ethernet;
    eapol_header_t  eapol;
    eap_header_t    eap;
    uint8_t         data[1];
} eap_packet_t;

typedef struct
{
    uint8_t  vendor_id[3];
    uint32_t vendor_type;
    uint8_t  op_code;
    uint8_t  flags;
} eap_expanded_header_t;

typedef struct
{
    uint8_t  flags;
} eap_tls_header_t;

typedef struct
{
    ether_header_t        ethernet;
    eapol_header_t        eapol;
    eap_header_t          eap;
    eap_tls_header_t      eap_tls;
    uint8_t               data[1]; // Data starts with a length of TLS data field or TLS data depending on the flags field
} eap_tls_packet_t;

typedef struct
{
    uint16_t     type;
    uint16_t     length;
    uint8_t      value[1];
}avp_request_t;

typedef struct
{
    uint16_t     type;
    uint16_t     length;
    uint16_t     status;
}avp_result_t;

typedef struct
{
    eap_header_t    header;
    avp_request_t   avp[1];
}peap_extention_request_t;

typedef struct
{
    eap_header_t   header;
    avp_result_t   avp[1];
}peap_extention_response_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
}mschapv2_header_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
    uint8_t     data[1];
}mschapv2_packet_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
    uint8_t     value_size;
    uint8_t     challenge[16];
    uint8_t     name[1];
}mschapv2_challenge_packet_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
    uint8_t     value_size;
    uint8_t     peer_challenge[16];
    uint8_t     reserved[8];
    uint8_t     nt_reponse[24];
    uint8_t     flags;
    uint8_t     name[1];
}mschapv2_response_packet_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
    uint8_t     message[1];
}mschapv2_success_request_packet_t;

typedef struct
{
    uint8_t     opcode;
}mschapv2_success_response_packet_t;

typedef mschapv2_success_request_packet_t mschapv2_failure_request_packet_t;

typedef mschapv2_success_response_packet_t mschapv2_failure_response_packet_t;

#pragma pack()

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
