#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "wwd_network.h"
#include "wwd_management.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_events.h"
#include "wwd_assert.h"
#include "RTOS/wwd_rtos_interface.h"
#include "platform/wwd_platform_interface.h"
#include "lwip/tcpip.h"
#include "wiced_utilities.h"
#include "network/wwd_buffer_interface.h"

#include "platform_mcu_peripheral.h"
#include "platform_init.h"

#include "lwip/opt.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/inet.h"
#include "netif/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "lwip/err.h"
#include "sys.h"

/* Macros*/
#define MAKE_IPV4_ADDR(a, b, c, d)  \
    (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))

/* Constants */
#define APP_THREAD_STACKSIZE        (1024)
#define COUNTRY                     WICED_COUNTRY_AUSTRALIA
#define HOSTIP_ADDR                 MAKE_IPV4_ADDR(192, 168, 8, 200)
#define GW_ADDR                     MAKE_IPV4_ADDR(192, 168, 8, 1)
#define NETMASK_ADDR                MAKE_IPV4_ADDR(255, 255, 255, 0)
#define SERV_ADDR_BYTE0             (192)
#define SERV_ADDR_BYTE1             (168)
#define SERV_ADDR_BYTE2             (8)
#define SERV_ADDR_BYTE3             (101)
#define SERV_PORT_NUM               (8000)
#define TCP_SOCKET_TASK_PRIORITY    (4)
#define TCP_SOCKET_TASK_STACK_SIZE  (2048*4)
#define AP_SSID                     "NMSL_saodeyi"
#define AP_PASSWD                   "11111111"

/* Static functions declarations */
static void tcpip_init_done(void *argv);
static void startup_thread(void *argv);
static int config_wifi_lwip(void);

/* Variable definitions */
static const wiced_ssid_t ap_ssid = {
    (uint8_t)(sizeof(AP_SSID) - 1),
    AP_SSID,
};
static xTaskHandle startup_thread_handle;
static struct netif dev_if;
ip4_addr_t hostip, netmask, gateway;

/* Extern variable and funtion declarations*/
extern int errno;
// test mqtt
extern void prvMQTTEchoTask(void *pvParameters);
extern void mqttTHSensorTask(void *argv);
// test othre task
extern void gui_task(void *argv);
extern void lcd_display_task(void *argv);
extern void led_blink_task(void *argv);
extern void temp_humi_smaple_task(void *argv);

/* Function definitios*/
static int config_wifi_lwip(void)
{
    /* Initialise Wiced */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    wwd_buffer_init(NULL);
    if (wwd_management_wifi_on(COUNTRY) != WWD_SUCCESS) {
        WPRINT_APP_ERROR(("Error while starting wifi!\n"));
    }

    /* Try to join in AP*/
    WPRINT_APP_INFO(("Joining : " AP_SSID "\n"));
    if (wwd_wifi_join(&ap_ssid, WICED_SECURITY_WPA2_MIXED_PSK, (const uint8_t *)AP_PASSWD,  \
            strlen(AP_PASSWD), NULL, WWD_STA_INTERFACE) != WWD_SUCCESS) {
        WPRINT_APP_INFO(("Failed to join : " AP_SSID "\n"));
        return -1;
    }
    WPRINT_APP_INFO(("Successfully joined : " AP_SSID "\n"));

    /* Config IP infomations and not use DHCP*/
    hostip.addr     = htonl(HOSTIP_ADDR);
    gateway.addr    = htonl(GW_ADDR);
    netmask.addr    = htonl(NETMASK_ADDR);

    /* Create network interface*/
    if (!netif_add(&dev_if, &hostip, &netmask, &gateway, \
            (void *)WWD_STA_INTERFACE, ethernetif_init, netif_input)) {
        WPRINT_APP_ERROR("Failed to start network interface\n");
        return -1;
    }
    netif_set_default(&dev_if);
    netif_set_up(&dev_if);

    /* TODO: DHCP setup if using DHCP*/
    
    WPRINT_APP_INFO( ( "Network ready IP: %s\n", ip4addr_ntoa(netif_ip4_addr(&dev_if))));
    
    return 0;
}

static void tcp_socket_task(void *argv)
{
    struct sockaddr_in servaddr;
    ip4_addr_t servipaddr;
    const uint8_t wbuf[] = {"Hello server, I'm Pandora!\r\n"};

    IP4_ADDR(&servipaddr, SERV_ADDR_BYTE0, SERV_ADDR_BYTE1, \
            SERV_ADDR_BYTE2, SERV_ADDR_BYTE3);
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(SERV_PORT_NUM);
    servaddr.sin_addr.s_addr = servipaddr.addr;

    while (1) {
        // family - IPV4 protocol, type - TCP
        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            WPRINT_APP_ERROR(("Create socket error!\n"));
        }
        memset(servaddr.sin_zero, 0, sizeof(servaddr.sin_zero));
        if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(struct sockaddr)) < 0) {
            WPRINT_APP_INFO(("Socket connect failed!\n"));
            closesocket(sockfd);
            vTaskDelay(10);
            continue;
        }
        WPRINT_APP_INFO(("Connect to server successful!\n"));

        while (1) {
            if (write(sockfd, (const uint8_t *)wbuf, sizeof(wbuf)) < 0) {
                WPRINT_APP_INFO(("Socket write err! ERRNO=%d\n", errno));
                break;
            } else {
                WPRINT_APP_INFO(("Socket write successful!\n"));
            }
            vTaskDelay(1000);
        }
        closesocket(sockfd);
    }
}

static void app_main(void)
{
    if (config_wifi_lwip() < 0)
        return;
#if 0 
    // Call lwip layer thread create funtion to initialize a new clinet thread
    sys_thread_new("tcp_socket", tcp_socket_task, NULL, \
            TCP_SOCKET_TASK_STACK_SIZE, TCP_SOCKET_TASK_PRIORITY);

#endif
    // test mqtt
    // sys_thread_new("mqtt_test", prvMQTTEchoTask, NULL,
    //                TCP_SOCKET_TASK_STACK_SIZE, TCP_SOCKET_TASK_PRIORITY);
    sys_thread_new("mqtt_th_sensor_task", mqttTHSensorTask, NULL,
                   TCP_SOCKET_TASK_STACK_SIZE, TCP_SOCKET_TASK_PRIORITY);
    sys_thread_new("led_blink_task", led_blink_task, NULL,
                   512 * 4, 4);
    sys_thread_new("lcd_blink", lcd_display_task, NULL,
                   2048 * 4, 4);
    sys_thread_new("sensor_task", temp_humi_smaple_task, NULL,
                   1024 * 4, 4);
}

int tcp_socket_server_main(void)
{
    signed portBASE_TYPE create_result;
    
    // NOTE: must add
    platform_init_mcu_infrastructure();

    /* Create an initial thread */
    create_result = xTaskCreate(startup_thread, "app_thread",   \
        (unsigned short)(APP_THREAD_STACKSIZE / sizeof( portSTACK_TYPE )),  \
        NULL, DEFAULT_THREAD_PRIO, &startup_thread_handle);

    wiced_assert("Failed to create main thread", create_result == pdPASS );
    REFERENCE_DEBUG_ONLY_VARIABLE( create_result );

    /* Start the FreeRTOS scheduler - this call should never return */
    vTaskStartScheduler( );

    /* Should never get here, unless there is an error in vTaskStartScheduler */
    WPRINT_APP_ERROR(("Main() function returned - error" ));

    return 0;
}

static void startup_thread(void *argv)
{
    xSemaphoreHandle lwip_done_sema;
    (void)argv;

    WPRINT_APP_INFO(("\nPlatform " PLATFORM " initialised\n"));
    WPRINT_APP_INFO(("Started FreeRTOS " FreeRTOS_VERSION "\n"));
    WPRINT_APP_INFO(("Starting LwIP " LwIP_VERSION "\n"));

    /* Create a semaphore to signal when LwIP has finished initialising */
    lwip_done_sema = xSemaphoreCreateCounting(1, 0);
    if (!lwip_done_sema) {
        /* could not create semaphore */
        WPRINT_APP_ERROR(("Could not create LwIP init semaphore"));
        return;
    }

    /* Initialise LwIP, providing the callback function and callback semaphore */
    tcpip_init( tcpip_init_done, (void*) &lwip_done_sema );
    xSemaphoreTake( lwip_done_sema, portMAX_DELAY );
    vQueueDelete( lwip_done_sema );

    /* Run the main application function */
    app_main( );

    /* Clean up this startup thread */
    vTaskDelete( startup_thread_handle );
}

static void tcpip_init_done( void * arg )
{
    xSemaphoreHandle * lwip_done_sema = (xSemaphoreHandle *) arg;
    xSemaphoreGive( *lwip_done_sema );
}
