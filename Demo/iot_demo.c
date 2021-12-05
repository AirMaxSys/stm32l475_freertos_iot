#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aht10.h"
#include "st7789v2.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "lvgl.h"
#include "gui.h"

#include "ff.h"

#include "lwip/tcpip.h"
#include "lwip/netif.h"

#include "wwd_network.h"
#include "wwd_management.h"
#include "wwd_wifi.h"
#include "wwd_constants.h"
#include "wwd_debug.h"
#include "platform_init.h"

#include "MQTTClient.h"

#include "proj_config.h"

typedef struct sensor_msg {
    uint16_t th_buf[2];
    xQueueHandle mutex;
} sensor_msg_t;

#define MAKE_IPV4_ADDR(a, b, c, d)  \
    (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))

#define APP_TASK_LOW_PRIORITY   1
#define APP_TASK_DFT_PRIORITY   3
#define APP_TASK_HIGH_PRIORITY  4
#define APP_TASK_LOW_STACKSIZE  128
#define APP_TASK_DFT_STACKSIZE  1024
#define APP_TASK_HIGH_STACKSIZE 2048

#define SDCARD_ROOT_PATH        "1:/"

#define WIFI_NAME               "29-1-us"
#define WIFI_SECURITY_KEY       "11111111"
#define HOSTIP_ADDR             MAKE_IPV4_ADDR(192, 168, 31, 100)
#define GW_ADDR                 MAKE_IPV4_ADDR(192, 168, 31, 1)
#define NETMASK_ADDR            MAKE_IPV4_ADDR(255, 255, 255, 0)

#define BROKER_HOSTNAME         "192.168.31.225"
#define BROKER_PORT             1883
#define CLINET_ID               "PANDORA_BOARD"
#define MQTT_USERNAME           "billy"
#define MQTT_PASSWD             "password"
#define TH_SENSOR_TOPIC         "dev/th_sensor"
#define LIGHT_SENSOR_TOPIC      "dev/light_sensor"
#define MQTT_MSG_PL_LEN         (50)

static xTaskHandle startup_task_handle;
static xTaskHandle mqtt_task_handle;
static xTaskHandle gui_task_handle;
static sensor_msg_t ssmsg;

static void led_task(void *arg)
{
    (void)(arg);

    while (1) {
        HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_RESET);
        portDelayMs(500);
        HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
        portDelayMs(500);
    }
}

static void sensor_task(void *arg)
{
    aht10_t aht10;

    (void)(arg);

    // Wait sensor start work
    portDelayMs(2000);

    aht10_init(&aht10);

    while (1) {
        // Smaple period
        portDelayMs(1000);

        aht10_get_value(&aht10);

        // Update mailbox content
        if (aht10.temp != 0 && aht10.humi != 0) {
            xSemaphoreTake(ssmsg.mutex, portMAX_DELAY);
            ssmsg.th_buf[0] = aht10.temp;
            ssmsg.th_buf[1] = aht10.humi;
            xSemaphoreGive(ssmsg.mutex);
            // Wakeup waitig tasks
            xTaskNotifyGive(mqtt_task_handle);
            xTaskNotifyGive(gui_task_handle);
        }
    }
}

static void gui_task(void *arg)
{
    lv_obj_t *img_temp, *img_humi;
    lv_obj_t *lb_temp, *lb_humi;
    sensor_msg_t msg;

    (void)(arg);

    // Init GUI core
    gui_lvgl_init();

    // Initialize temperature and humidity GUI
    gui_draw_temp_humi_icon_img(&img_temp, &img_humi);
    gui_setup_temp_humi_label(&lb_temp, &lb_humi);

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 0) {
            gui_update_temp_humi_text(lb_temp, lb_humi, ssmsg.th_buf);
        }

        // Update LVGL
        lv_task_handler();
        portYIELD();
    }
}

static void mqtt_task(void *arg)
{
    int res;
    MQTTClient client;
    Network network;
    uint8_t sendbuf[80];
    uint8_t readbuf[80];
    char *b_addr = BROKER_HOSTNAME;
    int b_port = BROKER_PORT;
    uint32_t init_timeout = 30000;
    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

    (void)arg;

    // MQTT initialize
    NetworkInit(&network);
    MQTTClientInit(&client, &network, init_timeout, sendbuf, \
            sizeof(sendbuf), readbuf, sizeof(readbuf));

    // MQTT network connect (lwip socket connect)
    while ((res = NetworkConnect(&network, b_addr, b_port)) != 0) {
        printf("Return code from network connect is %d\n", res);
        vTaskDelay(1000);
    }

    // Config MQTT connection data
    connectData.MQTTVersion = 3;
    connectData.clientID.cstring = CLINET_ID;
    connectData.username.cstring = MQTT_USERNAME;
    connectData.password.cstring = MQTT_PASSWD;
    // MQTT send connect request to broker
    while ((res = MQTTConnect(&client, &connectData)) != 0) {
        printf("Return code from MQTT connect is %d\n", res);
        vTaskDelay(1000);
    }
    if (!res) printf("MQTT Connected\n");

    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 0) {
            MQTTMessage mq_msg;
            float temp = ssmsg.th_buf[0] / 10.0 - 50;
            float humi = ssmsg.th_buf[1] / 10.0;
            char payload[MQTT_MSG_PL_LEN];

            mq_msg.qos = 1;
            mq_msg.retained = 0;
            mq_msg.payload = payload;
            snprintf(payload, MQTT_MSG_PL_LEN - 1, "%.1f %.1f", temp, humi);
            mq_msg.payloadlen = strlen(payload);
            if ((res = MQTTPublish(&client, TH_SENSOR_TOPIC, &mq_msg)) != 0) {
                printf("Return code from MQTT publish is %d\n", res);
            }
        }
        portYIELD();
    }
}

static int setup_wifi_fw(void)
{
    static const wiced_ssid_t ssid = {
        .length = (uint8_t)(sizeof(WIFI_NAME) - 1),
        .value = WIFI_NAME,
    };

    // Turn on wifi device, initialize lowlevel SDIO bus,
    //  burn firmware, config wifi chip
    if (wwd_management_wifi_on(WICED_COUNTRY_AUSTRALIA) != WWD_SUCCESS) {
        WPRINT_APP_ERROR(("Error while starting wifi!\n"));
        return 1;
    }
    // Try to join in AP
    WPRINT_APP_INFO(("Joining : " WIFI_NAME "\n"));
    if (wwd_wifi_join(&ssid, WICED_SECURITY_WPA2_MIXED_PSK, \
            (const uint8_t *)WIFI_SECURITY_KEY,             \
            (uint8_t)(sizeof(WIFI_SECURITY_KEY) - 1),       \
            NULL,       \
            WWD_STA_INTERFACE) != WWD_SUCCESS) {

        WPRINT_APP_INFO(("Failed to join : " WIFI_NAME "\n"));
        return 1;
    }
    WPRINT_APP_INFO(("Successfully joined : " WIFI_NAME "\n"));
    
    return 0;
}

static int setup_lwip_network_if(void)
{
    static struct netif devif;
    static ip4_addr_t hostip, netmask, gateway;

    hostip.addr = htonl(HOSTIP_ADDR); 
    gateway.addr = htonl(GW_ADDR);
    netmask.addr = htonl(NETMASK_ADDR);

    if (netif_add(&devif, &hostip, &netmask, &gateway,     \
            (void *)WWD_STA_INTERFACE, ethernetif_init,    \
            netif_input) == NULL) {
        WPRINT_APP_ERROR("Failed to start network interface\n");
        return 1;
    }
    netif_set_default(&devif);
    netif_set_up(&devif);
    WPRINT_APP_INFO(( "Network ready IP: %s\n", \
            ip4addr_ntoa(netif_ip4_addr(&devif))));

    return 0;
}

static void iot_tcpip_init_done_cb(void *arg)
{
    xSemaphoreHandle *sema = (xSemaphoreHandle *)arg;

    xSemaphoreGive(*sema);
}

extern void prvMQTTEchoTask(void *pvParameters);

static void app_main(void)
{
    setup_wifi_fw();
    setup_lwip_network_if();

    if (xTaskCreate(led_task, "LED", configMINIMAL_STACK_SIZE,  \
            NULL, APP_TASK_LOW_PRIORITY, NULL) != pdTRUE) {
        printf("LED task create failure!\n");
        while (1);
    }
    if (xTaskCreate(sensor_task, "sensor", APP_TASK_DFT_STACKSIZE,  \
            NULL, APP_TASK_HIGH_PRIORITY, NULL) != pdTRUE) {
        printf("Sensor task create failure!\n");
        while (1);
    }
    if (xTaskCreate(gui_task, "GUI", APP_TASK_HIGH_STACKSIZE,   \
            NULL, APP_TASK_DFT_PRIORITY, &gui_task_handle) != pdTRUE) {
        printf("GUI task create failure!\n");
        while (1);
    }
    if (xTaskCreate(mqtt_task, "mqtt", APP_TASK_HIGH_STACKSIZE, \
            NULL, APP_TASK_DFT_PRIORITY, &mqtt_task_handle) != pdTRUE) {
        printf("MQTT task create failure!\n");
        while (1);
    }
}

static void startup_task(void *arg)
{
    static FATFS fs;
    xSemaphoreHandle lwip_done_sema;

    (void)(arg);

    // Check sd card
    if (f_mount(&fs, SDCARD_ROOT_PATH, 1) != FR_OK) {
        printf(("Fatfs mount failed!\r\n"));
        // TODO: handle SD card mount failure status
        return;
    }

    // Create sensor messages buffer mutex
    ssmsg.mutex = xSemaphoreCreateMutex();
    if (!ssmsg.mutex) {
        printf("Create sensor meaages buffer mutex failure!\n");
        return;
    }

    // Create counting semaphore waiting for lwip done
    lwip_done_sema = xSemaphoreCreateCounting(1, 0);
    if (!lwip_done_sema) {
        printf("Create lwip initializtion semaphore failure!\r\n");
        return;
    }
    // Initialize lwip
    tcpip_init(iot_tcpip_init_done_cb, (void *)&lwip_done_sema);
    xSemaphoreTake(lwip_done_sema, portMAX_DELAY);
    // Delete semaphore
    vQueueDelete(lwip_done_sema);

    app_main();

    vTaskDelete(startup_task_handle);
}

int iot_main(void)
{
    // Initialize MCU related stuff
    platform_init_mcu_infrastructure();
    
    // Create start up task
    if (xTaskCreate(startup_task, "startup",  \
            APP_TASK_DFT_STACKSIZE, NULL, APP_TASK_LOW_PRIORITY, &startup_task_handle) != pdTRUE) {
        printf("Create start up task failure!\r\n");
        return 1;
    }

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never get here, unless there is and error in vTaskStartScheduler
    printf("Main() function returned - error!\r\n");

    return 0;
}
