#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aht10.h"
#include "st7789v2.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#include "lvgl.h"
#include "gui.h"

#include "ff.h"

#include "proj_config.h"

#define APP_TASK_LOW_PRIORITY   1
#define APP_TASK_DFT_PRIORITY   3
#define APP_TASK_HIGH_PRIORITY  4
#define APP_TASK_LOW_STACKSIZE  128
#define APP_TASK_DFT_STACKSIZE  1024
#define APP_TASK_HIGH_STACKSIZE 2048

#define SDCARD_ROOT_PATH    "1:/"

QueueHandle_t th_sensor_msg;
uint16_t th_sensor_msg_buf[2];

static xTaskHandle startup_task_handle;

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
    (void)(arg);
    aht10_t aht10;

    // Wait sensor start work
    portDelayMs(2000);

    aht10_init(&aht10);

    while (1) {
        // Smaple period
        portDelayMs(1000);

        aht10_get_value(&aht10);

        // Put data to message queue
        if (aht10.temp != 0 && aht10.humi != 0) {
            th_sensor_msg_buf[0] = aht10.temp;
            th_sensor_msg_buf[1] = aht10.humi;
#if USING_FREERTOS == 1
            if (th_sensor_msg != 0)
                xQueueSend(th_sensor_msg, th_sensor_msg_buf, 0);
#endif
        }
    }
}

static void gui_task(void *argv)
{
    (void)(argv);
    lv_obj_t *lb_temp, *lb_humi;
    lv_obj_t *img_temp, *img_humi;

    // Init GUI core
    gui_lvgl_init();

    // Initialize temperature and humidity GUI
    gui_draw_temp_humi_icon_img(&img_temp, &img_humi);
    gui_setup_temp_humi_label(&lb_temp, &lb_humi);

    for (;;) {
#if USING_FREERTOS == 1
        if (th_sensor_msg != 0)
            if (xQueueReceive(th_sensor_msg, th_sensor_msg_buf, pdMS_TO_TICKS(5)) == pdTRUE)
#endif
                gui_update_temp_humi_text(lb_temp, lb_humi, th_sensor_msg_buf);
        lv_task_handler();
    }
}

static void app_main(void)
{
    if (xTaskCreate(led_task, "LED", configMINIMAL_STACK_SIZE,  \
            NULL, APP_TASK_LOW_PRIORITY, NULL) != pdTRUE) {
        printf("LED task create failure!\r\n");
        while (1);
    }
    if (xTaskCreate(sensor_task, "sensor", APP_TASK_DFT_STACKSIZE,  \
            NULL, APP_TASK_HIGH_PRIORITY, NULL) != pdTRUE) {
        printf("Sensor task create failure!\r\n");
        while (1);
    }
    if (xTaskCreate(gui_task, "GUI", APP_TASK_HIGH_STACKSIZE,   \
            NULL, APP_TASK_DFT_PRIORITY, NULL) != pdTRUE) {
        printf("GUI task create failure!\r\n");
        while (1);
    }
}

static void startup_task(void *arg)
{
    (void)(arg);
    static FATFS fs;

    // Check sd card
    if (f_mount(&fs, SDCARD_ROOT_PATH, 1) != FR_OK) {
        printf(("Fatfs mount failed!\r\n"));
        // TODO: handle SD card mount failure status
    }

    th_sensor_msg = xQueueCreate(1, sizeof(th_sensor_msg_buf));
    if (!th_sensor_msg) {
        printf("MSG queue of TH seonsor created failed!\n");
    }

    app_main();

    vTaskDelete(startup_task_handle);
}

int iot_main(void)
{
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
