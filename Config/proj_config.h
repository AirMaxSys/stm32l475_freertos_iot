#ifndef _PROJ_CONFIG_H_
#define _PROJ_CONFIG_H_

#define USING_CMSIS_RTOS2   0
#define USING_FREERTOS      1
#define USING_UCOSIII       0
#define USING_ST_HAL_LIB	0

#if USING_CMSIS_RTOS2 == 1
    #include "cmsis_os2.h"

    #define portDelayMs     osDelay
#elif USING_UCOSIII == 1
#elif USING_FREERTOS == 1
    #include "FreeRTOS.h"
    #include "task.h"

    #define portDelayMs     vTaskDelay
#elif USING_ST_HAL_LIB == 1
    #include "stm32l4xx_hal.h"

    #define portDelayMs     HAL_Delay
#endif

#define TEST_LVGL_LIB       0
#define TEST_RTX_RTOS       0
#define TEST_AHT10_DRIVER   0
#define TEST_ST7789_DRIVER  0

#endif
