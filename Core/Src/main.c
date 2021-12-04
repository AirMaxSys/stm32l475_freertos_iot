/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "quadspi.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789v2.h"
#include "aht10.h"
#include "lvgl.h"
#include "ff.h"
#include "FreeRTOS.h"
#include "task.h"
#include "time.h"
#include "queue.h"
#include "proj_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern int wiced_scan_main(void);
extern int tcp_socket_server_main(void);
extern int iot_main(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_init_systick_for_RTX(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lcd_display_task(void *argv)
{
    st7789_init();
    for (;;)
    {
        st7789_fill_color(COLOR_BLUE);
        portDelayMs(1000);
        st7789_fill_color(COLOR_BRED);
        portDelayMs(1000);
        st7789_fill_color(COLOR_CYAN);
        portDelayMs(1000);
    }
}

void module_test_task(void *arg)
{
    (void)arg;
    // Test Fatfs basic functions
#if 0
    uint8_t i;
    FATFS fs;
    DIR dir;
    FIL fp;
    uint32_t res;
    uint32_t written, readcount;
    const char *path = "1:/DOC";
    const char *rwpath = "1:/DOC/test_fatfs.txt";
    const char *create_file_path = "1:/DOC/create_test.txt";
    const char wbuf[] = "The key to a successful open "
        "technology project is to ensure a neutral playing "
        "field for all developers, technologists, and companies "
        "to collectively contribute to project evolution and growth. "
        "The Linux Foundation was built on the idea of the democratization"
        "of code and scaling adoption, for all projects equally. Expert "
        "legal and governance support programs ensure everyone is on the "
        "same playing field.\n";
    char rbuf[1024];

    if ((res = f_mount(&fs, "1:/", 0)) == FR_OK) {
        if ((res = f_opendir(&dir, path)) == FR_OK) {
            static FILINFO fno;

            do {
                if ((res = f_readdir(&dir, &fno)) != FR_OK) {
                    printf("RES:%d f_readdir error!\r\n", res);
                    break;
                } else if (fno.fname[0] != '\0') {
                    printf("%s\r\n", fno.fname);
                }
            } while (fno.fname[0]);
        } else {
            printf("RES:%d f_opendir error!\r\n", res);
        }
        f_closedir(&dir);
        // Write data (mode a)
        res = f_open(&fp, rwpath, FA_WRITE | FA_OPEN_APPEND);
        if (res != FR_OK) {
            printf("RES:%d f_open to write error!\r\n", res);
        } else {
            if ((res = f_write(&fp, wbuf, sizeof(wbuf), &written)) != FR_OK)
                printf("RES:%d f_write error!\r\n", res);
            else
                printf("f_write[%d] Written[%d] success!\r\n", sizeof(wbuf), written);

            f_close(&fp);
        }

        // Read data
        res = f_open(&fp, rwpath, FA_READ);
        if (res != FR_OK) {
            printf("RES:%d f_open to read error!\r\n", res);
        } else {
            if ((res = f_read(&fp, rbuf, 1024, &readcount)) != FR_OK)
                printf("RES:%d f_read error!\r\n", res);
            else {
                printf("f_read[1024] Readden[%d] success!\r\n", readcount);
                rbuf[1023] = '\0';
                printf("%s", rbuf);
            }
            f_close(&fp);
        }

        // Create file
        res = f_open(&fp, create_file_path, FA_WRITE | FA_OPEN_ALWAYS);
        if (res != FR_OK) {
            printf("RES:%d f_open to create file error!\r\n", res);
        } else {
            if ((res = f_write(&fp, wbuf, sizeof(wbuf), &written)) != FR_OK)
                printf("RES:%d f_write error!\r\n", res);
            else
                printf("f_write[%d] Written[%d] success!\r\n", sizeof(wbuf), written);

            f_close(&fp);
        }
    } else {
        printf("RES:%d f_mount error!\r\n", res);
    }

    f_unmount("1:/");
#endif   

    // Test read wifi firmware bin file 43362A2.bin file size 213732bytes
    static FATFS fs;
    FIL fp;
    const char *root_path = "1:/";
    const char *wifi_fw_path = "1:/assets/wifi/43362A2.bin";
    uint32_t res;
    const uint16_t read_chunk = 2048;
    uint8_t rbuf[read_chunk];
    uint8_t count = 0;
    uint32_t read_count;
    uint32_t total_size = 0;

    if ((res = f_mount(&fs, root_path, 1)) != FR_OK) {
        printf("RES[%d] f_mount failure!\r\n", res);
    } else {
        if ((res = f_open(&fp, wifi_fw_path, FA_READ)) != FR_OK) {
            printf("RES[%d] f_open %s failure!\r\n", res, wifi_fw_path);
        } else {
            puts("Start read...");
            do {
                if ((res = f_read(&fp, rbuf, read_chunk, &read_count)) != FR_OK) {
                    printf("RES[%d] count[%d] f_read failure!\r\n", ++count, res);
                    break;
                } else {
                    total_size += read_count;
                } 
            } while (read_chunk == read_count);
            puts("End read...");
            puts("wifi frameware total size 213732 bytes");
            printf("Read total size %d\r\n", total_size);
            printf("Last time read contents count[%d]:\r\n", read_count);
            // Out put last time read 50 bytes
            if (read_count >= 50) {
                for (uint8_t i = 0; i < 50; ++i) {
                    if ((i+1) % 10)
                        printf("%02x ", rbuf[read_count-50+i]);
                    else
                        printf("%02x\r\n", rbuf[read_count-50+i]);
                }
            } else {
                for (uint8_t i = 0; i < read_count; ++i) {
                    if ((i+1) % 10)
                        printf("%02x ", rbuf[i]);
                    else
                        printf("%02x\r\n", rbuf[i]);
                }
            }

            f_close(&fp);
        }
        f_unmount(root_path);
    }

    for (;;) {
        portDelayMs(10000);
    }
} 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)

{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */

#if 0
#include "stdlib.h"
    
    static uint16_t counter = 0;
    uint16_t var;
    while (1) {
        var++;
        th_sensor_msg_buf[0] = 0xc00;
        char *ptr = (char *)malloc(1024);
        if (ptr != NULL) {
            counter ++;
            printf("Malloc total size: %dKb\n", counter);
            ptr = NULL;
        } else {
            while(1);
        }
    }
#endif

    // wiced_scan_main();
    // tcp_socket_server_main();
    
    //module_test_task(NULL);
    iot_main();


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

#if TEST_ST7789_DRIVER == 1
    st7789_init();
#endif

#if (TEST_LVGL_LIB == 1) && (TEST_RTX_RTOS == 0)
    lv_init();
    tft_lvgl_layer_init();
    lv_example_get_started_1();
    lv_example_get_started_3();
#endif

    while (1)
    {
        /* USER CODE END WHILE */
#if TEST_ST7789_DRIVER == 1
        st7789_fill_color(COLOR_BLUE);
        HAL_Delay(50);
        st7789_fill_color(COLOR_BRED);
        HAL_Delay(50);
        st7789_fill_color(COLOR_CYAN);
        HAL_Delay(50);
        st7789_fill_color(COLOR_YELLOW);
        HAL_Delay(50);
        st7789_draw_line(50, 70, 200, 70);
        st7789_draw_line(10, 70, 10, 120);
        st7789_draw_line(100, 20, 120, 50);
        st7789_draw_rectangle(50, 80, 200, 160);
        st7789_draw_line(50, 80, 200, 160);
        st7789_draw_line(50, 160, 200, 80);
        for (int i = 0; i < 20; ++i)
            st7789_draw_point(200 + i, 200, COLOR_GBLUE);
        HAL_Delay(1000);
#endif
#if (TEST_LVGL_LIB == 1) && (TEST_RTX_RTOS == 0)
        HAL_Delay(2);
        lv_task_handler();
#endif
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_SAI1 | RCC_PERIPHCLK_I2C3 | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_SDMMC1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
    PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
    PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK | RCC_PLLSAI1_48M2CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
  */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

// Init SystemTick
void HAL_init_systick_for_RTX(void)
{
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    SysTick_Config(sysclk / 1000);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0U);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM17)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM17)
    {
        // LVGL tick need incerase in SystemTick(TIM17) handler
        lv_tick_inc(1);
    }
    /* USER CODE END Callback 1 */
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("Detect stack overflow in task[%s]\n", pcTaskName);
}

void vApplicationIdleHook(void)
{
    static TimeOut_t timeout;
    static TickType_t fix_period = pdMS_TO_TICKS(1000);
    // capture current time
    vTaskSetTimeOutState(&timeout);
    if (xTaskCheckForTimeOut(&timeout, &fix_period) == pdTRUE)
    {
        printf("Remain %.1fKb!\n", xPortGetFreeHeapSize() / 1024.0);
        fix_period = pdMS_TO_TICKS(1000);
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
