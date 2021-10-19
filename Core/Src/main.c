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
#include "i2c.h"
#include "quadspi.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "tft_lvgl_layer.h"
#include "lvgl.h"
#include "gui.h"

#include "proj_config.h"
#include "aht10.h"
#include "st7789v2.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static QueueHandle_t TH_msgQ;
static uint16_t TH_msgQ_buf[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_init_systick_for_RTX(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void led_blink_task(void *argv)
{
    (void)(argv);
	while (1) {
		HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_RESET);
		portDelayMs(500);
		HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
		portDelayMs(500);
	}
}

void temp_humi_smaple_task(void *argv)
{
    (void)(argv);
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
            TH_msgQ_buf[0] = aht10.temp;
            TH_msgQ_buf[1] = aht10.humi;
#if USING_FREERTOS == 1
            xQueueSend(TH_msgQ, TH_msgQ_buf, 0);
#endif
        }
	}
}

void lcd_display_task(void *argv)
{
    st7789_init();
    for ( ;; ) {
        st7789_fill_color(COLOR_BLUE);
        portDelayMs(1000);
        st7789_fill_color(COLOR_BRED);
        portDelayMs(1000);
        st7789_fill_color(COLOR_CYAN);
        portDelayMs(1000);
    }
}

void gui_task(void *argv)
{
    (void)(argv);
    lv_obj_t *lb_temp, *lb_humi;
    lv_obj_t *img_temp, *img_humi;

    lv_init();
    tft_lvgl_layer_init();
    
    gui_draw_temp_humi_icon_img(&img_temp, &img_humi);
    gui_set_temp_humi_val_lb(&lb_temp, img_temp, &lb_humi, img_humi);

    for (;;)
    {
#if USING_FREERTOS == 1
        if (xQueueReceive(TH_msgQ, TH_msgQ_buf, pdMS_TO_TICKS(5)) == pdTRUE)
#endif
        {
            gui_temp_humi_val_update(lb_temp, lb_humi, TH_msgQ_buf);
        }
        lv_task_handler();
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
  MX_I2C3_Init();
  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

#if USING_FREERTOS == 1
    TH_msgQ = xQueueCreate(1, sizeof(TH_msgQ_buf));

    xTaskCreate(led_blink_task, "LED", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
    xTaskCreate(temp_humi_smaple_task, "TH sensor", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    xTaskCreate(gui_task, "GUI", 1024, NULL, 3, NULL);

    vTaskStartScheduler();

#endif

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
            st7789_draw_point(200+i, 200, COLOR_GBLUE);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_SDMMC1;
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
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK;
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
    SysTick_Config(sysclk/1000);
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
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM17) {
    // LVGL tick need incerase in SystemTick(TIM17) handler
    lv_tick_inc(1);
  }
  /* USER CODE END Callback 1 */
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

#ifdef  USE_FULL_ASSERT
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
