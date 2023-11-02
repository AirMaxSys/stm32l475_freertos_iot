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
#include "usart.h"
#include "usb_otg.h"
#include "adc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "lvgl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "proj_config.h"
#include "st7789v2.h"
#include "led.h"
#include "aht10.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void HAL_init_systick_for_RTX(void);

/*Extern demo task funtions declarations*/
extern int wiced_scan_main(void);
extern int tcp_socket_server_main(void);
extern int iot_main(void);
/*Extern test funtions declarations*/
extern void fatfs_test_task(void *arg);
extern void lcd_drv_test_task(void *argv);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vibrate_task(void *arg)
{
  for (;;) {
    // forward  - H L
    MOTOR_A_GPIO_Port->BSRR = MOTOR_A_Pin;
    MOTOR_B_GPIO_Port->BSRR = (MOTOR_B_Pin << 16);
    portDelayMs(100);
    // stop     - L L
    MOTOR_A_GPIO_Port->BSRR = (MOTOR_A_Pin << 16);
    MOTOR_A_GPIO_Port->BSRR = (MOTOR_A_Pin << 16);
    portDelayMs(20);
    // backward - L H
    MOTOR_A_GPIO_Port->BSRR = (MOTOR_A_Pin << 16);
    MOTOR_B_GPIO_Port->BSRR = MOTOR_B_Pin;
    portDelayMs(100);
    // stop     - L L
    MOTOR_A_GPIO_Port->BSRR = (MOTOR_A_Pin << 16);
    MOTOR_A_GPIO_Port->BSRR = (MOTOR_A_Pin << 16);
    portDelayMs(20);
    // break    - H H
    MOTOR_A_GPIO_Port->BSRR = MOTOR_A_Pin;
    MOTOR_B_GPIO_Port->BSRR = MOTOR_B_Pin;
    portDelayMs(1000);
  }
}

void beep_task(void *arg)
{
  for (;;) {
    portDelayMs(1000);
  }
}

void ir_task(void *arg)
{
  for (;;) {
    portDelayMs(1000);
  }
}

void led_task(void *arg)
{
  uint16_t ms = *(uint16_t *)arg;

  for (;;) {
    led_blue_blink(ms);
    led_green_blink(ms);
    led_red_blink(ms);
  }
}

void temp_sensor_task(void *arg)
{
  aht10_t temp_sensor;
  // Wait sensor start work
  portDelayMs(2000);

  aht10_init(&temp_sensor);
  for (;;) {
    aht10_get_value(&temp_sensor);
    // Smaple period
    portDelayMs(1000);
  }
}

void adc_task(void *arg)
{
  const uint8_t cali_temp1 = 30;
  const uint8_t cali_temp2 = 110;
  const uint16_t temp_cali_val1 = *(volatile uint16_t *)0x1FFF75A8;
  const uint16_t temp_cali_val2 = *(volatile uint16_t *)0x1FFF75CA;
  const uint16_t vref_cali_val = *(volatile uint16_t *)0x1FFF75AA;
  uint16_t adc_val = 0;
  float cpu_temp = 0.0;
  float coeff = (float)(cali_temp2 - cali_temp1)/(temp_cali_val2 - temp_cali_val1);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  for (;;) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 0xFFFF);
    adc_val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    // Vref 3.3 converting to 3
    adc_val *= (33.0/30);
    cpu_temp = coeff * (adc_val - temp_cali_val1) + 30;
    printf("Voltage:%d\tTemperature:%.1f\r\n", adc_val, cpu_temp);
    
    portDelayMs(500);
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
    MX_ADC1_Init();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    
    // st7789_init();
    led_init();

    // wiced_scan_main();
    // tcp_socket_server_main();
    // iot_main();
    static const uint16_t interval = 500;
  
    xTaskCreate(temp_sensor_task, "temperature", 256, NULL, 4, NULL);
    xTaskCreate(adc_task, "adc", 512, NULL, 4, NULL);
    xTaskCreate(led_task, "led", 128, (void *)&interval, 4, NULL);
//    xTaskCreate(vibrate_task, "vibrate", 128, NULL, 4, NULL);
    xTaskCreate(beep_task, "beep", 128, NULL, 6, NULL);
    xTaskCreate(ir_task, "ir", 128, NULL, 5, NULL);
    xTaskCreate(lcd_drv_test_task, "lcd", 1500, NULL, 3, NULL);
    vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        uint32_t ts = HAL_GetTick();
        st7789_fill_color( COLOR_RED );
        uint32_t fps = 1000 /( HAL_GetTick() - ts);
        led_blue_blink(500);
        st7789_fill_color( COLOR_BLUE );
        led_blue_blink(500);
        st7789_fill_color( COLOR_GREEN );
        led_blue_blink(500);
        st7789_fill_color( COLOR_WHITE );
        led_blue_blink(500);
        st7789_fill_color( COLOR_BLACK );
        led_blue_blink(500);
        printf("fps:%d freq:%dMhz\n", fps, HAL_RCC_GetSysClockFreq()/1000000);
    
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
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK
                              |RCC_PLLSAI1_ADC1CLK;
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
static void HAL_init_systick_for_RTX(void)
{
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    SysTick_Config(sysclk / 1000);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0U);
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
        printf("OS remain %.1fKb!\n", xPortGetFreeHeapSize() / 1024.0);
        fix_period = pdMS_TO_TICKS(1000);
    }
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
