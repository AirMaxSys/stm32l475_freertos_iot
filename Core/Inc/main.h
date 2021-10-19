/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_CS_Pin GPIO_PIN_3
#define SD_CS_GPIO_Port GPIOC
#define MOTOR_B_Pin GPIO_PIN_0
#define MOTOR_B_GPIO_Port GPIOA
#define MOTOR_A_Pin GPIO_PIN_1
#define MOTOR_A_GPIO_Port GPIOA
#define SD_CLK_Pin GPIO_PIN_5
#define SD_CLK_GPIO_Port GPIOA
#define SD_MISO_Pin GPIO_PIN_6
#define SD_MISO_GPIO_Port GPIOA
#define SD_MOSI_Pin GPIO_PIN_7
#define SD_MOSI_GPIO_Port GPIOA
#define WIFI_INT_Pin GPIO_PIN_5
#define WIFI_INT_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_2
#define BEEP_GPIO_Port GPIOB
#define LED_red_Pin GPIO_PIN_7
#define LED_red_GPIO_Port GPIOE
#define LED_green_Pin GPIO_PIN_8
#define LED_green_GPIO_Port GPIOE
#define LED_blue_Pin GPIO_PIN_9
#define LED_blue_GPIO_Port GPIOE
#define NRF_CLK_Pin GPIO_PIN_13
#define NRF_CLK_GPIO_Port GPIOB
#define NRF_MISO_Pin GPIO_PIN_14
#define NRF_MISO_GPIO_Port GPIOB
#define NRF_MOSI_Pin GPIO_PIN_15
#define NRF_MOSI_GPIO_Port GPIOB
#define AUDIO_PWR_Pin GPIO_PIN_15
#define AUDIO_PWR_GPIO_Port GPIOA
#define ICM_INT_Pin GPIO_PIN_0
#define ICM_INT_GPIO_Port GPIOD
#define WIFI_REG_ON_Pin GPIO_PIN_1
#define WIFI_REG_ON_GPIO_Port GPIOD
#define NRF_IRQ_Pin GPIO_PIN_3
#define NRF_IRQ_GPIO_Port GPIOD
#define NRF_CE_Pin GPIO_PIN_4
#define NRF_CE_GPIO_Port GPIOD
#define NRF_CS_Pin GPIO_PIN_5
#define NRF_CS_GPIO_Port GPIOD
#define AHT10_IIC_CLK_Pin GPIO_PIN_6
#define AHT10_IIC_CLK_GPIO_Port GPIOD
#define LCD_CS_Pin GPIO_PIN_7
#define LCD_CS_GPIO_Port GPIOD
#define LCD_CLK_Pin GPIO_PIN_3
#define LCD_CLK_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_5
#define LCD_SDA_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_6
#define LCD_RST_GPIO_Port GPIOB
#define LCD_PWR_Pin GPIO_PIN_7
#define LCD_PWR_GPIO_Port GPIOB
#define I2C_SOFT_SDA_Port GPIOC
#define I2C_SOFT_SDA_Pin GPIO_PIN_1
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
