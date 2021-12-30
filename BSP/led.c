#include "led.h"
#include "stm32l4xx_hal.h"
#include "proj_config.h"

#define LED_RED_PORT    GPIOE
#define LED_RED_PIN     GPIO_PIN_7 
#define LED_GRE_PORT    GPIOE
#define LED_GRE_PIN     GPIO_PIN_8
#define LED_BLU_PORT    GPIOE
#define LED_BLU_PIN     GPIO_PIN_9

void led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, LED_RED_PIN | LED_GRE_PIN | LED_BLU_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = LED_RED_PIN | LED_GRE_PIN | LED_BLU_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void led_red_blink(uint16_t ms)
{
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
    portDelayMs(ms);    
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
    portDelayMs(ms);    
}

void led_blue_blink(uint16_t ms)
{
    HAL_GPIO_WritePin(LED_BLU_PORT, LED_BLU_PIN, GPIO_PIN_RESET);
    portDelayMs(ms);    
    HAL_GPIO_WritePin(LED_BLU_PORT, LED_BLU_PIN, GPIO_PIN_SET);
    portDelayMs(ms);    
}

void led_green_blink(uint16_t ms)
{
    HAL_GPIO_WritePin(LED_GRE_PORT, LED_GRE_PIN, GPIO_PIN_RESET);
    portDelayMs(ms);    
    HAL_GPIO_WritePin(LED_GRE_PORT, LED_GRE_PIN, GPIO_PIN_SET);
    portDelayMs(ms);    
}