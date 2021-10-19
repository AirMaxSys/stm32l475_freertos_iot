#include "btn.h"

void btn_gpio_init(void)
{
    GPIO_InitTypeDef gpio_handler;

    // GPIO ports clock enable
    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio_handler.Pin = BTN_two_Pin|BTN_one_Pin|BTN_zero_Pin;
    gpio_handler.Mode = GPIO_MODE_INPUT;
    gpio_handler.Pull = GPIO_PULLUP;
    gpio_handler.Speed = 0;     // Input mode not used
    gpio_handler.Alternate = 0; // Input mode not used
    HAL_GPIO_Init(GPIOD, &gpio_handler);
}
