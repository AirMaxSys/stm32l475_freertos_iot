/******************************************************
* Specific platform porting
*******************************************************/
#include "FreeRTOSConfig.h"
#include "wwd_platform_common.h"

#include "stm32l475xx.h"
#include "stm32l4xx_hal.h"

// Control GPIO port - AP6181 reset?
const platform_gpio_t wifi_control_pins[] = {
    [WWD_PIN_RESET] = {GPIOD, 1},
};

// PD2     ------> SDMMC1_CMD
// PANDORA board SDIO GPIO port
const platform_gpio_t wifi_sdio_pins[] = {
    [WWD_PIN_SDIO_OOB_IRQ] = { GPIOC,  5 },
    [WWD_PIN_SDIO_CLK    ] = { GPIOC, 12 },
    [WWD_PIN_SDIO_CMD    ] = { GPIOD,  2 },
    [WWD_PIN_SDIO_D0     ] = { GPIOC,  8 },
    [WWD_PIN_SDIO_D1     ] = { GPIOC,  9 },
    [WWD_PIN_SDIO_D2     ] = { GPIOC, 10 },
    [WWD_PIN_SDIO_D3     ] = { GPIOC, 11 },
};

// In this port just need to set SDIO and DMA IRQ priority
void platform_init_peripheral_irq_priorities(void)
{
    // IRQ priority can not beyond configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    // because we will call FreeRTOS ISR API form IRQ handler
    HAL_NVIC_SetPriority(SDMMC1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
}


