#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include "stm32l475xx.h"

typedef struct gpio {
    GPIO_TypeDef *port;
    uint16_t pin;
} gpio_t;

#endif
