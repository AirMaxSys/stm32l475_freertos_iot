#ifndef _COMMON_H_
#define _COMMON_H_

#include "main.h"
#include <stdint.h>

typedef struct gpio {
    GPIO_TypeDef *port;
    uint32_t pin;
} gpio_t;

#endif
