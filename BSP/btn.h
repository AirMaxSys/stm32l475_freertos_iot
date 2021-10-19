#ifndef _BTN_H_
#define _BTN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx.h"

#define SYS_BTN_NUMS    3u

#define BTN_two_Pin GPIO_PIN_8
#define BTN_two_GPIO_Port GPIOD
#define BTN_one_Pin GPIO_PIN_9
#define BTN_one_GPIO_Port GPIOD
#define BTN_zero_Pin GPIO_PIN_10
#define BTN_zero_GPIO_Port GPIOD

#ifdef __cpluscplus
}
#endif

#endif