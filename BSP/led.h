#ifndef _LED_H_
#define _LED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void led_init(void);
void led_red_blink(uint16_t ms);
void led_blue_blink(uint16_t ms);
void led_green_blink(uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif
