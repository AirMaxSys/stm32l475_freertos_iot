#ifndef	_AHT10_H_
#define	_AHT10_H_

#ifdef	__cplusplus
extern "C" {
#endif

#include "i2c_soft.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

#define AHT10_BUFFER_SIZE	6u

#define AHT10_I2C_CLK_PORT      GPIOD
#define AHT10_I2C_CLK_PIN       GPIO_PIN_6
#define AHT10_I2C_SDA_PORT      GPIOC
#define AHT10_I2C_SDA_PIN       GPIO_PIN_1

typedef struct aht10 {
	uint16_t temp;
	uint16_t humi;
	uint8_t buf[AHT10_BUFFER_SIZE];
} aht10_t;

void aht10_init(aht10_t *paht);
void aht10_get_value(aht10_t *paht);
float aht10_temp(aht10_t *paht);
float aht10_humi(aht10_t *paht);

uint8_t aht10_status_reg(void);

#ifdef __cplusplus
}
#endif

#endif
