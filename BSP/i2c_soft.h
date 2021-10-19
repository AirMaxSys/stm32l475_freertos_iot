#ifndef _I2C_SOFT_H_
#define	_I2C_SOFT_H_

#include "main.h"
#include "common.h"
#include <stdint.h>

typedef enum {
    I2C_SOFT_WRITE_MODE = 0,
    I2C_SOFT_READ_MODE,
} i2c_soft_mode_enum_t;

typedef enum {
    I2C_SOFT_ERR_NONE = 0,
    I2C_SOFT_ERR_INIT_INSTENCE,
    I2C_SOFT_ERR_GPIO_PORT,
    I2C_SOFT_ERR_ADDR_LEN,
    I2C_SOFT_ERR_ADDR_AF,       // address ACK failure
    I2C_SOFT_ERR_TX_AF,         // Tx ACK failure
} i2c_soft_err_enum_t;

typedef struct i2c_soft {
    gpio_t *sda;
    gpio_t *scl;
    uint16_t slave_addr;
    uint8_t addr_len;
} i2c_soft_t;

i2c_soft_err_enum_t i2c_soft_init(i2c_soft_t *i2c);
void i2c_soft_set_slave_addr(uint8_t addr);
i2c_soft_err_enum_t i2c_soft_send_datas(uint8_t *buffer, uint16_t len, uint8_t stop_en);
i2c_soft_err_enum_t i2c_soft_recv_datas(uint8_t *buffer, uint16_t len);

#endif
