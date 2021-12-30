#include "i2c_soft.h"
#include "stm32l4xx_hal.h"
#include "proj_config.h"

// Definations of GPIO output/input methods
#define PIN_HI(port, pin)   HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)
#define PIN_LO(port, pin)   HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
#define PIN_RD(port, pin)   (HAL_GPIO_ReadPin((port), (pin)) == GPIO_PIN_SET) ? 1 : 0

// Definations of I2C SDA GPIO mode
#define SDA_OUT(port, pin)   do {    \
    uint8_t pos = 0;    \
    while (((pin) >> pos) != 0x1UL) pos++; \
    port->MODER &= (~(0x3UL<<(pos*2)));    \
    port->MODER |= (0x1UL<<(pos*2));   \
} while (0)

#define SDA_IN(port, pin)    do {    \
    uint8_t pos = 0;    \
    while (((pin) >> pos) != 0x1UL) pos++; \
    port->MODER &= (~(0x3UL<<(pos*2)));    \
    port->MODER |= (0x0UL<<(pos*2));   \
} while (0)

// Local I2C object pointer
static i2c_soft_t *p_i2c_sf;

static void delay(void)
{
    // Control I2C speed
#if 1
    volatile uint32_t i = 10000;
    while (i--) __asm volatile ("nop");
#else
    volatile uint16_t i, j;
    for (i = 0; i < 100; ++i)
        for (j = 0; j < 100; ++j)
            __asm volatile("nop");
#endif
}

i2c_soft_err_enum_t i2c_soft_init(i2c_soft_t *i2c)
{
	if (!i2c)
		return I2C_SOFT_ERR_INIT_INSTENCE;
	if (!i2c->sda || !i2c->scl)
		return I2C_SOFT_ERR_GPIO_PORT;
	if (i2c->addr_len != 7 && i2c->addr_len != 10)
		return I2C_SOFT_ERR_ADDR_LEN;

    // GPIO clock enable
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {0};

    /* globle i2c point to soft instence*/
    p_i2c_sf = i2c;

    /* init SDA and SCL gpio port*/
    HAL_GPIO_DeInit(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    HAL_GPIO_DeInit(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

    /* configure SDA and SCL GPIO pins must set in open drain mode*/
    gpio_init.Pin = p_i2c_sf->sda->pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(p_i2c_sf->sda->port, &gpio_init);
    gpio_init.Pin = p_i2c_sf->scl->pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(p_i2c_sf->scl->port, &gpio_init);

    /* set SDA and SCL to high level*/
    PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

    return I2C_SOFT_ERR_NONE;
}

static void i2c_soft_gen_start_cond(void)
{
    SDA_OUT(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
    delay();
    PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    delay();
    PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
}

static void i2c_soft_gen_stop_cond(void)
{
    SDA_OUT(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
    delay();
    PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    delay();
    PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
}


static void i2c_soft_send_ack(void)
{
    PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
    SDA_OUT(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
    PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
}

static void i2c_soft_send_nack(void)
{
    PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
    SDA_OUT(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
    PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
}

static uint8_t i2c_soft_wait_ack(void)
{
    uint8_t res = 0;

    SDA_IN(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
    res = PIN_RD(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

    return res;
}

static int8_t i2c_soft_send_byte(uint8_t byte)
{
    SDA_OUT(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

    for (int8_t i = 7; i >= 0; --i) {
        if ((byte & 0x80) >> 7)
            PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
        else 
            PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
        byte <<= 1;
        PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
        PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
    }
    if (i2c_soft_wait_ack() != 0) {
        return -1;
    }
    return 0;
}

static uint8_t i2c_soft_recv_byte(uint8_t ack_en)
{
    uint8_t ret = 0;

    SDA_IN(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
    for (int8_t i = 7; i >= 0; --i) {
        PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
        PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
        ret <<= 1;
        if (PIN_RD(p_i2c_sf->sda->port, p_i2c_sf->sda->pin))
            ret++;
    }
    if (ack_en)
        i2c_soft_send_ack();
    else
        i2c_soft_send_nack();

    return ret;
}

static int8_t i2c_soft_master_send_addr7(i2c_soft_mode_enum_t mode)
{
    uint8_t addr = (p_i2c_sf->slave_addr << 1) + (uint8_t)mode;
    
    // send slave address
    return i2c_soft_send_byte(addr);
    
}

void i2c_soft_set_slave_addr(uint8_t addr)
{
    p_i2c_sf->slave_addr = addr;
}

i2c_soft_err_enum_t i2c_soft_send_datas(uint8_t *buffer, uint16_t len, uint8_t stop_en)
{
    uint8_t res =  I2C_SOFT_ERR_NONE;

#ifdef USING_FREERTOS
    portDISABLE_INTERRUPTS();
#else
   __disable_irq();
#endif

    // master generete start condition
    i2c_soft_gen_start_cond();

    // send slave address
    if (i2c_soft_master_send_addr7(I2C_SOFT_WRITE_MODE) != 0) {
        res = I2C_SOFT_ERR_ADDR_AF;
        i2c_soft_gen_stop_cond();
        goto exit;
    }

    // send datas
    for (uint16_t i = 0; i < len; ++i) {
        if (i2c_soft_send_byte(buffer[i]) != 0) {
            res = I2C_SOFT_ERR_TX_AF;
            i2c_soft_gen_stop_cond();
            goto exit;
        }
    }

    // master generete stop condition
    if (stop_en)
        i2c_soft_gen_stop_cond();

exit:
#ifdef USING_FREERTOS
    portENABLE_INTERRUPTS();
#else
    __enable_irq();
#endif

    return res;
}

i2c_soft_err_enum_t i2c_soft_recv_datas(uint8_t *rxbuf, uint16_t len)
{
    uint8_t res =  I2C_SOFT_ERR_NONE;
    
    if (0 == len)   return res;

#ifdef USING_FREERTOS
    portDISABLE_INTERRUPTS();
#else
   __disable_irq();
#endif

    // Master device generete start condition signal
    i2c_soft_gen_start_cond();

    // Send slave device address
    if (i2c_soft_master_send_addr7(I2C_SOFT_READ_MODE) != 0) {
        res = I2C_SOFT_ERR_ADDR_AF;
        i2c_soft_gen_stop_cond();
        goto exit;
    }

    // receive datas
    for (uint16_t i = 0; i < len; ++i) {
        // master receive last byte
        if (i == len - 1) {
            // fetch data and send nack
            rxbuf[i] = i2c_soft_recv_byte(0);
            i2c_soft_gen_stop_cond();
            break;
        }
        rxbuf[i] = i2c_soft_recv_byte(1);
    }

exit:
#ifdef USING_FREERTOS
    portENABLE_INTERRUPTS();
#else
    __enable_irq();
#endif

    return res;
}
