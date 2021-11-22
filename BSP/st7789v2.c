/*!
* @file     st7789v2.c
* @brief    Driver of LCD controler st7789v2 and support basic drawing API
* @author   AirMaxsys
* @note		
*           #1 Using SPI user registed callback function setup USE_HAL_SPIREGISTER_CALLBACKS is 1 in stm32_l4xx_hal_conf.h
*           #2 GPIO pin: SCL(SPI clock) SDA(data out/in) CSX(SPI chip select) WRX(data/command) RST(reset) PWR(power)	
*           #3 Using DMA transmit data
*
* @date     2021/03/13  - First edit
*           2021/11/21  - Add SPI and DMA initialization
*                       - Add SPI and DMA IRQ handler
*                       - Remove DMA waiting and DMA IRQ flags clear funtions
*                       - Add FreeRTOS counting semaphere to wait DMA TX complete
*/

#include "st7789v2.h"
#include "proj_config.h"
#include "stm32l475xx.h"
#include "stm32l4xx_hal.h"

#if USING_FREERTOS == 1
#include "semphr.h"
#endif

/* ST7789 SPI bus related GOIO definations*/
#define ST7789_CS_PORT      GPIOD
#define ST7789_CS_PIN       GPIO_PIN_7
#define ST7789_CLK_PORT     GPIOB
#define ST7789_CLK_PIN      GPIO_PIN_3
#define ST7789_MOSI_PORT    GPIOB
#define ST7789_MOSI_PIN     GPIO_PIN_5
/* ST7789 device related GPIO definations*/
#define ST7789_DC_PORT      GPIOB
#define ST7789_DC_PIN       GPIO_PIN_4
#define ST7789_RST_PORT     GPIOB
#define ST7789_RST_PIN      GPIO_PIN_6
#define ST7789_PWR_PORT     GPIOB
#define ST7789_PWR_PIN      GPIO_PIN_7
/* ST7789 screen size defination*/
#define ST7789_FILL_COLOR_SIZE  (ST7789_W) * (ST7789_H) / 10

/* ST7789 SPI DMA transmit done IRQ callback function*/
void st7789_spi_dmatx_complete_cb(SPI_HandleTypeDef *hdma);

/* ST7789 SPI cad DMA local hander*/
static SPI_HandleTypeDef hspi_st7789;
static DMA_HandleTypeDef hdmatx_st7789;

/* ST7789 local varaibles*/
#if USING_FREERTOS == 1
static SemaphoreHandle_t sem_dmatx_cmp;
#else
static volatile uint8_t wait_dmatx_cmp;
#endif

/* ST7789 device GPIO, SPI and DMA initialize functions*/

static void st7789_device_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin     = ST7789_DC_PIN | ST7789_RST_PIN | ST7789_PWR_PIN;
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_PULLUP;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void st7789_spi_cs_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin     = ST7789_CS_PIN;
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_PULLUP;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void st7789_spi_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();

    GPIO_InitStruct.Pin         = ST7789_CLK_PIN|ST7789_MOSI_PIN;
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull        = GPIO_PULLUP;
    GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate   = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Initialize SPI hal handler using SPI3 to connect to TFT
    hspi_st7789.Instance                = SPI3;
    hspi_st7789.Init.Mode               = SPI_MODE_MASTER;
    hspi_st7789.Init.Direction          = SPI_DIRECTION_1LINE;
    hspi_st7789.Init.DataSize           = SPI_DATASIZE_8BIT;
    hspi_st7789.Init.CLKPolarity        = SPI_POLARITY_HIGH;
    hspi_st7789.Init.CLKPhase           = SPI_PHASE_2EDGE;
    hspi_st7789.Init.NSS                = SPI_NSS_SOFT;
    hspi_st7789.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_2;
    hspi_st7789.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    hspi_st7789.Init.TIMode             = SPI_TIMODE_DISABLE;
    hspi_st7789.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
    hspi_st7789.Init.NSSPMode           = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi_st7789) != HAL_OK) {
        // TODO: LOG error
        __NOP();
    }
    HAL_NVIC_SetPriority(SPI3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);

    // Register callback funtions
    HAL_SPI_RegisterCallback(&hspi_st7789, HAL_SPI_TX_COMPLETE_CB_ID, st7789_spi_dmatx_complete_cb);
    
    // Create DMA TX completion counting semaphore
#if USING_FREERTOS == 1
    sem_dmatx_cmp = xSemaphoreCreateCounting(1, 0);
    if (!sem_dmatx_cmp) {
        // TODO:LOG error
        __NOP();
    }
#endif
}

static void st7789_dma_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdmatx_st7789.Instance                  = DMA2_Channel2;
    hdmatx_st7789.Init.Request              = DMA_REQUEST_3;
    hdmatx_st7789.Init.Direction            = DMA_MEMORY_TO_PERIPH;
    hdmatx_st7789.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
    hdmatx_st7789.Init.MemInc               = DMA_MINC_ENABLE;
    hdmatx_st7789.Init.Mode                 = DMA_NORMAL;
    hdmatx_st7789.Init.PeriphDataAlignment  = DMA_PDATAALIGN_HALFWORD;
    hdmatx_st7789.Init.PeriphInc            = DMA_PINC_DISABLE;
    hdmatx_st7789.Init.Priority             = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdmatx_st7789);

    __HAL_LINKDMA(&hspi_st7789, hdmatx, hdmatx_st7789);

    HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 1);
    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
}

/* ST7789 device GPIO control functions*/

static inline void st7789_pin_write(GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static inline void st7789_power_on(void)
{
    st7789_pin_write(ST7789_PWR_PORT, ST7789_PWR_PIN, GPIO_PIN_SET);
}

static inline void st7789_power_off(void)
{
    st7789_pin_write(ST7789_PWR_PORT, ST7789_PWR_PIN, GPIO_PIN_RESET);
}

static inline void st7789_select(void)
{
    st7789_pin_write(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_RESET);
}

static inline void st7789_unselect(void)
{
    st7789_pin_write(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_SET);
}

static inline void st7789_dc_cmd(void)
{
    st7789_pin_write(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_RESET);
}

static inline void st7789_dc_data(void)
{
    st7789_pin_write(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET);
}

/* ST7789 write data or command functions*/

static void st7789_write_cmd(uint8_t cmd)
{
    st7789_dc_cmd();
    st7789_select();
    HAL_SPI_Transmit(&hspi_st7789, &cmd, 1, 0xFF);
    st7789_unselect();
}

static void st7789_write_data(uint8_t data)
{
    st7789_dc_data();
    st7789_select();
    HAL_SPI_Transmit(&hspi_st7789, &data, 1, 0xFF);
    st7789_unselect();
}

static void st7789_write_half_word(const uint16_t data)
{
    uint8_t buf[2] = {data >> 8, data};

    st7789_dc_data();
    st7789_select();
    HAL_SPI_Transmit(&hspi_st7789, buf, 2, 0xFF);
    st7789_unselect();
}

/* ST7789 power control functions*/

void st7789_enter_sleep(void)
{
    st7789_power_off();
    portDelayMs(5);
    st7789_write_cmd(0x10);
}

void st7789_exit_sleep(void)
{
    st7789_power_on();
    portDelayMs(5);
    st7789_write_cmd(0x11);
    portDelayMs(120);
}

void st7789_reset(void)
{
    st7789_pin_write(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET);
    portDelayMs(100);
    st7789_pin_write(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET);
}

/**
 * @brief ST7789 device initialization
 */
void st7789_init(void)
{
    st7789_device_gpio_init();
    st7789_spi_cs_init();
    st7789_spi_init();
    st7789_dma_init();

    st7789_reset();
    /* Memory Data Access Control */
    st7789_write_cmd(0x36);
    st7789_write_data(0x00);
    /* Seting RGB565 */
    st7789_write_cmd(0x3A);
    st7789_write_data(0x65);
    /* Porch Setting */
    st7789_write_cmd(0xB2);
    st7789_write_data(0x0C);
    st7789_write_data(0x0C);
    st7789_write_data(0x00);
    st7789_write_data(0x33);
    st7789_write_data(0x33);
    /*  Gate Control */
    st7789_write_cmd(0xB7);
    st7789_write_data(0x35);
    /* VCOM Setting */
    st7789_write_cmd(0xBB);
    st7789_write_data(0x19);
    /* LCM Control */
    st7789_write_cmd(0xC0);
    st7789_write_data(0x2C);
    /* VDV and VRH Command Enable */
    st7789_write_cmd(0xC2);
    st7789_write_data(0x01);
    /* VRH Set */
    st7789_write_cmd(0xC3);
    st7789_write_data(0x12);
    /* VDV Set */
    st7789_write_cmd(0xC4);
    st7789_write_data(0x20);
    /* Frame Rate Control in Normal Mode */
    st7789_write_cmd(0xC6);
    st7789_write_data(0x0F);
    /* Power Control 1 */
    st7789_write_cmd(0xD0);
    st7789_write_data(0xA4);
    st7789_write_data(0xA1);
    /* Positive Voltage Gamma Control */
    st7789_write_cmd(0xE0);
    st7789_write_data(0xD0);
    st7789_write_data(0x04);
    st7789_write_data(0x0D);
    st7789_write_data(0x11);
    st7789_write_data(0x13);
    st7789_write_data(0x2B);
    st7789_write_data(0x3F);
    st7789_write_data(0x54);
    st7789_write_data(0x4C);
    st7789_write_data(0x18);
    st7789_write_data(0x0D);
    st7789_write_data(0x0B);
    st7789_write_data(0x1F);
    st7789_write_data(0x23);
    /* Negative Voltage Gamma Control */
    st7789_write_cmd(0xE1);
    st7789_write_data(0xD0);
    st7789_write_data(0x04);
    st7789_write_data(0x0C);
    st7789_write_data(0x11);
    st7789_write_data(0x13);
    st7789_write_data(0x2C);
    st7789_write_data(0x3F);
    st7789_write_data(0x44);
    st7789_write_data(0x51);
    st7789_write_data(0x2F);
    st7789_write_data(0x1F);
    st7789_write_data(0x1F);
    st7789_write_data(0x20);
    st7789_write_data(0x23);
    /* Display Inversion On */
    st7789_write_cmd(0x21);
    /* Sleep Out */
    st7789_write_cmd(0x11);
    /* wait for power stability */
    portDelayMs(100);

    // st7789_fill_color(COLOR_WHITE);

    /* Display on*/
    st7789_power_on();
    st7789_write_cmd(0x29);
}

/**
 * @brief Set up drawing window area
 */
void st7789_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    st7789_write_cmd(0x2a);
    st7789_write_data(x1 >> 8);
    st7789_write_data(x1);
    st7789_write_data(x2 >> 8);
    st7789_write_data(x2);

    st7789_write_cmd(0x2b);
    st7789_write_data(y1 >> 8);
    st7789_write_data(y1);
    st7789_write_data(y2 >> 8);
    st7789_write_data(y2);

    st7789_write_cmd(0x2C);
}

/**
 * @brief Host transfer datas to LCD controler using SPI bus with DMA
 * @param buffer Transmission data buffer
 * @param bufsize Transmission data buffer size
 */
void st7789_transfer_datas(uint8_t buffer[], uint32_t bufsize)
{
    uint8_t *pbuf = buffer;
    const uint16_t chunk_size = 65535u;

    // Setting DC pin high to transmit data and select SPI CS
    st7789_dc_data();
    st7789_select();

    // ST HAL LIB API of SPI transmisson only allow data size less than 65535
    if (bufsize > chunk_size) {
        while (bufsize > chunk_size) {
            HAL_SPI_Transmit_DMA(&hspi_st7789, pbuf, chunk_size);
            // Using FreeRTOS counting semaphore to wait transmit done
            // Then start transferring next data chunk
#if USING_FREERTOS == 1
            xSemaphoreTake(sem_dmatx_cmp, portMAX_DELAY);
#else
            // Using global variable to wait transmit done
            while (!wait_dmatx_cmp);
            wait_dmatx_cmp = 0;
#endif
            bufsize -= chunk_size;
            pbuf += chunk_size;
        }
    }
    HAL_SPI_Transmit_DMA(&hspi_st7789, pbuf, bufsize);
#if USING_FREERTOS == 1
    xSemaphoreTake(sem_dmatx_cmp, portMAX_DELAY);
#else
    while (!wait_dmatx_cmp);
    wait_dmatx_cmp = 0;
#endif
    
    // Unselect SPI CS pin
    st7789_unselect();
}

/**
 * @brief Fill a color on the screen
 * @param color RGB565 hex value of color
*/
void st7789_fill_color(uint16_t color)
{
    uint8_t data[2] = {0};
    uint8_t buf[ST7789_FILL_COLOR_SIZE] = {0};

    data[0] = color >> 8;
    data[1] = color;

    st7789_set_window(0, 0, ST7789_W - 1, ST7789_H - 1);

    // Fill transmission buffer
    for (uint16_t i = 0; i < ST7789_FILL_COLOR_SIZE / 2; ++i) {
        buf[i * 2] = data[0];
        buf[i * 2 + 1] = data[1];
    }

    // Setting DC pin high to transmit data and select SPI CS
    st7789_dc_data();
    st7789_select();
    for (uint8_t i = 0; i < 20; ++i) {
        HAL_SPI_Transmit_DMA(&hspi_st7789, buf, ST7789_FILL_COLOR_SIZE);
#if USING_FREERTOS == 1
        xSemaphoreTake(sem_dmatx_cmp, portMAX_DELAY);
#else
        while (!wait_dmatx_cmp);
        wait_dmatx_cmp = 0;
#endif
    }
    // Unselect SPI CS pin
    st7789_unselect();
}

/**
 * @brief Draw a colored point.
 * @param x x coordinate of point
 * @param y y coordinate of point
 * @param color RGB565 hex value of color
*/
void st7789_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    st7789_set_window(x, y, x, y);
    st7789_write_half_word(color);
}

/**
 * @brief Draw a line on the screen.
 * @param x1 x coordinate of one point
 * @param y1 y coordinate of one point
 * @param x2 x coordinate of another point
 * @param y2 y coordinate of another point
*/
void st7789_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (x1 > ST7789_W || x2 > ST7789_W || y1 > ST7789_H || y2 > ST7789_H)
        return;
    
    // Draw a line fast
    if (y1 == y2 || x1 == x2) {
        uint16_t t = 0;
        uint8_t buf[ST7789_W*2] = {0};

        if (y1 == y2) {
            t = (x2 > x1) ? (x2 - x1) : (x1 - x2);
        } else {
            t = (y2 > y1) ? (y2 - y1) : (y1 - y2);
        }

        for (uint16_t i = 0; i < t; ++i) {
            buf[2*i] = COLOR_BLACK >> 8;
            buf[2*i+1] = COLOR_BLACK;
        }

        st7789_set_window(x1, y1, x2, y2);

        st7789_dc_data();
        st7789_select();
        HAL_SPI_Transmit_DMA(&hspi_st7789, buf, 2*t);
#if USING_FREERTOS == 1
        xSemaphoreTake(sem_dmatx_cmp, portMAX_DELAY);
#else
        while (!wait_dmatx_cmp);
        wait_dmatx_cmp = 0;
#endif
        st7789_unselect();
    } else {
        int delta, delta_x, delta_y, row, colum, inc_x, inc_y;
        int xerr = 0, yerr = 0;
        
        delta_x = x2 - x1;
        delta_y = y2 - y1;
        row = x1;
        colum = y1;

        if (delta_x > 0) {
            inc_x = 1;
        } else if (delta_x == 0) {
            inc_x = 0;
        } else {
            inc_x = -1;
            delta_x = -delta_x;
        }

        if (delta_y > 0) {
            inc_y = 1;
        } else if (delta_y == 0) {
            inc_y = 0;
        } else {
            inc_y = -1;
            delta_y = -delta_y;
        }

        delta = (delta_x > delta_y) ? delta_x : delta_y;

        for (int i = 0; i <= delta + 1; ++i) {
            st7789_draw_point(row, colum, COLOR_BLACK);
            xerr += delta_x;
            yerr += delta_y;
            if (xerr > delta) {
                xerr -= delta;
                row += inc_x;
            }
            if (yerr > delta) {
                yerr -= delta;
                colum += inc_y;
            }
        }
    }
}

/**
 * @brief Draw a rectangle with appropriate coordinate
*/
void st7789_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    st7789_draw_line(x1, y1, x2, y1);
    st7789_draw_line(x1, y1, x1, y2);
    st7789_draw_line(x1, y2, x2, y2);
    st7789_draw_line(x2, y1, x2, y2);
}
/**
 * @brief SPI transmit complete callback function.This function will be called 
 * after the DMA transfer completion interrupt is triggered.
*/
void st7789_spi_dmatx_complete_cb(SPI_HandleTypeDef *hspi)
{
    if ((hspi == &hspi_st7789)) {
#if USING_FREERTOS == 1
        BaseType_t task_need_woken;
        xSemaphoreGiveFromISR(sem_dmatx_cmp, &task_need_woken);
#else
        wait_dmatx_cmp = 1;
#endif
    }
}

/**
 * @brief SPI3 interrupt handle function
*/
void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi_st7789);
}

/**
 * @brief DMA2 channel2 interrupt handle function
*/
void DMA2_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdmatx_st7789);
}
