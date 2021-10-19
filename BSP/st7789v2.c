/*!
* @file		st7789v2.c - st7789v2 LCD driver
* @brief	TFT display resolution 240*320, color 262K RGB(666)
* @note		#1 use 4lines SPI bus to communicate and st HAL spi driver lib
*			#2 use DMA transmit data
*			#3 IO pin: SCL(SPI clock) SDA(data out/in) CSX(SPI chip select) WRX(data/command) RST(reset) PWR(power)	
*			#4 when multiple byte read, stm32 SPI can't shift dummy bit(dummy cycle), so that CMD 0x4h(read display id)
*				can not succeed!!!
* @author	AirMaxSys
* @date		2021/03/13	- first edit
*/

#include "st7789v2.h"
#include "proj_config.h"
#include "main.h"

extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi3;

#define ST7789_FILL_COLOR_SIZE  (ST7789_W) * (ST7789_H) / 10

#define st7789_spi_handler      hspi3
#define st7789_dma_tx_handler   hdma_spi3_tx

static inline void st7789_wait_dma_tc(void)
{
    while (__HAL_DMA_GET_FLAG(&st7789_dma_tx_handler, DMA_FLAG_TC2) == RESET)
        ;
}

static void st7789_clear_spi_dma_flag_state(SPI_HandleTypeDef *hspi, DMA_HandleTypeDef *hdma)
{
    // Clear DMA half transfer and transfer complete flags
    hdma->DmaBaseAddress->IFCR = DMA_ISR_HTIF1 << (hdma->ChannelIndex & 0x1CU);
    hdma->DmaBaseAddress->IFCR = DMA_ISR_TCIF1 << (hdma->ChannelIndex & 0x1CU);
    // Reset SPI and DMA handler state
    hspi->State = HAL_SPI_STATE_READY;
    hdma->State = HAL_DMA_STATE_READY;
    // According to HAL_DMA_IRQHandler() funtion must unlock DMA handler
    __HAL_UNLOCK(hdma);
}

static inline void st7789_pin_write(GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static void st7789_write_cmd(uint8_t cmd)
{
    st7789_pin_write(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&st7789_spi_handler, &cmd, 1);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void st7789_write_data(uint8_t data)
{
    st7789_pin_write(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&st7789_spi_handler, &data, 1);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void st7789_write_half_word(const uint16_t data)
{
    uint8_t buf[2] = {data >> 8, data};
    st7789_pin_write(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&st7789_spi_handler, buf, 2);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void st7789_power_on(void)
{
    st7789_pin_write(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_SET);
}

void st7789_power_off(void)
{
    st7789_pin_write(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);
}

void st7789_enter_sleep(void)
{
    st7789_pin_write(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);
    portDelayMs(5);
    st7789_write_cmd(0x10);
}

void st7789_exit_sleep(void)
{
    st7789_pin_write(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_SET);
    portDelayMs(5);
    st7789_write_cmd(0x11);
    portDelayMs(120);
}

void st7789_reset(void)
{
    st7789_pin_write(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    portDelayMs(100);
    st7789_pin_write(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
}

void st7789_init(void)
{
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

void st7789_transfer_datas(uint8_t buffer[], uint32_t bufsize)
{
    uint8_t *pbuf = buffer;
    const uint16_t chunk_size = 65535u;

    // Setting DC pin high to transmit data and select SPI CS
    st7789_pin_write(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

    // ST HAL LIB API of DMA transmisson only allow data size less than 65535
    if (bufsize > chunk_size) {
        while (bufsize >= chunk_size) {
            HAL_SPI_Transmit_DMA(&st7789_spi_handler, pbuf, chunk_size);
            st7789_wait_dma_tc();
            st7789_clear_spi_dma_flag_state(&st7789_spi_handler, &st7789_dma_tx_handler);

            bufsize -= chunk_size;
            pbuf += chunk_size;
        }
        if (bufsize > 0) {
            HAL_SPI_Transmit_DMA(&st7789_spi_handler, pbuf, bufsize);
            st7789_wait_dma_tc();
            st7789_clear_spi_dma_flag_state(&st7789_spi_handler, &st7789_dma_tx_handler);
        }
    } else {
        HAL_SPI_Transmit_DMA(&st7789_spi_handler, pbuf, bufsize);
        st7789_wait_dma_tc();
        st7789_clear_spi_dma_flag_state(&st7789_spi_handler, &st7789_dma_tx_handler);
    }
    
    // Unselect SPI CS pin
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

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
    st7789_pin_write(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

#if 1
    for (uint8_t i = 0; i < 20; ++i) {
        if (HAL_SPI_Transmit_DMA(&st7789_spi_handler, buf, ST7789_FILL_COLOR_SIZE) != HAL_OK)
            __NOP();
        // Polling untill transmission done
        st7789_wait_dma_tc();
        // Clear flags and reset state
        st7789_clear_spi_dma_flag_state(&st7789_spi_handler, &st7789_dma_tx_handler);
    }
#else
    for (uint16_t i = 0; i < ST7789_H; ++i)
        for (uint16_t j = 0; j < ST7789_W; ++j)
            HAL_SPI_Transmit_IT(&st7789_spi_handler, data, 2);
#endif
    // Unselect SPI CS pin
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void st7789_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    st7789_set_window(x, y, x, y);
    st7789_write_half_word(color);
}

void st7789_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (x1 > ST7789_W || x2 > ST7789_W || y1 > ST7789_H || y2 > ST7789_H)
        return;
    
    // Fast drawing line
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

        st7789_pin_write(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
        st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit_DMA(&st7789_spi_handler, buf, 2*t);
        st7789_wait_dma_tc();
        st7789_clear_spi_dma_flag_state(&st7789_spi_handler, &st7789_dma_tx_handler);
        st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
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

void st7789_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    st7789_draw_line(x1, y1, x2, y1);
    st7789_draw_line(x1, y1, x1, y2);
    st7789_draw_line(x1, y2, x2, y2);
    st7789_draw_line(x2, y1, x2, y2);
}
