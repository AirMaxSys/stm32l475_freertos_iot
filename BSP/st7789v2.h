#ifndef _ST7789V2_H_
#define _ST7789V2_H_

#ifdef  _cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ST7789_W    240
#define ST7789_H    240

#define COLOR_WHITE            0xFFFF
#define COLOR_BLACK            0x0000
#define COLOR_BLUE             0x001F
#define COLOR_BRED             0XF81F
#define COLOR_GRED             0XFFE0
#define COLOR_GBLUE            0X07FF
#define COLOR_RED              0xF800
#define COLOR_MAGENTA          0xF81F
#define COLOR_GREEN            0x07E0
#define COLOR_CYAN             0x7FFF
#define COLOR_YELLOW           0xFFE0
#define COLOR_BROWN            0XBC40
#define COLOR_BRRED            0XFC07
#define COLOR_GRAY             0X8430
#define COLOR_GRAY175          0XAD75
#define COLOR_GRAY151          0X94B2
#define COLOR_GRAY187          0XBDD7
#define COLOR_GRAY240          0XF79E

void st7789_init(void);
void st7789_transfer_datas(uint8_t buffer[], uint32_t bufsize);
void st7789_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void st7789_draw_point(uint16_t x, uint16_t y, uint16_t color);
void st7789_fill_color(uint16_t color);
void st7789_draw_point(uint16_t x, uint16_t y, uint16_t color);
void st7789_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void st7789_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

#ifdef  _cplusplus
}
#endif

#endif
