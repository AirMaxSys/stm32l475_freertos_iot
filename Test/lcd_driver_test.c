#include "st7789v2.h"
#include "proj_config.h"

void lcd_drv_test_task(void *argv)
{
    st7789_init();

    for (;;) {
        st7789_fill_color(COLOR_BLUE);
        portDelayMs(200);
        st7789_fill_color(COLOR_BRED);
        portDelayMs(200);
        st7789_fill_color(COLOR_CYAN);
        portDelayMs(200);
        st7789_fill_color(COLOR_YELLOW);
        portDelayMs(200);
        st7789_draw_line(50, 70, 200, 70);
        st7789_draw_line(10, 70, 10, 120);
        st7789_draw_line(100, 20, 120, 50);
        st7789_draw_rectangle(50, 80, 200, 160);
        st7789_draw_line(50, 80, 200, 160);
        st7789_draw_line(50, 160, 200, 80);
        for (int i = 0; i < 20; ++i)
            st7789_draw_point(200 + i, 200, COLOR_GBLUE);
        portDelayMs(200);
    }
}
