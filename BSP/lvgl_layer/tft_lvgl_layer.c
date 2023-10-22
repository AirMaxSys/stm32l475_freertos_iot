#include "lvgl.h"
#include "lv_conf.h"

#include "stm32l4xx.h"

#include "tft_lvgl_layer.h"
#include "st7789v2.h"

// TFT display resoultion
#define TFT_LVGL_LAYER_DISP_HOR_RES  ST7789_W
#define TFT_LVGL_LAYER_DISP_VER_RES  ST7789_H

// Flush data to TFT call back funtion for LVGL display driver
static void tft_flush_cb(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

void tft_lvgl_layer_init(void)
{
    // LVGL drawing buffer handler
    static lv_disp_draw_buf_t tft_disp_buf;
    // Using two buffer which one for DMA transfer data and another for MCU fill data
    static lv_color_t tft_disp_buf1[TFT_LVGL_LAYER_DISP_HOR_RES*20];

    // LVGL layer display driver handler
    static lv_disp_drv_t tft_disp_drv;

    // Init TFT
    st7789_init();

    // Init buffer for LVGL drawing
    lv_disp_draw_buf_init(&tft_disp_buf, tft_disp_buf1, NULL, TFT_LVGL_LAYER_DISP_HOR_RES*20);

    // Init display driver basic items
    lv_disp_drv_init(&tft_disp_drv);
    // Init display drivers mandatory fields
    tft_disp_drv.draw_buf = &tft_disp_buf;
    tft_disp_drv.hor_res = TFT_LVGL_LAYER_DISP_HOR_RES;
    tft_disp_drv.ver_res = TFT_LVGL_LAYER_DISP_VER_RES;
    tft_disp_drv.flush_cb = tft_flush_cb;
    // Register the driver to LVGL
    lv_disp_drv_register(&tft_disp_drv);
}

static void tft_flush_cb(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    // Checking parameters
    if (!disp_drv || !area || !color_p)
        return;
    if (area->x1 < 0 || area->x1 > TFT_LVGL_LAYER_DISP_HOR_RES - 1)
        return;
    if (area->x2 < 0 || area->x2 > TFT_LVGL_LAYER_DISP_HOR_RES - 1)
        return;
    if (area->y1 < 0 || area->y1 > TFT_LVGL_LAYER_DISP_HOR_RES - 1)
        return;
    if (area->y2 < 0 || area->y2 > TFT_LVGL_LAYER_DISP_HOR_RES - 1)
        return;
    if (area->x2 < area->x1 || area->y1 < area->y2)
        return;

    // Set address of drawing window
    st7789_set_window(area->x1, area->y1, area->x2, area->y2);

    // Transfer data with DMA
    st7789_transfer_datas((uint8_t *)color_p,   \
        sizeof(lv_color_t) * (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));

    // Inform LVGL that flushing ready
    lv_disp_flush_ready(disp_drv);
}
