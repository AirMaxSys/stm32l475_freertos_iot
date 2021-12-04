/**
 * @file gui.c
*/

#include "lvgl.h"
#include "tft_lvgl_layer.h"
#include "fs_lvgl_layer.h"

/* GUI color defination*/
#define GUI_COLOR_GREY              0x3B3B3Bu

/* GUI assets filesystem directory defination*/
/* @note "S:" string is LVGL filesystem checking demand*/
#define GUI_TEMP_IMG_PATH           "S:1:/assets/icon/temp.bin"
#define GUI_HUMI_IMG_PATH           "S:1:/assets/icon/humi.bin"
#define GUI_MONTSERRAT14_FONT_PATH  "S:1:/assets/font/mont14.bin"
#define GUI_MONTSERRAT48_FONT_PATH  "S:1:/assets/font/mont48.bin"

const lv_font_t *p_font_montserrat14;
const lv_font_t *p_font_montserrat48;

/**
 * @brief Initialize LVGL
*/
void gui_lvgl_init(void)
{
    // Initialize LVGL
    lv_init();

    // Initialize LVGL filesystem
    lv_fs_if_fatfs_init();

    // Load default font
    p_font_montserrat14 = lv_font_load(GUI_MONTSERRAT14_FONT_PATH);

    // Initialize lowlevel mmc driver
    tft_lvgl_layer_init();
}

/**
 * @brief Get image binary file form sd card and draw icon
 * @param p_temp_icon [out] lvgl temperature image object pointer
 * @param p_humi_icon [out] lvgl humidity image object pointer
*/
void gui_draw_temp_humi_icon_img(lv_obj_t **p_temp_icon, lv_obj_t **p_humi_icon)
{
    // Get image binary file
    lv_obj_t *temp_icon = lv_img_create(lv_scr_act());
    lv_img_set_src(temp_icon, GUI_TEMP_IMG_PATH);
    lv_obj_align(temp_icon, LV_ALIGN_TOP_LEFT, 0, 15);

    lv_obj_t *humi_icon = lv_img_create(lv_scr_act());
    lv_img_set_src(humi_icon, GUI_HUMI_IMG_PATH);
    lv_obj_align(humi_icon, LV_ALIGN_TOP_RIGHT, 0, 15);

    *p_temp_icon = temp_icon;
    *p_humi_icon = humi_icon;
}

/**
 * @brief Set up temperature and humidity label and text
 * @param p_lb_temp [out] temperature label pointer
 * @param p_lb_humi [out] humidity label pointer
*/
void gui_setup_temp_humi_label(lv_obj_t **p_lb_temp, lv_obj_t **p_lb_humi)
{
    // Load text font
    p_font_montserrat48 = lv_font_load(GUI_MONTSERRAT48_FONT_PATH);

    // Set up label text using style object
    static lv_style_t lb_style;
    lv_style_init(&lb_style);
    lv_style_set_text_font(&lb_style, p_font_montserrat48);
    lv_style_set_text_color(&lb_style, lv_color_hex(GUI_COLOR_GREY));
    lv_style_set_text_opa(&lb_style, LV_OPA_70);

    // Initialize temperature label
    lv_obj_t *lb_temp = lv_label_create(lv_scr_act());
    lv_obj_add_style(lb_temp, &lb_style, 0);
    lv_obj_align(lb_temp, LV_ALIGN_BOTTOM_LEFT, 10, -50);
    lv_label_set_text(lb_temp, "0.0");

    // Initialize humidity label
    lv_obj_t *lb_humi = lv_label_create(lv_scr_act());
    lv_obj_add_style(lb_humi, &lb_style, 0);
    lv_obj_align(lb_humi, LV_ALIGN_BOTTOM_RIGHT, -10, -50);
    lv_label_set_text(lb_humi, "0.0");

    // Output labels object pointer
    *p_lb_temp = lb_temp;
    *p_lb_humi = lb_humi;
}

/**
 * @brief Update temperature and humidity value on screen
 * @param lb_temp lvgl label object pointer of temperature
 * @param lb_humi lvgl label object pointer of humidity
 * @param msg temperature and humudity value buffer pointer
*/
void gui_update_temp_humi_text(lv_obj_t *lb_temp, lv_obj_t *lb_humi, const uint16_t *msg)
{
    if (!lb_temp || ! lb_humi || !msg) return;

    // Refresh temperature and humidity labels text value
    lv_label_set_text_fmt(lb_temp, "%.1f", msg[0] / 10.0 - 50);
    lv_label_set_text_fmt(lb_humi, "%.1f", msg[1] / 10.0);
}
