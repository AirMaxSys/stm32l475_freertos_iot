#include "gui.h"

void gui_draw_temp_humi_icon_img(lv_obj_t **p_temp_icon, lv_obj_t **p_humi_icon)
{
    LV_IMG_DECLARE(img_temp_rgb)
    lv_obj_t *temp_icon = lv_img_create(lv_scr_act());
    lv_img_set_src(temp_icon, &img_temp_rgb);
    lv_obj_align(temp_icon, LV_ALIGN_TOP_LEFT, 0, 15);

    LV_IMG_DECLARE(img_humi_rgb)
    lv_obj_t *humi_icon = lv_img_create(lv_scr_act());
    lv_img_set_src(humi_icon, &img_humi_rgb);
    lv_obj_align(humi_icon, LV_ALIGN_TOP_RIGHT, 0, 15);

    *p_temp_icon = temp_icon;
    *p_humi_icon = humi_icon;
}

void gui_set_temp_humi_val_lb(lv_obj_t **p_lb_temp, lv_obj_t *img_temp,
    lv_obj_t **p_lb_humi, lv_obj_t *img_humi)
{
    if (!img_temp || !img_humi)
        return;

    static lv_style_t lb_style;
    lv_style_init(&lb_style);
    lv_style_set_text_font(&lb_style, &lv_font_montserrat_48);
    lv_style_set_text_color(&lb_style, lv_color_hex(TH_LB_TEXT_COLOR_GREY));
    lv_style_set_text_opa(&lb_style, LV_OPA_70);

    lv_obj_t *lb_temp = lv_label_create(lv_scr_act());
    lv_obj_add_style(lb_temp, &lb_style, 0);
    lv_obj_align_to(lb_temp, img_temp, LV_ALIGN_BOTTOM_MID, 0, 55);
    lv_label_set_text(lb_temp, "0.0");

    lv_obj_t *lb_humi = lv_label_create(lv_scr_act());
    lv_obj_add_style(lb_humi, &lb_style, 0);
    lv_obj_align_to(lb_humi, img_humi, LV_ALIGN_BOTTOM_MID, 0, 55);
    lv_label_set_text(lb_humi, "0.0");

    *p_lb_temp = lb_temp;
    *p_lb_humi = lb_humi;
}

void gui_temp_humi_val_update(lv_obj_t *lb_temp, lv_obj_t *lb_humi, const uint16_t *msg)
{
    if (!lb_temp || ! lb_humi || !msg)
        return;

    // Refresh LVGL text label using event
    lv_label_set_text_fmt(lb_temp, "%.1f", msg[0] / 10.0 - 50);
    lv_label_set_text_fmt(lb_humi, "%.1f", msg[1] / 10.0);
}
