#ifndef _GUI_H_
#define _GUI_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include <stdint.h>

#define TH_LB_TEXT_COLOR_GREY   0x3B3B3Bu

void gui_draw_temp_humi_icon_img(lv_obj_t **p_temp_icon, lv_obj_t **p_humi_icon);
void gui_set_temp_humi_val_lb(lv_obj_t **p_lb_temp, lv_obj_t *img_temp,
    lv_obj_t **p_lb_humi, lv_obj_t *img_humi);
void gui_temp_humi_val_update(lv_obj_t *lb_temp, lv_obj_t *lb_humi, const uint16_t *msg);

#ifdef  __cplusplus
}
#endif

#endif
