/**
 * @file gui.h
*/

#ifndef _GUI_H_
#define _GUI_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include <stdint.h>

void gui_draw_temp_humi_icon_img(lv_obj_t **p_temp_icon, lv_obj_t **p_humi_icon);
void gui_setup_temp_humi_label(lv_obj_t **p_lb_temp, lv_obj_t **p_lb_humi);
void gui_update_temp_humi_text(lv_obj_t *lb_temp, lv_obj_t *lb_humi, const uint16_t *msg);

#ifdef  __cplusplus
}
#endif

#endif
