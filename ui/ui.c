// Placeholder so the simulator has something to draw. Replace with your own screens.
#include <lvgl.h>
#include "ui_api.h"

void ui_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    lv_obj_t *label = lv_label_create(scr);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, "Hello from ui/");
    lv_obj_center(label);
}
