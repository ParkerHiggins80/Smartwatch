// Touch test screen. Replace with your own screens later.
#include <lvgl.h>
#include "ui_api.h"

static lv_obj_t *coord_label; // shows where the finger is
static lv_obj_t *count_label; // text inside the button
static lv_obj_t *dot;         // follows the finger while pressed
static int taps;

// Runs for every touch event on the screen background
static void screen_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL) {
        return;
    }

    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING) {
        lv_label_set_text_fmt(coord_label, "x %d  y %d", p.x, p.y);
        lv_obj_set_pos(dot, p.x - 10, p.y - 10);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_HIDDEN);
    } else if (code == LV_EVENT_LONG_PRESSED) {
        lv_label_set_text(coord_label, "long press");
    } else if (code == LV_EVENT_RELEASED) {
        lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
    }
}

// Runs when the button is tapped (pressed and released on it)
static void btn_event_cb(lv_event_t *e)
{
    taps++;
    lv_label_set_text_fmt(count_label, "Taps: %d", taps);
}

void ui_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    // Without this, dragging near the edge would scroll the screen instead
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr, screen_event_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_label_set_text(title, "Hello");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 70);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 200, 80);
    lv_obj_center(btn);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    count_label = lv_label_create(btn);
    lv_obj_set_style_text_font(count_label, &lv_font_montserrat_20, 0);
    lv_label_set_text(count_label, "Taps: 0");
    lv_obj_center(count_label);

    coord_label = lv_label_create(scr);
    lv_obj_set_style_text_font(coord_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(coord_label, lv_color_white(), 0);
    lv_label_set_text(coord_label, "Touch anywhere");
    lv_obj_align(coord_label, LV_ALIGN_BOTTOM_MID, 0, -80);

    dot = lv_obj_create(scr);
    lv_obj_set_size(dot, 20, 20);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    // The dot must not catch touches itself, or it would block the screen under it
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
}