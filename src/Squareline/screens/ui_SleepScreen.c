// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project_QVGA

#include "../ui.h"

void ui_SleepScreen_screen_init(void)
{
    ui_SleepScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_SleepScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_SleepTextArea = lv_textarea_create(ui_SleepScreen);
    lv_obj_set_width(ui_SleepTextArea, 222);
    lv_obj_set_height(ui_SleepTextArea, 70);
    lv_obj_set_x(ui_SleepTextArea, 4);
    lv_obj_set_y(ui_SleepTextArea, 1);
    lv_obj_set_align(ui_SleepTextArea, LV_ALIGN_CENTER);
    lv_textarea_set_text(ui_SleepTextArea, "Sleep");
    lv_textarea_set_placeholder_text(ui_SleepTextArea, "Placeholder...");



}
