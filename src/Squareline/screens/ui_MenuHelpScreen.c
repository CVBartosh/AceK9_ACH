// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project_QVGA

#include "../ui.h"

void ui_MenuHelpScreen_screen_init(void)
{
    ui_MenuHelpScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_MenuHelpScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label1 = lv_label_create(ui_MenuHelpScreen);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, 0);
    lv_obj_set_y(ui_Label1, 70);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "Please Test System Weekly");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0x00E100), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image1 = lv_img_create(ui_MenuHelpScreen);
    lv_img_set_src(ui_Image1, &ui_img_acek9logo_png);
    lv_obj_set_width(ui_Image1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image1, 0);
    lv_obj_set_y(ui_Image1, -74);
    lv_obj_set_align(ui_Image1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image1, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_btnGoToMenu = lv_btn_create(ui_MenuHelpScreen);
    lv_obj_set_width(ui_btnGoToMenu, 100);
    lv_obj_set_height(ui_btnGoToMenu, 50);
    lv_obj_set_x(ui_btnGoToMenu, 0);
    lv_obj_set_y(ui_btnGoToMenu, 12);
    lv_obj_set_align(ui_btnGoToMenu, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_btnGoToMenu, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_btnGoToMenu, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_btnGoToMenu, lv_color_hex(0xB4CD0E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_btnGoToMenu, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label3 = lv_label_create(ui_MenuHelpScreen);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, 0);
    lv_obj_set_y(ui_Label3, 13);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Menu");
    lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label3, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_btnGoToMenu, ui_event_btnGoToMenu, LV_EVENT_ALL, NULL);

}
