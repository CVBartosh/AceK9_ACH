// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project_QVGA

#include "../ui.h"

void ui_MenuScreen5_screen_init(void)
{
    ui_MenuScreen5 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_MenuScreen5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_ImgButtonExitMenu5 = lv_imgbtn_create(ui_MenuScreen5);
    lv_imgbtn_set_src(ui_ImgButtonExitMenu5, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowlefticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButtonExitMenu5, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButtonExitMenu5, 240);
    lv_obj_set_width(ui_ImgButtonExitMenu5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButtonExitMenu5, -138);
    lv_obj_set_y(ui_ImgButtonExitMenu5, 0);
    lv_obj_set_align(ui_ImgButtonExitMenu5, LV_ALIGN_CENTER);

    ui_ImgButton25 = lv_imgbtn_create(ui_MenuScreen5);
    lv_imgbtn_set_src(ui_ImgButton25, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowrighticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton25, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton25, 240);
    lv_obj_set_width(ui_ImgButton25, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton25, 137);
    lv_obj_set_y(ui_ImgButton25, 0);
    lv_obj_set_align(ui_ImgButton25, LV_ALIGN_CENTER);

    ui_ImgButton26 = lv_imgbtn_create(ui_MenuScreen5);
    lv_imgbtn_set_src(ui_ImgButton26, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowdownicon_png, NULL);
    lv_obj_set_height(ui_ImgButton26, 44);
    lv_obj_set_width(ui_ImgButton26, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton26, 0);
    lv_obj_set_y(ui_ImgButton26, 98);
    lv_obj_set_align(ui_ImgButton26, LV_ALIGN_CENTER);

    ui_LabelLog1 = lv_label_create(ui_MenuScreen5);
    lv_obj_set_width(ui_LabelLog1, 45);
    lv_obj_set_height(ui_LabelLog1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelLog1, -90);
    lv_obj_set_y(ui_LabelLog1, -65);
    lv_obj_set_align(ui_LabelLog1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelLog1, "Log1:");
    lv_obj_set_style_text_color(ui_LabelLog1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelLog1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelLog1, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelLog1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelLog1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ImgButton27 = lv_imgbtn_create(ui_MenuScreen5);
    lv_imgbtn_set_src(ui_ImgButton27, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowupicon_png, NULL);
    lv_obj_set_height(ui_ImgButton27, 44);
    lv_obj_set_width(ui_ImgButton27, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton27, 0);
    lv_obj_set_y(ui_ImgButton27, -98);
    lv_obj_set_align(ui_ImgButton27, LV_ALIGN_CENTER);

    ui_LabelLog2 = lv_label_create(ui_MenuScreen5);
    lv_obj_set_width(ui_LabelLog2, 45);
    lv_obj_set_height(ui_LabelLog2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelLog2, -90);
    lv_obj_set_y(ui_LabelLog2, -46);
    lv_obj_set_align(ui_LabelLog2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelLog2, "Log2:");
    lv_obj_set_style_text_color(ui_LabelLog2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelLog2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelLog2, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelLog2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelLog2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelLog3 = lv_label_create(ui_MenuScreen5);
    lv_obj_set_width(ui_LabelLog3, 45);
    lv_obj_set_height(ui_LabelLog3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelLog3, -90);
    lv_obj_set_y(ui_LabelLog3, -27);
    lv_obj_set_align(ui_LabelLog3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelLog3, "Log3:");
    lv_obj_set_style_text_color(ui_LabelLog3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelLog3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelLog3, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelLog3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelLog3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelLog4 = lv_label_create(ui_MenuScreen5);
    lv_obj_set_width(ui_LabelLog4, 45);
    lv_obj_set_height(ui_LabelLog4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelLog4, -90);
    lv_obj_set_y(ui_LabelLog4, -8);
    lv_obj_set_align(ui_LabelLog4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelLog4, "Log4:");
    lv_obj_set_style_text_color(ui_LabelLog4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelLog4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelLog4, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelLog4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelLog4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelLog5 = lv_label_create(ui_MenuScreen5);
    lv_obj_set_width(ui_LabelLog5, 45);
    lv_obj_set_height(ui_LabelLog5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelLog5, -90);
    lv_obj_set_y(ui_LabelLog5, 11);
    lv_obj_set_align(ui_LabelLog5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelLog5, "Log5:");
    lv_obj_set_style_text_color(ui_LabelLog5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelLog5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelLog5, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelLog5, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelLog5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelLog6 = lv_label_create(ui_MenuScreen5);
    lv_obj_set_width(ui_LabelLog6, 45);
    lv_obj_set_height(ui_LabelLog6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelLog6, -90);
    lv_obj_set_y(ui_LabelLog6, 30);
    lv_obj_set_align(ui_LabelLog6, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelLog6, "Log6:");
    lv_obj_set_style_text_color(ui_LabelLog6, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelLog6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelLog6, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelLog6, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelLog6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelLog7 = lv_label_create(ui_MenuScreen5);
    lv_obj_set_width(ui_LabelLog7, 45);
    lv_obj_set_height(ui_LabelLog7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelLog7, -90);
    lv_obj_set_y(ui_LabelLog7, 49);
    lv_obj_set_align(ui_LabelLog7, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelLog7, "Log7:");
    lv_obj_set_style_text_color(ui_LabelLog7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelLog7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelLog7, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelLog7, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelLog7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_ImgButtonExitMenu5, ui_event_ImgButtonExitMenu5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButton25, ui_event_ImgButton25, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButton26, ui_event_ImgButton26, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButton27, ui_event_ImgButton27, LV_EVENT_ALL, NULL);

}
