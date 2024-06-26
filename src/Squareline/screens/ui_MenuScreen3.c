// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project_QVGA

#include "../ui.h"

void ui_MenuScreen3_screen_init(void)
{
    ui_MenuScreen3 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_MenuScreen3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_ImgButtonExitMenu3 = lv_imgbtn_create(ui_MenuScreen3);
    lv_imgbtn_set_src(ui_ImgButtonExitMenu3, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowlefticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButtonExitMenu3, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButtonExitMenu3, 240);
    lv_obj_set_width(ui_ImgButtonExitMenu3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButtonExitMenu3, -138);
    lv_obj_set_y(ui_ImgButtonExitMenu3, 0);
    lv_obj_set_align(ui_ImgButtonExitMenu3, LV_ALIGN_CENTER);

    ui_ImgButton21 = lv_imgbtn_create(ui_MenuScreen3);
    lv_imgbtn_set_src(ui_ImgButton21, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowrighticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton21, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton21, 240);
    lv_obj_set_width(ui_ImgButton21, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton21, 137);
    lv_obj_set_y(ui_ImgButton21, 0);
    lv_obj_set_align(ui_ImgButton21, LV_ALIGN_CENTER);

    ui_ImgButton22 = lv_imgbtn_create(ui_MenuScreen3);
    lv_imgbtn_set_src(ui_ImgButton22, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowdownicon_png, NULL);
    lv_obj_set_height(ui_ImgButton22, 44);
    lv_obj_set_width(ui_ImgButton22, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton22, 0);
    lv_obj_set_y(ui_ImgButton22, 98);
    lv_obj_set_align(ui_ImgButton22, LV_ALIGN_CENTER);

    ui_LabelIntelaBoxSN = lv_label_create(ui_MenuScreen3);
    lv_obj_set_width(ui_LabelIntelaBoxSN, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelIntelaBoxSN, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelIntelaBoxSN, 0);
    lv_obj_set_y(ui_LabelIntelaBoxSN, -58);
    lv_obj_set_align(ui_LabelIntelaBoxSN, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelIntelaBoxSN, "IntelaBox SN");
    lv_obj_set_style_text_color(ui_LabelIntelaBoxSN, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelIntelaBoxSN, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelIntelaBoxSN, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelIntelaBoxSN, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelIntelaBoxSN, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelIntelaBoxSNValue = lv_label_create(ui_MenuScreen3);
    lv_obj_set_width(ui_LabelIntelaBoxSNValue, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelIntelaBoxSNValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelIntelaBoxSNValue, 0);
    lv_obj_set_y(ui_LabelIntelaBoxSNValue, -37);
    lv_obj_set_align(ui_LabelIntelaBoxSNValue, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelIntelaBoxSNValue, "I500X0005G#TEST1");
    lv_obj_set_style_text_color(ui_LabelIntelaBoxSNValue, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelIntelaBoxSNValue, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelIntelaBoxSNValue, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelIntelaBoxSNValue, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelIntelaBoxSNValue, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelControlHeadSN = lv_label_create(ui_MenuScreen3);
    lv_obj_set_width(ui_LabelControlHeadSN, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelControlHeadSN, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelControlHeadSN, 0);
    lv_obj_set_y(ui_LabelControlHeadSN, -13);
    lv_obj_set_align(ui_LabelControlHeadSN, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelControlHeadSN, "Control Head SN");
    lv_obj_set_style_text_color(ui_LabelControlHeadSN, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelControlHeadSN, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelControlHeadSN, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelControlHeadSN, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelControlHeadSN, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelControlHeadSNValue = lv_label_create(ui_MenuScreen3);
    lv_obj_set_width(ui_LabelControlHeadSNValue, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelControlHeadSNValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelControlHeadSNValue, 0);
    lv_obj_set_y(ui_LabelControlHeadSNValue, 9);
    lv_obj_set_align(ui_LabelControlHeadSNValue, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelControlHeadSNValue, "CHU5B10035#CCH00");
    lv_obj_set_style_text_color(ui_LabelControlHeadSNValue, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelControlHeadSNValue, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelControlHeadSNValue, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelControlHeadSNValue, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelControlHeadSNValue, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ImgButton23 = lv_imgbtn_create(ui_MenuScreen3);
    lv_imgbtn_set_src(ui_ImgButton23, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowupicon_png, NULL);
    lv_obj_set_height(ui_ImgButton23, 44);
    lv_obj_set_width(ui_ImgButton23, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton23, 0);
    lv_obj_set_y(ui_ImgButton23, -98);
    lv_obj_set_align(ui_ImgButton23, LV_ALIGN_CENTER);

    ui_BTNRestoreDefaults = lv_btn_create(ui_MenuScreen3);
    lv_obj_set_width(ui_BTNRestoreDefaults, 193);
    lv_obj_set_height(ui_BTNRestoreDefaults, 26);
    lv_obj_set_x(ui_BTNRestoreDefaults, 0);
    lv_obj_set_y(ui_BTNRestoreDefaults, 46);
    lv_obj_set_align(ui_BTNRestoreDefaults, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BTNRestoreDefaults, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BTNRestoreDefaults, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_LabelControlHeadSNValue1 = lv_label_create(ui_MenuScreen3);
    lv_obj_set_width(ui_LabelControlHeadSNValue1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelControlHeadSNValue1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelControlHeadSNValue1, 0);
    lv_obj_set_y(ui_LabelControlHeadSNValue1, 46);
    lv_obj_set_align(ui_LabelControlHeadSNValue1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelControlHeadSNValue1, "Restore Default Settings");
    lv_obj_set_style_text_color(ui_LabelControlHeadSNValue1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelControlHeadSNValue1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelControlHeadSNValue1, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_ImgButtonExitMenu3, ui_event_ImgButtonExitMenu3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButton21, ui_event_ImgButton21, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButton22, ui_event_ImgButton22, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButton23, ui_event_ImgButton23, LV_EVENT_ALL, NULL);

}
