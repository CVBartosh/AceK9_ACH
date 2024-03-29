// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project_QVGA

#include "../ui.h"

void ui_SerialCOMsScreen_screen_init(void)
{
    ui_SerialCOMsScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_SerialCOMsScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_LabelTXData = lv_label_create(ui_SerialCOMsScreen);
    lv_obj_set_width(ui_LabelTXData, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelTXData, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelTXData, -69);
    lv_obj_set_y(ui_LabelTXData, -106);
    lv_obj_set_align(ui_LabelTXData, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelTXData, "TX Data");

    ui_TextAreaTXData = lv_textarea_create(ui_SerialCOMsScreen);
    lv_obj_set_width(ui_TextAreaTXData, 247);
    lv_obj_set_height(ui_TextAreaTXData, 57);
    lv_obj_set_x(ui_TextAreaTXData, 26);
    lv_obj_set_y(ui_TextAreaTXData, -65);
    lv_obj_set_align(ui_TextAreaTXData, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_TextAreaTXData, "Placeholder...");



    ui_LabelRXData = lv_label_create(ui_SerialCOMsScreen);
    lv_obj_set_width(ui_LabelRXData, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelRXData, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelRXData, -66);
    lv_obj_set_y(ui_LabelRXData, -20);
    lv_obj_set_align(ui_LabelRXData, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelRXData, "RX Data");

    ui_TextAreaRXData = lv_textarea_create(ui_SerialCOMsScreen);
    lv_obj_set_width(ui_TextAreaRXData, 241);
    lv_obj_set_height(ui_TextAreaRXData, 111);
    lv_obj_set_x(ui_TextAreaRXData, 27);
    lv_obj_set_y(ui_TextAreaRXData, 53);
    lv_obj_set_align(ui_TextAreaRXData, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_TextAreaRXData, "Placeholder...");



    ui_ImgButton11 = lv_imgbtn_create(ui_SerialCOMsScreen);
    lv_imgbtn_set_src(ui_ImgButton11, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowlefticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton11, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton11, 240);
    lv_obj_set_width(ui_ImgButton11, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton11, -138);
    lv_obj_set_y(ui_ImgButton11, 0);
    lv_obj_set_align(ui_ImgButton11, LV_ALIGN_CENTER);

    lv_obj_add_event_cb(ui_ImgButton11, ui_event_ImgButton11, LV_EVENT_ALL, NULL);

}
