// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project_QVGA

#include "../ui.h"

void ui_ACEDATAScreen_screen_init(void)
{
    ui_ACEDATAScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ACEDATAScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_LabelOutputs1 = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelOutputs1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelOutputs1, LV_SIZE_CONTENT);    /// 16
    lv_obj_set_x(ui_LabelOutputs1, 0);
    lv_obj_set_y(ui_LabelOutputs1, -107);
    lv_obj_set_align(ui_LabelOutputs1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelOutputs1, "ACEDATA");
    lv_label_set_recolor(ui_LabelOutputs1, "true");
    lv_obj_set_style_text_color(ui_LabelOutputs1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelOutputs1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelOutputs1, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelOutputs1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelOutputs1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ImgButton9 = lv_imgbtn_create(ui_ACEDATAScreen);
    lv_imgbtn_set_src(ui_ImgButton9, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowlefticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton9, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton9, 240);
    lv_obj_set_width(ui_ImgButton9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton9, -138);
    lv_obj_set_y(ui_ImgButton9, 0);
    lv_obj_set_align(ui_ImgButton9, LV_ALIGN_CENTER);

    ui_LabelTemp1 = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelTemp1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelTemp1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelTemp1, 86);
    lv_obj_set_y(ui_LabelTemp1, 28);
    lv_label_set_text(ui_LabelTemp1, "Temp 1");
    lv_obj_set_style_text_color(ui_LabelTemp1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelTemp1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelTemp1, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelTemp1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelTemp1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelTemp1Val = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelTemp1Val, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelTemp1Val, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelTemp1Val, 146);
    lv_obj_set_y(ui_LabelTemp1Val, 28);
    lv_label_set_text(ui_LabelTemp1Val, "000.0F");
    lv_obj_set_style_text_color(ui_LabelTemp1Val, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelTemp1Val, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelTemp1Val, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelTemp1Val, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelTemp1Val, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelTemp2 = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelTemp2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelTemp2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelTemp2, 83);
    lv_obj_set_y(ui_LabelTemp2, 50);
    lv_label_set_text(ui_LabelTemp2, "Temp 2");
    lv_obj_set_style_text_color(ui_LabelTemp2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelTemp2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelTemp2, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelTemp2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelTemp2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelTemp2Val = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelTemp2Val, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelTemp2Val, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelTemp2Val, 146);
    lv_obj_set_y(ui_LabelTemp2Val, 49);
    lv_label_set_text(ui_LabelTemp2Val, "000.0F");
    lv_obj_set_style_text_color(ui_LabelTemp2Val, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelTemp2Val, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelTemp2Val, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelTemp2Val, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelTemp2Val, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelBattery = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelBattery, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelBattery, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelBattery, 108);
    lv_obj_set_y(ui_LabelBattery, 70);
    lv_label_set_text(ui_LabelBattery, "Batt");
    lv_obj_set_style_text_color(ui_LabelBattery, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelBattery, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelBattery, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelBattery, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelBattery, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelBatteryVal = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelBatteryVal, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelBatteryVal, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelBatteryVal, 146);
    lv_obj_set_y(ui_LabelBatteryVal, 70);
    lv_label_set_text(ui_LabelBatteryVal, "00.0V");
    lv_obj_set_style_text_color(ui_LabelBatteryVal, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelBatteryVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelBatteryVal, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelBatteryVal, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelBatteryVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelEngineStatus = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelEngineStatus, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelEngineStatus, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelEngineStatus, 48);
    lv_obj_set_y(ui_LabelEngineStatus, 91);
    lv_label_set_text(ui_LabelEngineStatus, "Stall Sensor");
    lv_obj_set_style_text_color(ui_LabelEngineStatus, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelEngineStatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelEngineStatus, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelEngineStatus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelEngineStatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelEngineStatusVal = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelEngineStatusVal, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelEngineStatusVal, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelEngineStatusVal, 146);
    lv_obj_set_y(ui_LabelEngineStatusVal, 91);
    lv_label_set_text(ui_LabelEngineStatusVal, "Running");
    lv_obj_set_style_text_color(ui_LabelEngineStatusVal, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelEngineStatusVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelEngineStatusVal, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelEngineStatusVal, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelEngineStatusVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelEngineStallSensor = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelEngineStallSensor, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelEngineStallSensor, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelEngineStallSensor, 47);
    lv_obj_set_y(ui_LabelEngineStallSensor, 112);
    lv_label_set_text(ui_LabelEngineStallSensor, "Stall Sensor");
    lv_obj_set_style_text_color(ui_LabelEngineStallSensor, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelEngineStallSensor, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelEngineStallSensor, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelEngineStallSensor, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelEngineStallSensor, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelEngineStallSensorVal = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelEngineStallSensorVal, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelEngineStallSensorVal, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelEngineStallSensorVal, 145);
    lv_obj_set_y(ui_LabelEngineStallSensorVal, 112);
    lv_label_set_text(ui_LabelEngineStallSensorVal, "Present");
    lv_obj_set_style_text_color(ui_LabelEngineStallSensorVal, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelEngineStallSensorVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelEngineStallSensorVal, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelEngineStallSensorVal, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelEngineStallSensorVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelEngineStallCount = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelEngineStallCount, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelEngineStallCount, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelEngineStallCount, 53);
    lv_obj_set_y(ui_LabelEngineStallCount, 133);
    lv_label_set_text(ui_LabelEngineStallCount, "Stall Count");
    lv_obj_set_style_text_color(ui_LabelEngineStallCount, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelEngineStallCount, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelEngineStallCount, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelEngineStallCount, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelEngineStallCount, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelEngineStallCountVal = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelEngineStallCountVal, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelEngineStallCountVal, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelEngineStallCountVal, 145);
    lv_obj_set_y(ui_LabelEngineStallCountVal, 133);
    lv_label_set_text(ui_LabelEngineStallCountVal, "99");
    lv_obj_set_style_text_color(ui_LabelEngineStallCountVal, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelEngineStallCountVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelEngineStallCountVal, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelEngineStallCountVal, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelEngineStallCountVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelAux1 = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelAux1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelAux1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelAux1, 100);
    lv_obj_set_y(ui_LabelAux1, 154);
    lv_label_set_text(ui_LabelAux1, "Aux 1");
    lv_obj_set_style_text_color(ui_LabelAux1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelAux1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelAux1, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelAux1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelAux1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelAux1Val = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelAux1Val, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelAux1Val, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelAux1Val, 145);
    lv_obj_set_y(ui_LabelAux1Val, 154);
    lv_label_set_text(ui_LabelAux1Val, "Off");
    lv_obj_set_style_text_color(ui_LabelAux1Val, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelAux1Val, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelAux1Val, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelAux1Val, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelAux1Val, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelAux2 = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelAux2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelAux2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelAux2, 97);
    lv_obj_set_y(ui_LabelAux2, 175);
    lv_label_set_text(ui_LabelAux2, "Aux 2");
    lv_obj_set_style_text_color(ui_LabelAux2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelAux2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelAux2, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelAux2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelAux2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelAux2Val = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelAux2Val, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelAux2Val, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelAux2Val, 145);
    lv_obj_set_y(ui_LabelAux2Val, 175);
    lv_label_set_text(ui_LabelAux2Val, "Off");
    lv_obj_set_style_text_color(ui_LabelAux2Val, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelAux2Val, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelAux2Val, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelAux2Val, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelAux2Val, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelK9Doors = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelK9Doors, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelK9Doors, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelK9Doors, 69);
    lv_obj_set_y(ui_LabelK9Doors, 196);
    lv_label_set_text(ui_LabelK9Doors, "K9 Doors");
    lv_obj_set_style_text_color(ui_LabelK9Doors, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelK9Doors, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelK9Doors, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelK9Doors, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelK9Doors, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelK9DoorsVal = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelK9DoorsVal, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelK9DoorsVal, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelK9DoorsVal, 145);
    lv_obj_set_y(ui_LabelK9DoorsVal, 196);
    lv_label_set_text(ui_LabelK9DoorsVal, "Closed");
    lv_obj_set_style_text_color(ui_LabelK9DoorsVal, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelK9DoorsVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelK9DoorsVal, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelK9DoorsVal, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelK9DoorsVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelVIMModel = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelVIMModel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelVIMModel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelVIMModel, 57);
    lv_obj_set_y(ui_LabelVIMModel, 218);
    lv_label_set_text(ui_LabelVIMModel, "VIM Model");
    lv_obj_set_style_text_color(ui_LabelVIMModel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelVIMModel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelVIMModel, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelVIMModel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelVIMModel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelVIMModelVal = lv_label_create(ui_ACEDATAScreen);
    lv_obj_set_width(ui_LabelVIMModelVal, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelVIMModelVal, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelVIMModelVal, 145);
    lv_obj_set_y(ui_LabelVIMModelVal, 219);
    lv_label_set_text(ui_LabelVIMModelVal, "XXXXXXXXXXXXXXXX");
    lv_obj_set_style_text_color(ui_LabelVIMModelVal, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelVIMModelVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelVIMModelVal, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_LabelVIMModelVal, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_LabelVIMModelVal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ImgButton10 = lv_imgbtn_create(ui_ACEDATAScreen);
    lv_imgbtn_set_src(ui_ImgButton10, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_arrowrighticon_png, NULL);
    lv_imgbtn_set_src(ui_ImgButton10, LV_IMGBTN_STATE_DISABLED, NULL, &ui_img_arrowiconblank_png, NULL);
    lv_obj_set_height(ui_ImgButton10, 240);
    lv_obj_set_width(ui_ImgButton10, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_ImgButton10, 137);
    lv_obj_set_y(ui_ImgButton10, 0);
    lv_obj_set_align(ui_ImgButton10, LV_ALIGN_CENTER);

    lv_obj_add_event_cb(ui_ImgButton9, ui_event_ImgButton9, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButton10, ui_event_ImgButton10, LV_EVENT_ALL, NULL);

}
