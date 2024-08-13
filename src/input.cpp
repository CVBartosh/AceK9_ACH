#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <input.hpp>

#ifdef TOUCH_WIDTH
arduino::ft6206<TOUCH_WIDTH,TOUCH_HEIGHT> touch(Wire);

void lvgl_touch_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t x;
    uint16_t y;
    
    if(touch.update() && touch.xy(&x,&y)) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

extern "C" void input_init() {
    touch.initialize();
    touch.rotation(TOUCH_ROTATION);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_read;
    lv_indev_drv_register(&indev_drv);

}
#else
extern "C" void input_init() {
}
#endif
