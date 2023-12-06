#include <display.hpp>
#define LVGL_LCD_BUF_SIZE (32*1024)

#include <lvgl.h>
#define LCD_IMPLEMENTATION
#include <lcd_init.h>
arduino::lcd_miser<2,true,1> dimmer;
static lv_disp_draw_buf_t disp_buf;  // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;       // contains callback functions
static lv_color_t *lv_disp_buf;
static lv_color_t *lv_disp_buf2;

static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    lv_disp_flush_ready(&disp_drv);
    return true;
}
static void lvgl_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    lcd_panel_draw_bitmap(offsetx1, offsety1, offsetx2 , offsety2 , (void*)color_map);
}
extern "C" void display_update() {
    dimmer.wake(); // uncomment to enable sleeping
    dimmer.update();
}
extern "C" void display_init() {
    lv_disp_buf = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_disp_buf2 = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf, lv_disp_buf2, LVGL_LCD_BUF_SIZE);
    lv_disp_drv_init(&disp_drv);
    
#ifdef LCD_SWAP_XY
    disp_drv.hor_res = LCD_VRES;
    disp_drv.ver_res = LCD_HRES;
#else
    disp_drv.hor_res = LCD_HRES;
    disp_drv.ver_res = LCD_VRES;
#endif
    disp_drv.flush_cb = lvgl_flush;
    disp_drv.draw_buf = &disp_buf;
// init process is slightly different for RGB displays (not used here, but future proofed)
#ifndef LCD_PIN_NUM_VSYNC
    lcd_panel_init(LVGL_LCD_BUF_SIZE*2, lcd_flush_ready);
#else 
    lcd_panel_init();
#endif
    lv_disp_drv_register(&disp_drv);
    dimmer.initialize();
    
}

