#ifndef TOUCH_CONFIG_H
#define TOUCH_CONFIG_H

#ifdef ESP_PLATFORM
#ifdef ILI9341_SPI
#define TOUCH_ROTATION 3 // 1
#define TOUCH_WIDTH 240
#define TOUCH_HEIGHT 320
#endif // ILI9341_SPI

#ifdef ILI9341_PARALLEL8
#define TOUCH_ROTATION 1
#define TOUCH_WIDTH 240
#define TOUCH_HEIGHT 320
#endif // ILI9341_PARALLEL8

#endif // ESP_PLATFORM
#ifndef TOUCH_WIDTH
#ifdef LCD_SWAP_XY
#if LCD_SWAP_XY
#define TOUCH_WIDTH 320
#define TOUCH_HEIGHT 480
#else
#define TOUCH_WIDTH 480
#define TOUCH_HEIGHT 320
#endif
#else
//#define TOUCH_ROTATION 0 // 1
//#define TOUCH_WIDTH 480
//#define TOUCH_HEIGHT 320
#endif
#endif
#endif // LCD_CONFIG_H