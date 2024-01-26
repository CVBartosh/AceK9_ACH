#pragma once
#ifdef TOUCH_WIDTH
#include <touch_config.h>
#include <ft6206.hpp>


extern arduino::ft6206<TOUCH_WIDTH,TOUCH_HEIGHT> touch;
#endif
extern "C" void input_init();