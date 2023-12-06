#pragma once
#include <touch_config.h>
#include <ft6206.hpp>


extern arduino::ft6206<TOUCH_WIDTH,TOUCH_HEIGHT> touch;

extern "C" void input_init();