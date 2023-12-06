#pragma once
#include <lcd_miser.hpp>

extern arduino::lcd_miser<2,true,1> dimmer;
extern "C" void display_init();
extern "C" void display_update();