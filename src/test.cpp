#define XBEE Serial1
#define MONITOR Serial
#define ACECONSerial Serial2
#include <Arduino.h>
#include <lvgl.h>
#include <Squareline/ui.h>
#include <interface.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "display.hpp"
#include "input.hpp"
#include "xbee/platform.h"
#include "xbee/atcmd.h"
#include "xbee/tx_status.h"
#include "xbee/user_data.h"
    #define ACEDATA_TX 48
void flood_it() {
    ACECONSerial.begin(4800,SERIAL_8N1,-1,ACEDATA_TX,true);
    while(true) {
        ACECONSerial.write("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
        ACECONSerial.flush(true);
    }
}