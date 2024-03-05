#define XBEE Serial1
#define MONITOR Serial
#define ACECONSerial Serial2
#include <Arduino.h>
#include <Wire.h>
#include <functional>
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
#include "test.h"
#include "vim_controller.hpp"
#include "SPIFFS.h"
#include "init_csv.h"

//================================================== Temperature Variables =========================================
#define TempErrorGeneral    'E'
#define TempErrorOpen       '5'
#define TempErrorSho        '4'
#define TempSignPos         ' '
#define TempSignNeg         '-'

enum TempState {tst_OK,tst_Warning,tst_Over, tst_OverPlus,tst_Under};
struct TempValues{
    float leftTemp;
    bool leftTempError;
    char leftTempSign;
    TempState leftTempState;
    float rightTemp;
    bool rightTempError;
    char rightTempSign;
    TempState rightTempState;
    float avgTemp;
    bool valueChanged;
    bool unitsChanged;
};

TempValues tempvalues_current;
TempValues tempvalues_previous;
static bool last_received = false;
//================================================== Battery Variables =========================================*/
enum BatteryThreshold {B_100=100,B_105=105,B_110=110,B_115=115,B_120=120};
BatteryThreshold BatterySetting = BatteryThreshold::B_100;

struct BatteryValues{
    float voltage;
    bool error;
    bool valueChanged;
};

BatteryValues battvalues_current;
BatteryValues battvalues_previous;

int BadBatteryCounter=0;
#define MaxBadBattValCounter 5

//================================================== Engine Stall Variables =========================================*/
struct EngineValues{
    bool engineStalled;
    int engineStallCount;
    bool engineStallSensorPresent;
    bool ignitionOn;
    bool valueChanged;
};

EngineValues enginevalues_current;
EngineValues enginevalues_previous;

//================================================== Aux Variables =========================================*/
struct AuxValues{
    bool aux1Active;
    bool aux2Active;
    bool valueChanged;
};

AuxValues auxvalues_current;
AuxValues auxvalues_previous;

//================================================== K9 Door Variables =========================================*/
struct DoorValues{
    bool doorOpen;
    bool doorPopped;
    bool valueChanged;
};

DoorValues doorvalues_current;
DoorValues doorvalues_previous;

//================================================== VIM Variables =========================================*/
String VIMSerialNumber;


//================================================== GLOBAL VARIABLES =========================================*/

struct last_packet_info {
    STATUS_CODE status; // = STATUS_CODE::SUCCESS;
    COMMAND_ID cmd; // = (COMMAND_ID)0;
};
static last_packet_info last_packet = {STATUS_CODE::SUCCESS,(COMMAND_ID)0};

struct last_at_info {
    char commandstr[2];
    uint32_t value;
    bool value_received;
};

static last_at_info last_at_cmd;

static config_packet config_data;

static command_packet command_data;

struct acecon {
    bool alm;
    bool hps;
    bool ppt;
    bool pps;
    bool ign;
    bool valueChanged;
};
acecon aceconvalues_current = {false,false,false,false,false,false};
acecon aceconvalues_previous = {false,false,false,false,false,false};

struct acedata{
    char leftTempSign;
    float leftTemp;
    char rightTempSign;
    float rightTemp;
    float batteryVoltage;
    bool engineStallSensorPresent;
    int EngineStallCount;
    bool engineStalled;
    bool Aux1Input;
    bool Aux2Input;
    bool K9DoorOpen;
    char VIMSerialNumber[17];
    bool ValueChanged;
};

acedata acedata_current;
acedata acedata_previous;

enum PowerOpt {p_CarONCarOFF,p_CarONManOFF,p_ManONManOFF,p_NoK9Left,p_OFF,p_AlwaysOFF};
enum Batt {b_10 = 1,b_105 = 2,b_11 = 3,b_115 =4,b_12 =5};

#define HotTempArraySize 19
const int HotTempOpt_F[HotTempArraySize] = { 77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95 };
const int HotTempOpt_C[HotTempArraySize] = { 25,26,26,27,27,28,28,29,29,30,31,31,32,32,33,33,34,34,35 };

#define ColdTempArraySize 14
const int ColdTempOpt_F[ColdTempArraySize] = { 0,10,14,18,21,25,28,32,38,42,46,50,54,58 };
const int ColdTempOpt_C[ColdTempArraySize] = { 0,-12,-10,-8,-6,-4,-2,0,2,4,6,8,10,12 };

struct SystemSettings{
    bool TempAveragingEnabled;
    bool AutoSnoozeEnabled;
    bool StallMonitorEnabled;
    bool AuxInputEnabled;
    bool doorDisabled;
    int AlarmHotSetIndex;
    int AlarmColdSetIndex;
    bool AlarmUnitF;
    PowerOpt AlarmPower;
    PowerOpt DoorPower;
    Batt BatteryVoltage;

};

SystemSettings systemsettings_current;
SystemSettings systemsettings_previous;
SystemSettings systemsettings_default;

#define EngineStallThreshold  1

//================================================== XBEE STUFF =========================================*/
    xbee_serial_t XBEE_SERPORT;
    int status;
    xbee_dev_t my_xbee;
    bool xbee_initialized = false;
    bool xbee_cell_connected = false;
    unsigned long signal_strength_timer;
    #define signal_strength_timer_threshold 1*60*1000
    uint32_t xbee_signal_strength_current = 255;
    bool xbee_signal_strength_changed = false;

    #define XBEE_SIGNALSTRENGTH0 0x69
    #define XBEE_SIGNALSTRENGTH1 0x61
    #define XBEE_SIGNALSTRENGTH2 0x53
    #define XBEE_SIGNALSTRENGTH3 0x45
    #define XBEE_SIGNALSTRENGTH4 0x37
    
    bool xbee_reset = false;
    #define XBEE_RXD    4
    #define XBEE_TXD    5
    #define XBEE_RSSI   6
    #define XBEE_RESET  7
    #define XBEE_CMD    15
    #define XBEE_LINK   16


//================================================== ACECON GPIOs =========================================
    #define ACECON_PPS_IN 9
    #define ACECON_IGN_IN 10
    #define ACECON_POP_IN 11
    #define ACECON_PPS_OUT 12
    #define ACECON_PPT_OUT 13
    #define ACECON_HPS_OUT 14
    #define ACECON_ALM_OUT 21
    #define ACEDATA_RX 47
    #define ACEDATA_TX 48

//================================================== ACEDATA Stuff =========================================
    #define ACEDATA_Temp1_Sign_POS 11
    #define ACEDATA_Temp1_1000X_POS 12 
    #define ACEDATA_Temp1_100X_POS 13 
    #define ACEDATA_Temp1_10X_POS 14 
    #define ACEDATA_Temp1_1X_POS 15 

    #define ACEDATA_Temp2_Sign_POS 16
    #define ACEDATA_Temp2_1000X_POS 17 
    #define ACEDATA_Temp2_100X_POS 18 
    #define ACEDATA_Temp2_10X_POS 19 
    #define ACEDATA_Temp2_1X_POS 20 

    #define ACEDATA_Batt_100X_POS 21
    #define ACEDATA_Batt_10X_POS 22
    #define ACEDATA_Batt_1X_POS 23

    #define ACEDATA_Stall_Sensor_Present_POS 91
    #define ACEDATA_Stall_Count_10X_POS 33
    #define ACEDATA_Stall_Count_1X_POS 34
    #define ACEDATA_Stall_Status_POS 32

    #define ACEDATA_Aux1_Input_POS  36
    #define ACEDATA_Aux2_Input_POS  35

    #define ACEDATA_K9Door_POS  37

    #define ACEDATA_VIM_SN_POS 46
    #define ACEDATA_VIM_SN_LENGTH 16


bool SettingsLoaded = false;

void save_settings() {

    if(SPIFFS.exists("/settings")) {
        SPIFFS.remove("/settings");
    }
    File file = SPIFFS.open("/settings","wb",true);
    if(sizeof(systemsettings_current)>file.write((uint8_t*)&systemsettings_current,sizeof(systemsettings_current))) {
        MONITOR.println("Error writing settings file. Too few bytes written");
    }
    file.close();

}

void print_settings(){
    MONITOR.println("Printing System Settings");
    MONITOR.println("AlarmPower: " + (String)(systemsettings_current.AlarmPower));
}

bool load_settings() {
    MONITOR.println("Loading Settings File");
    memcpy(&systemsettings_current,&systemsettings_default,sizeof(systemsettings_current));
    File file = SPIFFS.open("/settings","rb",false);
    if(!file) {
        MONITOR.println("Settings File Not Found");
        return false;
    }
    if(sizeof(systemsettings_current)>file.read((uint8_t*)&systemsettings_current,sizeof(systemsettings_current))) {
        memcpy(&systemsettings_current,&systemsettings_default,sizeof(systemsettings_current));
        file.close();
        MONITOR.println("Settings File Invalid (read length)");
        return false;
    }
    file.close();
    return true;
}

//================================================== XBee HAL Functions =========================================*/

// function that handles received User Data frames
int user_data_rx(xbee_dev_t *xbee, const void FAR *raw,uint16_t length, void FAR *context)
{
    XBEE_UNUSED_PARAMETER(xbee);
    XBEE_UNUSED_PARAMETER(context);
    
    const xbee_frame_user_data_rx_t FAR *data = (const xbee_frame_user_data_rx_t FAR *)raw;
    int payload_length = length - offsetof(xbee_frame_user_data_rx_t,
                                           payload);

    MONITOR.printf("Message from %s interface:\n",
        xbee_user_data_interface(data->source));

    if(payload_length<1) {
        MONITOR.println("No frame payload received");
        return 0;
    }
    const uint8_t* payload = data->payload;
    int cmd = payload[0];
    
    switch((COMMAND_ID)cmd) {
        case COMMAND_ID::ACKNOWLEDGE: {
            acknowledge_packet pck;
            memcpy(&pck,payload+5,payload_length-5);
            MONITOR.printf("Acknowledge Packet Received: %d\n",pck.status);
            last_packet.cmd = pck.cmd_ID;
            last_packet.status = pck.status;
            last_received = true;
        }
        break;
        case COMMAND_ID::COMMAND: {
            MONITOR.println("Command Packet Received");
            command_packet pck;
            memcpy(&pck,payload+5,payload_length-5);
            last_packet.cmd = pck.cmd_ID;
            command_data = pck;
            last_received = true;
        }
        break;
        case COMMAND_ID::CONFIG: {
            MONITOR.println("Config Packet Received");
            config_packet pck;
            memcpy(&pck,payload+5,payload_length-5);
            
            last_packet.cmd = pck.cmd_ID;
            config_data = pck;
            last_received = true;
            
        }
        break; 
    }

    // If all characters of message are printable, just print it as a string
    // with printf().  Otherwise use hex_dump() for non-printable messages.
    int printable = TRUE;
    for (size_t i = 0; printable && i < payload_length; ++i) {
        if (!isprint(data->payload[i])) {
            printable = FALSE;
        }
    }

    if (printable) {
        MONITOR.printf("%.*s\n\n", payload_length, data->payload);
    } else {
        hex_dump(data->payload, payload_length, HEX_DUMP_FLAG_OFFSET);
    }

    return 0;
}
int dump_tx_status(xbee_dev_t *xbee, const void FAR *frame, uint16_t length, void FAR *context)
{
    XBEE_UNUSED_PARAMETER(xbee);
    XBEE_UNUSED_PARAMETER(length);
    XBEE_UNUSED_PARAMETER(context);

    const xbee_frame_tx_status_t *tx_status = (const xbee_frame_tx_status_t*) frame;
    char buffer[40];
    const char *status = NULL;

    // Provide descriptive strings for the only two errors we expect
    // from sending User Data Relay frames.
    switch (tx_status->delivery) {
        case XBEE_TX_DELIVERY_INVALID_INTERFACE:
            status = "invalid interface";
            break;
        case XBEE_TX_DELIVERY_INTERFACE_BLOCKED:
            status = "interface blocked";
            break;
        default:
            sprintf(buffer, "unknown status 0x%X", tx_status->delivery);
            status = buffer;
    }

    printf("Error on message id 0x%02X: %s\n", tx_status->frame_id, status);

    return 0;
}
 
const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
    XBEE_FRAME_HANDLE_LOCAL_AT,
    { XBEE_FRAME_USER_DATA_RX, 0, user_data_rx, NULL },
    { XBEE_FRAME_TX_STATUS, 0, dump_tx_status, NULL },
    XBEE_FRAME_TABLE_END
};

int sendUserDataRelayAPIFrame(xbee_dev_t *xbee, const char *tx, const int num_tx)
{
    // Note: It's safe to pass the pointer here (no need to create a deep copy) because xbee_user_data_relay_tx() will write out the bytes right away.
    int ret = xbee_user_data_relay_tx(xbee, XBEE_USER_DATA_IF_MICROPYTHON, tx, num_tx);
    if (ret < 0)
    {
        printf("%s: ERROR: Failed to send frame to the XBee via serial. Error code: %d\n", __func__, ret);
        return ret;  // The value is negative so it contains the error code.
    }
    return 0;
}

uint32_t crc32(uint32_t crc, unsigned char *buf, size_t len)
{
    int k;

    crc = ~crc;
    while (len--) {
        crc ^= *buf++;
        for (k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
    }
    return ~crc;
}

//================================================== ACECON Funcitons =========================================*/

void set_HPS(bool value)
{
    aceconvalues_previous.hps = aceconvalues_current.hps;
    aceconvalues_current.hps = value;
    if (aceconvalues_current.hps){
        lv_obj_add_state(ui_SwitchHPS, LV_STATE_CHECKED);
    }else{
        lv_obj_clear_state(ui_SwitchHPS, LV_STATE_CHECKED);
    }
    MONITOR.printf("HPS Val Set to  %s\r\n",aceconvalues_current.hps?"HIGH":"LOW");
    digitalWrite(ACECON_HPS_OUT,value);
}

void set_PPS(bool value)
{
    aceconvalues_previous.pps = aceconvalues_current.pps;
    aceconvalues_current.pps = value;
    if (aceconvalues_current.pps){
        lv_obj_add_state(ui_SwitchPPS, LV_STATE_CHECKED);
    }else{
        lv_obj_clear_state(ui_SwitchPPS, LV_STATE_CHECKED);
    }
    MONITOR.printf("PPS Val Set to  %s\r\n",aceconvalues_current.pps?"HIGH":"LOW");
    digitalWrite(ACECON_PPS_OUT,value);
}

void set_PPT(bool value)
{    
    if (aceconvalues_current.ppt){
        lv_obj_add_state(ui_SwitchPPT, LV_STATE_CHECKED);
    }else{
        lv_obj_clear_state(ui_SwitchPPT, LV_STATE_CHECKED);
    }
    MONITOR.printf("PPT Val Set to  %s\r\n",aceconvalues_current.ppt?"HIGH":"LOW");
    digitalWrite(ACECON_PPT_OUT,value);
}

void set_ALM(bool value)
{
    aceconvalues_previous.alm = aceconvalues_current.alm;
    aceconvalues_current.alm = value;
    if (aceconvalues_current.alm){
        lv_obj_add_state(ui_SwitchALM, LV_STATE_CHECKED);
    }else{
        lv_obj_clear_state(ui_SwitchALM, LV_STATE_CHECKED);
    }
    MONITOR.printf("ALM Val Set to  %s\r\n",aceconvalues_current.alm?"HIGH":"LOW");
    digitalWrite(ACECON_ALM_OUT,value);
}

void send_init_packet() {
    int len = strlen(INIT_CSV_CSV);
    char* sz = (char*)malloc(6+len)+5;
    strcpy(sz,INIT_CSV_CSV);
    if(sz==(char*)5) {
        MONITOR.println("Out of memory. Tough luck.");
        return;
    }
    sz[len]=0;
    uint32_t crc = crc32(0,(uint8_t*)sz,len);
    uint8_t* p = (uint8_t*)(sz-5);
    *p=255;
    memcpy(p+1,&crc,4);
    sendUserDataRelayAPIFrame(&my_xbee,(char*)p,len+5);
    free(p);
}

//================================================== Callback Funcitons =========================================*/

static bool at_cmd_recv = false;

int on_xbee_at_cmd(const xbee_cmd_response_t FAR *response)
{
    at_cmd_recv = true;
    bool_t printable;
    uint_fast8_t length, i;
    uint8_t status;
    const uint8_t FAR *p;

    MONITOR.printf("\nResponse for: %s\n", response->command.str);
    
    last_at_cmd.commandstr[0] = response->command.str[0];
    last_at_cmd.commandstr[1] = response->command.str[1];

    if (response->flags & XBEE_CMD_RESP_FLAG_TIMEOUT)
    {
        MONITOR.println("(timed out)");
        return XBEE_ATCMD_DONE;
    }

    status = response->flags & XBEE_CMD_RESP_MASK_STATUS;
    if (status != XBEE_AT_RESP_SUCCESS)
    {
        MONITOR.printf("(error: %s)\n",
                      (status == XBEE_AT_RESP_ERROR) ? "general" : (status == XBEE_AT_RESP_BAD_COMMAND) ? "bad command"
                                                               : (status == XBEE_AT_RESP_BAD_PARAMETER) ? "bad parameter"
                                                               : (status == XBEE_AT_RESP_TX_FAIL)       ? "Tx failure"
                                                                                                        : "unknown error");
        return XBEE_ATCMD_DONE;
    }

    length = response->value_length;
    if (!length) // command sent successfully, no value to report
    {
        MONITOR.println("(success)");
        return XBEE_ATCMD_DONE;
    }

    // check to see if we can print the value out as a string
    printable = 1;
    p = response->value_bytes;
    for (i = length; printable && i; ++p, --i)
    {
        printable = isprint(*p);
    }

    if (printable)
    {
        MONITOR.printf("= \"%.*" PRIsFAR "\" ", length, response->value_bytes);
    }
    if (length <= 4)
    {
        // format hex string with (2 * number of bytes in value) leading zeros
        MONITOR.printf("= 0x%0*" PRIX32 " (%" PRIu32 ")\n", length * 2, response->value,
                      response->value);

        last_at_cmd.value = response->value;
        last_at_cmd.value_received = true;

    }
    else if (length <= 32)
    {
        // format hex string
        MONITOR.printf("= 0x");
        for (i = length, p = response->value_bytes; i; ++p, --i)
        {
            MONITOR.printf("%02X", *p);
        }
        MONITOR.println("");
    }
    else
    {
        MONITOR.printf("= %d bytes:\n", length);
        hex_dump(response->value_bytes, length, HEX_DUMP_FLAG_TAB);
    }

    return XBEE_ATCMD_DONE;
}

void on_xbee_error(COMMAND_ID id, STATUS_CODE code) {
    MONITOR.printf("XBee Error: Command (%d), Status (%d)\n",(int)id,(int)code);
    last_packet.status = (STATUS_CODE)0;
}

void on_monitor_init(const char* data) {
        // Initialize the AT Command layer for this XBee device and have the
        // driver query it for basic information (hardware version, firmware version,
        // serial number, IEEE address, etc.)
        xbee_cmd_init_device(&my_xbee);
        do {
            xbee_dev_tick(&my_xbee);
            status = xbee_cmd_query_status(&my_xbee);
        } while (status == -EBUSY);
        if (status) {
            MONITOR.printf("Error: (%d) waiting for query to complete.\n",status);
        }

}

void on_monitor_at(const char* data) {
    char sz[3];
    memcpy(sz,data+2,2);
    sz[2]=0;
    int16_t request = xbee_cmd_create(&my_xbee, sz);
    if (request < 0)
    {
        // Note that strerror() expects the positive error value
        // (what would have been stored in errno) so we have to
        // negate the xbee_cmd_create() return value.
        MONITOR.printf("Error creating request: %d\n",
                        request, strerror(-request));
    }
    else
    {

        MONITOR.println("Sending command to xbee");
        // if (ieee)
        // {
        //     xbee_cmd_set_target( request, ieee, WPAN_NET_ADDR_UNDEFINED);
        // }
        at_cmd_recv = false;
        xbee_cmd_set_callback(request, on_xbee_at_cmd, NULL);
        
        xbee_cmd_send(request);
    }
}

void on_monitor_connect(const char* str) {
    last_packet.cmd = COMMAND_ID::CONNECT;
    connect_packet data;
    memset(&data,0,sizeof(data));
    strcpy(data.host,"acek9server.com");
    strcpy(data.lastWillMessage,"disconnected");
    strcpy(data.lastWillTopic,"unit/wj00002/connection");
    strcpy(data.username,"wj00002");
    strcpy(data.password,"0Aa9YyKccy4DBDK8");
    strcpy(data.unitname,"wj00002");
    data.cleanSession = ACE_TRUE;
    data.lastWillQos = 1;
    data.port =8883;
    data.lastWillRetain = ACE_TRUE;
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending connect packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_data(const char* str) {
    last_packet.cmd = COMMAND_ID::DATA;
    data_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "data");
    data.qos = 1;
    data.retainFlag = ACE_FALSE;
    strcpy(data.timeStampUTC, "2020-09-11T08:02:17:350Z");

    data.powerOn = ACE_TRUE;
    data.ignitionOn = ACE_TRUE;
    data.eventCode = 90;
    data.cellStrength = 64;
    data.alarmOn = ACE_TRUE;
    data.leftTemp = 752;
    data.rightTemp = 746;
    data.stallSensorPresent = ACE_TRUE;
    data.stallCount = 0;
    data.batteryVoltage = 141;
    strcpy(data.doorPopUTC, "2020-09-11T08:02:17:350Z");
    data.version = 2;
    //data.newstuff = ACE_TRUE;

    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending data packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_connection(const char* str) {
    
    last_packet.cmd = COMMAND_ID::CONNECTION;

    connection_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "connection");
    data.qos = 1;
    data.retainFlag = ACE_TRUE;
    strcpy(data.status, "Online");
    
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending connection packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_status(const char* str) {
    
    last_packet.cmd = COMMAND_ID::STATUS;

    status_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "status");
    data.qos = 1;
    data.retainFlag = ACE_FALSE;

    strcpy(data.unitID, "V550B01100#12344");
    strcpy(data.unitname, "VP01234");
    strcpy(data.ctrlHeadSerialNumber, "09B");
    strcpy(data.unitFirmwareVersion, /*C502E4061G-10165*/"C502E4061G10165");
    strcpy(data.modemModel, "SARA-R410M-02B");
    strcpy(data.modemFirmwareVersion, "L0.0.00.00.05.08");
    strcpy(data.carrierCode, "A1");
    strcpy(data.mobileEquipmentID, "356726108107145");
    strcpy(data.integratedCircuitCardID, "89148000005057376071");
    data.doorPopCount = 3416;
    
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending status packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_log(const char* str) {
    
    last_packet.cmd = COMMAND_ID::LOG;

    log_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "logs");
    data.qos = 1;
    data.retainFlag = ACE_FALSE;
    strcpy(data.timeStampUTC, "2023-10-24T08:02:17.350Z");
    data.type = 0;
    strcpy(data.message, "Example Log Text");
    
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending log packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_config(const char* str) {
    
    last_packet.cmd = COMMAND_ID::CONFIG;

    config_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "config");
    data.qos = 1;
    data.retainFlag = ACE_FALSE;
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending config packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_subscribe7(const char* str) {
    
    last_packet.cmd = COMMAND_ID::CONFIG;

    command_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "config");
    data.qos = 1;
    data.retainFlag = ACE_FALSE;
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));

    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending config packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_subscribe8(const char* str) {
    
    last_packet.cmd = COMMAND_ID::COMMAND;

    command_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "command");
    data.qos = 1;
    data.retainFlag = ACE_FALSE;
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));

    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending config packet\n", status);
    }
    else 
    {
        
    }

    
}


void monitor_dev_tick(HardwareSerial& s) {
    if(s.available()) {
        String str = s.readString();
        MONITOR.printf("ECHO: %s\n",str.c_str());
        String cmd = str;
        cmd.toLowerCase();
        cmd.trim();
        if(cmd.substring(0,2)=="at") {
            on_monitor_at(str.c_str());
        } else if(cmd=="init") {
            on_monitor_init(str.c_str());
        } else if(cmd=="connect") {
            on_monitor_connect(str.c_str());
        } else if(cmd=="data") {
            on_monitor_data(str.c_str());
        } else if(cmd=="status") {
            on_monitor_status(str.c_str());
        } else if(cmd=="log") {
            on_monitor_log(str.c_str());  
        } else if(cmd=="connection") {
            on_monitor_connection(str.c_str());
        } else if (cmd=="config"){
            on_monitor_config(str.c_str());
        } else if (cmd=="subscribe7"){
            on_monitor_subscribe7(str.c_str()); 
        } else if (cmd=="subscribe8"){
            on_monitor_subscribe8(str.c_str());
        } else if (cmd=="save default settings"){
            systemsettings_current = systemsettings_default;    
            save_settings();            
        } else if (cmd=="save custom settings"){
            systemsettings_current.AlarmPower = PowerOpt::p_ManONManOFF;
            save_settings();
        } else if (cmd=="report settings"){
            print_settings();
        }
    }
}

float ConvertFtoC(float F)
{
	return (round((F - 32) * (5.0 / 9)));
}

void ui_update_acecon() {

    bool redrawFlag = false;

    if (tempvalues_current.valueChanged || tempvalues_current.unitsChanged){
        //MONITOR.println("UI: Temp Values Need Updating");
        redrawFlag = true;

        if(tempvalues_previous.leftTemp!=tempvalues_current.leftTemp || tempvalues_current.unitsChanged) {
            //MONITOR.println("UI: Updating Left Temp");
            static char szLeftTemp[6];
            if (systemsettings_current.AlarmUnitF){
                sprintf(szLeftTemp,"%3.1f",tempvalues_current.leftTemp);
            }else{
                sprintf(szLeftTemp,"%3.1f",ConvertFtoC(tempvalues_current.leftTemp));
            }
            
            lv_label_set_text_static(ui_LabelTemp1Val,szLeftTemp);
            lv_label_set_text_static(ui_LabelLeftTemp,szLeftTemp);
        }

        if (tempvalues_previous.rightTemp!=tempvalues_current.rightTemp || tempvalues_current.unitsChanged) {
            //MONITOR.println("UI: Updating Right Temp");
            static char szRightTemp[6];
            if (systemsettings_current.AlarmUnitF){
                sprintf(szRightTemp,"%3.1f",tempvalues_current.rightTemp);
            }else{
                sprintf(szRightTemp,"%3.1f",ConvertFtoC(tempvalues_current.rightTemp));
            }

            lv_label_set_text_static(ui_LabelTemp2Val,szRightTemp);
            lv_label_set_text_static(ui_LabelRightTemp,szRightTemp);
        }

        if (systemsettings_current.TempAveragingEnabled){
            //MONITOR.println("UI: Updating Avg Temp");
            static char szAvgTemp[6];

            if (systemsettings_current.AlarmUnitF){
                sprintf(szAvgTemp,"%3.1f",tempvalues_current.avgTemp);
            }else{
                sprintf(szAvgTemp,"%3.1f",ConvertFtoC(tempvalues_current.avgTemp));
            }

            lv_label_set_text_static(ui_LabelTempAvg,szAvgTemp);

        }else{
            lv_label_set_text_static(ui_LabelTempAvg,"Disabled");
        }
        
        if (tempvalues_current.unitsChanged){
            tempvalues_current.unitsChanged = false;
        }
        
        tempvalues_current.valueChanged = false;
    }


    if (enginevalues_current.valueChanged){
        //MONITOR.println("UI: Engine Values Need Updating");
        redrawFlag = true;

        //MONITOR.println("UI: Updating Ignition");
        if (enginevalues_current.ignitionOn) {
            //MONITOR.println("UI: Clearing Ignition State");
            lv_obj_clear_state(ui_ImgButtonKey, LV_STATE_PRESSED); 
                      
        } else {
            //MONITOR.println("UI: Adding Ignition State");
            lv_obj_add_state(ui_ImgButtonKey, LV_STATE_PRESSED);

                       
        }

        if(enginevalues_current.ignitionOn) {
            lv_obj_add_state(ui_SwitchIGN, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchIGN, LV_STATE_CHECKED);
        }

        
        if (enginevalues_current.engineStalled && systemsettings_current.StallMonitorEnabled){
            //MONITOR.println("UI: Adding Engine Stalled State");
            lv_obj_add_state(ui_ImgButtonEngine, LV_STATE_PRESSED);
        } else{
            //MONITOR.println("UI: Clearing Engine Stalled State");
            lv_obj_clear_state(ui_ImgButtonEngine, LV_STATE_PRESSED);
        }


        enginevalues_current.valueChanged = false;
    }
    

    if (battvalues_current.valueChanged){
        //MONITOR.println("UI: Temp Values Need Updating");
        redrawFlag = true;

        if (battvalues_current.error == true){
            lv_obj_add_state(ui_ImgButtonBattery, LV_STATE_PRESSED);
        }else{
            lv_obj_clear_state(ui_ImgButtonBattery, LV_STATE_PRESSED);
        }
        
        battvalues_current.valueChanged = false;
    }

    if (doorvalues_current.valueChanged){

        redrawFlag = true;

        if (systemsettings_current.doorDisabled){
            MONITOR.println("UI: Door Disabled");
            lv_obj_add_flag(ui_ImgButtonDoorClosed,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorOpen,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorPopped,LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_ImgButtonDoorDisabled,LV_OBJ_FLAG_HIDDEN);
        }else if  (doorvalues_current.doorPopped){
            MONITOR.println("UI: Door Popped");
            lv_obj_add_flag(ui_ImgButtonDoorClosed,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorOpen,LV_OBJ_FLAG_HIDDEN);            
            lv_obj_add_flag(ui_ImgButtonDoorDisabled,LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_ImgButtonDoorPopped,LV_OBJ_FLAG_HIDDEN);
        }else if  (doorvalues_current.doorOpen){
            MONITOR.println("UI: Door Opened");
            lv_obj_add_flag(ui_ImgButtonDoorClosed,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorPopped,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorDisabled,LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_ImgButtonDoorOpen,LV_OBJ_FLAG_HIDDEN);           
        }else{
            MONITOR.println("UI: Door Closed");
            lv_obj_add_flag(ui_ImgButtonDoorOpen,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorPopped,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorDisabled,LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_ImgButtonDoorClosed,LV_OBJ_FLAG_HIDDEN);
        }

        doorvalues_current.valueChanged = false;

    }

    if (xbee_signal_strength_changed){
        redrawFlag = true;

        MONITOR.print("Changing WiFi Icon:");

        if (xbee_signal_strength_current >= XBEE_SIGNALSTRENGTH0){
            MONITOR.println("0");
                lv_obj_clear_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }else if (xbee_signal_strength_current >= XBEE_SIGNALSTRENGTH1){
            MONITOR.println("1");
            lv_obj_add_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }else if (xbee_signal_strength_current >= XBEE_SIGNALSTRENGTH2){
            MONITOR.println("2");
            lv_obj_add_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }else if (xbee_signal_strength_current >= XBEE_SIGNALSTRENGTH3){
            MONITOR.println("3");
            lv_obj_add_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }else{
            MONITOR.println("4");
            lv_obj_add_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }

        xbee_signal_strength_changed = false;
    }

    // Redraw Screen
    if(redrawFlag){
        MONITOR.println("Redrawing Screen");

        lv_scr_load(lv_scr_act());
        redrawFlag = false;
    } 

       
}

void acedata_parse_temperature(String str){
       
    tempvalues_previous = tempvalues_current;
    
    //================================================== Left Temp =========================================*/
    //MONITOR.printf("Left Temp Sign: %c\n",str.charAt(ACEDATA_Temp1_Sign_POS));
    acedata_current.leftTempSign = (str.charAt(ACEDATA_Temp1_Sign_POS));
    
    // Check for Error First
    if (acedata_current.leftTempSign == TempErrorGeneral){
                
        if (str.charAt(ACEDATA_Temp1_10X_POS) == TempErrorOpen || str.charAt(ACEDATA_Temp1_10X_POS) == TempErrorSho){
            acedata_current.leftTempSign = str.charAt(ACEDATA_Temp1_10X_POS);
        }
        
        tempvalues_current.leftTempError = true;
        tempvalues_current.leftTempSign = acedata_current.leftTempSign;

    }
    else 
    {
        
        // Parse Temperature Magnitude	
        String ia =str.substring(ACEDATA_Temp1_1000X_POS,ACEDATA_Temp1_1X_POS+1);
       
        acedata_current.leftTemp = ((float)atoi(ia.c_str()))/10.0f;
        
        if(acedata_current.leftTempSign == TempSignNeg){
            acedata_current.leftTemp = acedata_current.leftTemp * -1;
        }

        tempvalues_current.leftTemp = acedata_current.leftTemp;

        tempvalues_current.leftTempError = false;

    }

    // MONITOR.print("Left Temp Value: ");
    // MONITOR.println(tempvalues_current.leftTemp);

    //================================================== Right Temp =========================================*/
    //MONITOR.printf("Right Temp Sign: %c\n",str.charAt(ACEDATA_Temp2_Sign_POS));
    acedata_current.rightTempSign = (str.charAt(ACEDATA_Temp2_Sign_POS));
    
    // Check for Error First
    if (acedata_current.rightTempSign == TempErrorGeneral){
                
        if (str.charAt(ACEDATA_Temp2_10X_POS) == TempErrorOpen || str.charAt(ACEDATA_Temp2_10X_POS) == TempErrorSho){
            acedata_current.rightTempSign = str.charAt(ACEDATA_Temp2_10X_POS);
        }
        
        tempvalues_current.rightTempError = true;
        tempvalues_current.rightTempSign = acedata_current.rightTempSign;

    }
    else 
    {
        
        // Parse Temperature Magnitude	
        String ia =str.substring(ACEDATA_Temp2_1000X_POS,ACEDATA_Temp2_1X_POS+1);
       
        acedata_current.rightTemp = ((float)atoi(ia.c_str()))/10.0f;
        
        if(acedata_current.rightTempSign == TempSignNeg){
            acedata_current.rightTemp = acedata_current.rightTemp * -1;
        }

        tempvalues_current.rightTemp = acedata_current.rightTemp;

        tempvalues_current.rightTempError = false;

    }

    //MONITOR.print("Right Temp Value: ");
    //MONITOR.println(tempvalues_current.rightTemp);

    //================================================== Average Temp =========================================*/

    // If Both Sensors have not returned an error
    if (tempvalues_current.leftTempError == false && tempvalues_current.rightTempError == false){
        
        tempvalues_current.avgTemp = (tempvalues_current.leftTemp + tempvalues_current.rightTemp)/2;
    }
    else
    {
        // If a single sensor is failed, used the working sensor value as the average
        if (tempvalues_current.leftTempError == true && tempvalues_current.rightTempError == false)
        {
            tempvalues_current.avgTemp = tempvalues_current.rightTemp;
        }
        else if (tempvalues_current.leftTempError == false && tempvalues_current.rightTempError == true)
        {
            tempvalues_current.avgTemp = tempvalues_current.leftTemp;
        }
    }

    //MONITOR.print("Average Temp Value: ");
    //MONITOR.println(tempvalues_current.avgTemp);

    if (tempvalues_previous.leftTemp!=tempvalues_current.leftTemp || tempvalues_previous.rightTemp!=tempvalues_current.rightTemp){
        tempvalues_current.valueChanged = true;
    }

}

void acedata_parse_battery(String str){
       
    battvalues_previous = battvalues_current;
    
    //================================================== Battery Value =========================================*/
    	
        String ia =str.substring(ACEDATA_Batt_100X_POS,ACEDATA_Batt_1X_POS+1);
       
        acedata_current.batteryVoltage = ((float)atoi(ia.c_str()))/10.0f;
    
    battvalues_current.voltage = acedata_current.batteryVoltage;

    // MONITOR.print("Battery Voltage: ");
    // MONITOR.println(battvalues_current.voltage);

    // Check if the Batt Voltage is out of range
	if (acedata_current.batteryVoltage < battvalues_current.voltage/10)
    {
        BadBatteryCounter++;
        if (BadBatteryCounter > MaxBadBattValCounter)
        {
            battvalues_current.error = true;
        }
    }
    else
    {
        BadBatteryCounter = 0;
        battvalues_current.error = false;
    }

    if (battvalues_previous.error != battvalues_current.error){
        battvalues_current.valueChanged = true;
    }

}

void acedata_parse_engine_stall(String str){

    enginevalues_previous = enginevalues_current;

    if (str.charAt(ACEDATA_Stall_Status_POS)=='0'){
        //MONITOR.println("Engine Stalled");
        acedata_current.engineStalled = true;
    }else{
        //MONITOR.println("Engine NOT Stalled");
        acedata_current.engineStalled = false;
    }

    String ia =str.substring(ACEDATA_Stall_Count_10X_POS,ACEDATA_Stall_Count_1X_POS+1);
       
        acedata_current.EngineStallCount = ((float)atoi(ia.c_str()))/10.0f;
        enginevalues_current.engineStallCount = acedata_current.EngineStallCount;
    
    //MONITOR.print("Engine Stall Count:");
    //MONITOR.println(enginevalues_current.engineStallCount);


    if (str.charAt(ACEDATA_Stall_Sensor_Present_POS) =='A'){
        //MONITOR.println("Engine Stall Sensor Present");
        acedata_current.engineStallSensorPresent = true;
    }else{
        //MONITOR.println("Engine Stall Sensor Not Present");
        acedata_current.engineStallSensorPresent = false;
    }

    enginevalues_current.engineStallSensorPresent = acedata_current.engineStallSensorPresent;

    if (enginevalues_current.engineStallCount >= EngineStallThreshold){
        //MONITOR.println("Stall Condition Achieved");
        enginevalues_current.engineStalled = true;
        
    }else{
        enginevalues_current.engineStalled = false;
    }

    if (enginevalues_previous.engineStalled != enginevalues_current.engineStalled){
        //MONITOR.println("Engine Stalled Valued Changed ");
        enginevalues_current.valueChanged = true;
    }


}

void acedata_parse_aux(String str){
    
    auxvalues_previous = auxvalues_current;

    if (str.charAt(ACEDATA_Aux1_Input_POS)=='1'){
        acedata_current.Aux1Input = true;
    }else{
        acedata_current.Aux1Input = false;
    }
    auxvalues_current.aux1Active = acedata_current.Aux1Input;

    if (str.charAt(ACEDATA_Aux2_Input_POS)=='1'){
        acedata_current.Aux2Input = true;
    }else{
        acedata_current.Aux2Input = false;
    }
    auxvalues_current.aux2Active = acedata_current.Aux2Input;

}

void acedata_parse_k9door(String str){
    
    doorvalues_previous = doorvalues_current;

    if (str.charAt(ACEDATA_K9Door_POS)=='1'){
        //MONITOR.println("Door Open");
        acedata_current.K9DoorOpen = true;
    }else{
        //MONITOR.println("Door Closed");
        acedata_current.K9DoorOpen = false;
    }

    doorvalues_current.doorOpen = acedata_current.K9DoorOpen;
    
    if (doorvalues_previous.doorOpen != doorvalues_current.doorOpen){
        //MONITOR.println("Door Value Changed");
        doorvalues_current.valueChanged = true;
    }

}

static void ui_switch_handler(lv_event_t * e)
{
    
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        bool checked = lv_obj_has_state(obj,LV_STATE_CHECKED);
        if(obj==ui_SwitchPPT) {
            MONITOR.printf("PPT Switch %s\r\n",checked?"on":"off");
            if (aceconvalues_previous.pps != aceconvalues_current.pps){
                set_PPT(checked?HIGH:LOW);
            }
        } else if(obj==ui_SwitchPPS) {
            MONITOR.printf("PPS Switch %s\r\n",checked?"on":"off");
            set_PPS(checked?HIGH:LOW);
        } else if(obj==ui_SwitchALM) {
            MONITOR.printf("ALM Switch %s\r\n",checked?"on":"off");
            set_ALM(checked?HIGH:LOW);
        } else if(obj==ui_SwitchHPS) {
            MONITOR.printf("HPS Switch %s\r\n",checked?"on":"off");
            set_HPS(checked?HIGH:LOW);
        }
    }
}

static void menu_AlarmPower_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        MONITOR.print("Alarm Power Changed To: ");
        systemsettings_previous.AlarmPower = systemsettings_current.AlarmPower;

        switch(lv_dropdown_get_selected(obj)){

            case 0:
                MONITOR.println("CarON/CarOFF");
                systemsettings_current.AlarmPower = PowerOpt::p_CarONCarOFF;
            break;

            case 1:
                MONITOR.println("CarON/ManOFF");
                systemsettings_current.AlarmPower = PowerOpt::p_CarONManOFF;
            break;

            case 2:
                MONITOR.println("ManON/ManOFF");
                systemsettings_current.AlarmPower = PowerOpt::p_ManONManOFF;
            break;

            case 3:
                MONITOR.println("NO K9 LEFT BEHIND");
                systemsettings_current.AlarmPower = PowerOpt::p_NoK9Left;
            break;

            case 4:
                MONITOR.println("Always OFF");
                systemsettings_current.AlarmPower = PowerOpt::p_OFF;
            break;

            default:
                MONITOR.println("Error: Unknown Power Option");
            break;

        }
       
    }

}

static void menu_DoorPower_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        MONITOR.print("Door Power Changed To: ");
        systemsettings_previous.DoorPower = systemsettings_current.DoorPower;

        switch(lv_dropdown_get_selected(obj)){

            case 0:
                MONITOR.println("CarON/CarOFF");
                systemsettings_current.DoorPower = PowerOpt::p_CarONCarOFF;
            break;

            case 1:
                MONITOR.println("CarON/ManOFF");
                systemsettings_current.DoorPower = PowerOpt::p_CarONManOFF;
            break;

            case 2:
                MONITOR.println("ManON/ManOFF");
                systemsettings_current.DoorPower = PowerOpt::p_ManONManOFF;
            break;

            case 3:
                MONITOR.println("Always OFF");
                systemsettings_current.DoorPower = PowerOpt::p_OFF;
            break;

            default:
                MONITOR.println("Error: Unknown Power Option");
            break;

        }
       
    }

}

static String print_Battery_Voltage_Settings(Batt b){
    
    String str;

    switch(systemsettings_current.BatteryVoltage){

        case Batt::b_10:
            return "10.0V";
        break;

        case Batt::b_105:
            return "10.5V";
        break;

        case Batt::b_11:
            return "11.0V";
        break;

        case Batt::b_115:
            return "11.5V";
        break;

        case Batt::b_12:
            return "12.0V";
        break;

        default:
            return "";
        break;

    }
}

static void menu_BattVoltSetDown_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        MONITOR.print("Battery Voltage Setting Changed To: ");
        systemsettings_previous.BatteryVoltage = systemsettings_current.BatteryVoltage;

        switch (systemsettings_current.BatteryVoltage){

            case Batt::b_10:
                // Minimum Value
            break;

            case Batt::b_105:
                MONITOR.println("10.0V");
                systemsettings_current.BatteryVoltage = Batt::b_10;
            break;

            case Batt::b_11:
                MONITOR.println("10.5V");
                systemsettings_current.BatteryVoltage = Batt::b_105;                
            break;

            case Batt::b_115:
                MONITOR.println("11.0V");
                systemsettings_current.BatteryVoltage = Batt::b_11;
            break;

            case Batt::b_12:
                MONITOR.println("11.5V");
                systemsettings_current.BatteryVoltage = Batt::b_115;
            break;

        }

        lv_label_set_text(ui_LabelBattVoltSetValue,print_Battery_Voltage_Settings(systemsettings_current.BatteryVoltage).c_str());
       
    }

}

static void menu_BattVoltSetUp_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        MONITOR.print("Battery Voltage Setting Changed To: ");
        systemsettings_previous.BatteryVoltage = systemsettings_current.BatteryVoltage;

        switch (systemsettings_current.BatteryVoltage){

            case Batt::b_10:
                MONITOR.println("10.5V");
                systemsettings_current.BatteryVoltage = Batt::b_105;
            break;

            case Batt::b_105:
                MONITOR.println("11.0V");
                systemsettings_current.BatteryVoltage = Batt::b_11;
            break;

            case Batt::b_11:
                MONITOR.println("11.5V");
                systemsettings_current.BatteryVoltage = Batt::b_115;                
            break;

            case Batt::b_115:
                MONITOR.println("12.0V");
                systemsettings_current.BatteryVoltage = Batt::b_12;
            break;

            case Batt::b_12:
                // Maximum Value
            break;

        }

        lv_label_set_text(ui_LabelBattVoltSetValue,print_Battery_Voltage_Settings(systemsettings_current.BatteryVoltage).c_str());
       
    }

}

static String print_Alarm_Hot_Set(int index){
    char sz[16];
    const int* arr = systemsettings_current.AlarmUnitF?HotTempOpt_F:HotTempOpt_C;
    snprintf(sz,sizeof(sz),"%d %c",arr[index],systemsettings_current.AlarmUnitF?'F':'C');
    return sz;
    
}

static void menu_AlarmHotSetUp_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        MONITOR.print("Alarm Hot Set Changed To: ");
        systemsettings_previous.AlarmHotSetIndex = systemsettings_current.AlarmHotSetIndex;

        if (systemsettings_current.AlarmHotSetIndex == HotTempArraySize-1){
            MONITOR.println("MAXIMUM VALUE");
        }else{
            systemsettings_current.AlarmHotSetIndex++;
            MONITOR.println(print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex));
        }
        lv_label_set_text(ui_LabelAlarmHotSetValue,print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex).c_str());
        
    }

}

static void menu_AlarmHotSetDown_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        MONITOR.print("Alarm Hot Set Changed To: ");
        systemsettings_previous.AlarmHotSetIndex = systemsettings_current.AlarmHotSetIndex;

        if (systemsettings_current.AlarmHotSetIndex == 0){
            // Minimum Value
        }else{
            systemsettings_current.AlarmHotSetIndex--;
            MONITOR.println(print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex));
        }
        lv_label_set_text(ui_LabelAlarmHotSetValue,print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex).c_str());
        
    }

}

static String print_Alarm_Cold_Set(int index){
    char sz[16];
    const int* arr = systemsettings_current.AlarmUnitF?ColdTempOpt_F:ColdTempOpt_C;
    snprintf(sz,sizeof(sz),"%d %c",arr[index],systemsettings_current.AlarmUnitF?'F':'C');
    return sz;
    
}

static void menu_AlarmColdSetUp_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        MONITOR.print("Alarm Cold Set Changed To: ");
        systemsettings_previous.AlarmColdSetIndex = systemsettings_current.AlarmColdSetIndex;

        if (systemsettings_current.AlarmColdSetIndex == ColdTempArraySize-1){
            MONITOR.println("MAXIMUM VALUE");
        }else{
            systemsettings_current.AlarmColdSetIndex++;
            MONITOR.println(print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex));
        }
        lv_label_set_text(ui_LabelAlarmColdSetValue,print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex).c_str());
        
    }

}

static void menu_AlarmColdSetDown_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        MONITOR.print("Alarm Cold Set Changed To: ");
        systemsettings_previous.AlarmColdSetIndex = systemsettings_current.AlarmColdSetIndex;

        if (systemsettings_current.AlarmColdSetIndex == 0){
            // Minimum Value
        }else{
            systemsettings_current.AlarmColdSetIndex--;
            MONITOR.println(print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex));
        }
        if (systemsettings_current.AlarmColdSetIndex==0){
            lv_label_set_text(ui_LabelAlarmColdSetValue,"OFF");
        }else{
            lv_label_set_text(ui_LabelAlarmColdSetValue,print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex).c_str());
        }
        
        
    }

}

static void menu_TempUnits_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        MONITOR.print("Temp Units Changed To: ");
        systemsettings_previous.AlarmUnitF = systemsettings_current.AlarmUnitF;

        switch(lv_dropdown_get_selected(obj)){

            case 0:
                MONITOR.println("Farenheit");
                systemsettings_current.AlarmUnitF = true;
            break;

            case 1:
                MONITOR.println("Celsius");
                systemsettings_current.AlarmUnitF = false;
            break;
           
            default:
                MONITOR.println("Error: Unknown Temp Units");
            break;

        }
    
    lv_label_set_text(ui_LabelAlarmColdSetValue,print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex).c_str());
    lv_label_set_text(ui_LabelAlarmHotSetValue,print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex).c_str()); 

    tempvalues_current.unitsChanged = true;
    ui_update_acecon();

    }

}

static void menu_TempAveraging_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        MONITOR.print("Temp Averaging Changed To: ");
        systemsettings_previous.TempAveragingEnabled = systemsettings_current.TempAveragingEnabled;

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED){
            MONITOR.println("True");
            systemsettings_current.TempAveragingEnabled = true;
        }else{
            MONITOR.println("False");
            systemsettings_current.TempAveragingEnabled = false;
        }
            
    tempvalues_current.unitsChanged = true;
    ui_update_acecon();

    }

}

static void menu_AutoSnooze_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        MONITOR.print("Auto Snooze Changed To: ");
        systemsettings_previous.AutoSnoozeEnabled = systemsettings_current.AutoSnoozeEnabled;

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED){
            MONITOR.println("True");
            systemsettings_current.AutoSnoozeEnabled = true;
        }else{
            MONITOR.println("False");
            systemsettings_current.AutoSnoozeEnabled= false;
        }
 
    }

}

static void menu_AuxInput_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        MONITOR.print("Aux Input Changed To: ");
        systemsettings_previous.AuxInputEnabled = systemsettings_current.AuxInputEnabled;

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED){
            MONITOR.println("True");
            systemsettings_current.AuxInputEnabled = true;
        }else{
            MONITOR.println("False");
            systemsettings_current.AuxInputEnabled= false;
        }
            

    }

}

static void menu_StallMonitor_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        MONITOR.print("Stall Monitor Enabled Changed To: ");
        systemsettings_previous.StallMonitorEnabled = systemsettings_current.StallMonitorEnabled;

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED){
            MONITOR.println("True");
            systemsettings_current.StallMonitorEnabled = true;
        }else{
            MONITOR.println("False");
            systemsettings_current.StallMonitorEnabled= false;
        }
            
        enginevalues_current.valueChanged = true;
        ui_update_acecon();

    }

}

static void menu_ExitMenu_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        MONITOR.println("Exiting Menu");

        MONITOR.println("Saving Settings");
        save_settings();
       
    }

}

void update_menu_settings(){

    lv_dropdown_set_selected(ui_DropdownAlarmPower,(uint16_t)systemsettings_current.AlarmPower);
    lv_dropdown_set_selected(ui_DropdownDoorPower,(uint16_t)systemsettings_current.DoorPower);
    lv_label_set_text(ui_LabelBattVoltSetValue,print_Battery_Voltage_Settings(systemsettings_current.BatteryVoltage).c_str());
    lv_label_set_text(ui_LabelAlarmHotSetValue,print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex).c_str());
    lv_label_set_text(ui_LabelAlarmColdSetValue,print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex).c_str());
    systemsettings_current.AlarmUnitF? lv_dropdown_set_selected(ui_DropdownTempUnits,systemsettings_current.AlarmUnitF?0:1) : lv_dropdown_set_selected(ui_DropdownTempUnits,systemsettings_current.AlarmUnitF?0:1);
    systemsettings_current.AutoSnoozeEnabled? lv_obj_add_state(ui_CheckboxAutoSnooze,LV_STATE_CHECKED) : lv_obj_clear_state(ui_CheckboxAutoSnooze,LV_STATE_CHECKED);
    systemsettings_current.AuxInputEnabled? lv_obj_add_state(ui_CheckboxAuxIn,LV_STATE_CHECKED) : lv_obj_clear_state(ui_CheckboxAuxIn,LV_STATE_CHECKED);
    systemsettings_current.StallMonitorEnabled? lv_obj_add_state(ui_CheckboxStallMonitor,LV_STATE_CHECKED) : lv_obj_clear_state(ui_CheckboxStallMonitor,LV_STATE_CHECKED);
    systemsettings_current.TempAveragingEnabled? lv_obj_add_state(ui_CheckboxTemperatureAveraging,LV_STATE_CHECKED) : lv_obj_clear_state(ui_CheckboxTemperatureAveraging,LV_STATE_CHECKED);
    
}

void acedata_parse_serialnumber(String str){
    int i;
    for (i =0; i<ACEDATA_VIM_SN_LENGTH;i++){
        VIMSerialNumber[i]=str.charAt(ACEDATA_VIM_SN_POS+i);
    }
    VIMSerialNumber[i]=0;

}

void process_vim(const char* incoming, void* state) {
    static String trickle;
    vim_data data = vim_load();
    if(*incoming) {
        String s(incoming);
        //MONITOR.printf("Incoming! %s\n",incoming);
        trickle +=s;
        
    }
    int newlineIndex = trickle.indexOf("\n");
    int dollarsignIndex = trickle.indexOf("$");

    //MONITOR.printf("Received trickle: %s\n",trickle.c_str());

    while(newlineIndex > -1) {

        if (dollarsignIndex == -1){
            
            trickle = trickle.substring(newlineIndex+1);
            //MONITOR.printf("No Dollar Sign: Reset trickle: %s\n",trickle.c_str());
        }else{
            String line = trickle.substring(dollarsignIndex,newlineIndex+1);
            line.trim();
            //MONITOR.printf("Parsed line: %s\n",line.c_str());

            trickle = trickle.substring(newlineIndex+1);
            //MONITOR.printf("Reset trickle: %s\n",trickle.c_str());
                    
            int index=0;
            int length=10;

            if(line.substring(index,length)=="$ACEK9,IH1") {
                //MONITOR.printf("Parsed line: %s\n",line.c_str());
                acedata_previous = acedata_current;
                acedata_parse_temperature(line);
                acedata_parse_battery(line);
                acedata_parse_engine_stall(line);
                acedata_parse_aux(line);
                acedata_parse_k9door(line);
                acedata_parse_serialnumber(line);

            }

            if(line.substring(index,length)=="$ACEK9,IH2") {
                //MONITOR.printf("Parsed line: %s\n",line.c_str());
                
            }

            if(line.substring(index,length)=="$ACEK9,IP1"){
                //MONITOR.printf("Parsed line: %s\n",line.c_str());
                vim_write_sz("$ACEK9,CH1A,C502E5A63G-DEV0103B2C502E5A63GDADA00\r\n");

            } 

            newlineIndex = trickle.indexOf("\n");
        
        }
    
    }
      

    //vim_write_sz()
}

void acecon_dev_tick() {
    
    //================================================== Read ACECON Inputs =========================================*/
    aceconvalues_previous = aceconvalues_current;
    aceconvalues_current.ppt = digitalRead(ACECON_POP_IN);
    aceconvalues_current.pps = digitalRead(ACECON_PPS_IN);
    aceconvalues_current.ign = digitalRead(ACECON_IGN_IN);

    if(aceconvalues_previous.ppt != aceconvalues_current.ppt) {
        MONITOR.printf("PPT Val Changed to  %s\r\n",aceconvalues_current.ppt?"HIGH":"LOW");
        aceconvalues_current.valueChanged = true;

        if(aceconvalues_current.ppt) {
            lv_obj_add_state(ui_SwitchPPT, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchPPT, LV_STATE_CHECKED);
        }
    }
    if(aceconvalues_previous.pps != aceconvalues_current.pps) {
        MONITOR.printf("PPS Val Changed to  %s\r\n",aceconvalues_current.pps?"HIGH":"LOW");
        aceconvalues_current.valueChanged = true;

        if(aceconvalues_current.pps) {
            lv_obj_add_state(ui_SwitchPPS, LV_STATE_CHECKED);
            systemsettings_current.doorDisabled = false;
        } else {
            lv_obj_clear_state(ui_SwitchPPS, LV_STATE_CHECKED);
            systemsettings_current.doorDisabled = true;
        }
    }
    if(aceconvalues_previous.ign != aceconvalues_current.ign) {
        enginevalues_previous.ignitionOn = enginevalues_current.ignitionOn;
        enginevalues_current.ignitionOn = aceconvalues_current.ign;
        MONITOR.printf("IGN Val Changed to  %s\r\n",enginevalues_current.ignitionOn?"HIGH":"LOW");
        enginevalues_current.valueChanged = true; 
        aceconvalues_current.valueChanged = true;
       
    }
    if(aceconvalues_previous.hps != aceconvalues_current.hps) {
        MONITOR.printf("HPS Val Changed to  %s\r\n",aceconvalues_current.hps?"HIGH":"LOW");
        aceconvalues_current.valueChanged = true;

        if(aceconvalues_current.hps) {
            lv_obj_add_state(ui_SwitchHPS, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchHPS, LV_STATE_CHECKED);
        }
    }
    if(aceconvalues_previous.alm != aceconvalues_current.alm) {
        MONITOR.printf("ALM Val Changed to  %s\r\n",aceconvalues_current.alm?"HIGH":"LOW");
        aceconvalues_current.valueChanged = true;

        if(aceconvalues_current.alm) {
            lv_obj_add_state(ui_SwitchALM, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchALM, LV_STATE_CHECKED);
        }
    }

    
    

}

void check_door_condition(){

    // TODO: fill in Door POPPED and DOOR Disabled condition checks
    // Place Holder Values
    systemsettings_current.doorDisabled = false;
    doorvalues_current.doorPopped = false;

}

void check_cell_signal(){
    
    if (xbee_cell_connected && (millis() - signal_strength_timer > signal_strength_timer_threshold )){
        signal_strength_timer = millis();
        
        MONITOR.println("Checking Cell Signal");
        String str = "atdb";
        MONITOR.println("Sending:" + str);
        
        on_monitor_at(str.c_str()); 
    }

    if (last_at_cmd.commandstr[0] == 'd' && last_at_cmd.commandstr[1] == 'b' && last_at_cmd.value_received){
        MONITOR.println("Received db Response");
        if (xbee_signal_strength_current != last_at_cmd.value){
            xbee_signal_strength_changed = true;
        }
        xbee_signal_strength_current = last_at_cmd.value;
        
        last_at_cmd.value_received = false;

    }

}

void setup() {
    
    SPIFFS.begin(true);
    Wire.begin( 1,42,100*1000); 
    lv_init();
    display_init();
    input_init();
    ui_init();
    vim_init(process_vim);

    //================================================== Menu Events =========================================*/
    lv_obj_add_event_cb(ui_DropdownAlarmPower, menu_AlarmPower_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_DropdownDoorPower, menu_DoorPower_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonBattVoltSetUp, menu_BattVoltSetUp_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonBattVoltSetDown, menu_BattVoltSetDown_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonHotSetUp, menu_AlarmHotSetUp_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonHotSetDown, menu_AlarmHotSetDown_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonColdSetUp, menu_AlarmColdSetUp_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonColdSetDown, menu_AlarmColdSetDown_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_DropdownTempUnits, menu_TempUnits_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_CheckboxTemperatureAveraging, menu_TempAveraging_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_CheckboxAutoSnooze, menu_AutoSnooze_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_CheckboxAuxIn, menu_AuxInput_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_CheckboxStallMonitor, menu_StallMonitor_handler, LV_EVENT_ALL, NULL);

    lv_obj_add_event_cb(ui_ImgButtonExitMenu1, menu_ExitMenu_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonExitMenu2, menu_ExitMenu_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonExitMenu3, menu_ExitMenu_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonExitMenu4, menu_ExitMenu_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ImgButtonExitMenu5, menu_ExitMenu_handler, LV_EVENT_ALL, NULL);

    //================================================== ACECON Screen Events =========================================*/
    lv_obj_add_event_cb(ui_SwitchPPT, ui_switch_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwitchPPS, ui_switch_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwitchALM, ui_switch_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwitchHPS, ui_switch_handler, LV_EVENT_ALL, NULL);
    dimmer.max_level(.0625);
    //================================================== XBEE Setup =========================================*/
#ifndef CUSTOM
    XBEE.begin(115200, SERIAL_8N1, 18, 17);
#else
    XBEE.begin(115200, SERIAL_8N1, XBEE_RXD, XBEE_TXD);
#endif
    MONITOR.begin(115200);
    MONITOR.printf("Booted\n");
    XBEE_SERPORT.ser = &XBEE;
    XBEE_SERPORT.baudrate = 115200;
    strcpy(XBEE_SERPORT.portname,"XBEE");
#ifndef CUSTOM
    XBEE_SERPORT.pin_rx = 18;
    XBEE_SERPORT.pin_tx = 17;
#else
    XBEE_SERPORT.pin_rx = 4;
    XBEE_SERPORT.pin_tx = 5;
#endif
    if (xbee_dev_init(&my_xbee, &XBEE_SERPORT, NULL, NULL)) {
        MONITOR.printf("Failed to initialize device.\n");
        while(1);
    }

    //================================================== XBEE Setup =========================================*/
    pinMode(XBEE_RSSI,INPUT);
    pinMode(XBEE_RESET,OUTPUT);
    pinMode(XBEE_CMD,INPUT);
    pinMode(XBEE_LINK,INPUT);

    digitalWrite(XBEE_RESET,LOW);


    //================================================== ACECON Setup =========================================*/
    pinMode(ACECON_PPS_IN,INPUT);
    pinMode(ACECON_IGN_IN,INPUT);
    pinMode(ACECON_POP_IN,INPUT);

    pinMode(ACECON_PPS_OUT,OUTPUT); 
    pinMode(ACECON_PPT_OUT,OUTPUT);
    pinMode(ACECON_HPS_OUT,OUTPUT);
    pinMode(ACECON_ALM_OUT,OUTPUT);

    digitalWrite(ACECON_PPT_OUT,LOW);
    digitalWrite(ACECON_PPS_OUT,LOW);
    digitalWrite(ACECON_ALM_OUT,LOW);
    digitalWrite(ACECON_HPS_OUT,LOW);
      
    //================================================== Set System Defaults =========================================*/
    
    systemsettings_default.AlarmPower = p_AlwaysOFF;
    systemsettings_default.AutoSnoozeEnabled = false;
    systemsettings_default.AuxInputEnabled = false;
    systemsettings_default.BatteryVoltage = b_12;
    systemsettings_default.DoorPower = p_CarONCarOFF;
    systemsettings_default.StallMonitorEnabled = true;
    systemsettings_default.TempAveragingEnabled = true;
    systemsettings_default.doorDisabled = false;
    systemsettings_default.AlarmUnitF = true;
    systemsettings_default.AlarmHotSetIndex = 3;
    systemsettings_default.AlarmColdSetIndex = 7;

    SettingsLoaded = load_settings();

    update_menu_settings();

    systemsettings_previous = systemsettings_current;


    acedata_current.ValueChanged = false;
    tempvalues_current.valueChanged = true;
    enginevalues_current.valueChanged = true;
    doorvalues_current.valueChanged = true;

    set_HPS(HIGH);
    set_PPS(HIGH);


    MONITOR.printf("Exiting Setup()\n");

    
}



void loop() {

    //Check XBee 
    if (last_packet.cmd == COMMAND_ID::ACKNOWLEDGE){
                
        if (last_packet.status == STATUS_CODE::XBEE_INITIALIZED){
            
            if (xbee_initialized){
                MONITOR.println("====================WARNING=============================");
                MONITOR.println("XBEE Reset During Operation");
                xbee_reset = true;
            }

            MONITOR.println("XBEE Initialized");
            xbee_initialized = true;
            
            last_packet.cmd = (COMMAND_ID)NULL;
            last_packet.status = (STATUS_CODE)NULL;

        }else if (last_packet.status == STATUS_CODE::XBEE_CELL_CONNECTED){
            
            MONITOR.println("XBEE Cell Connected");
             xbee_cell_connected = true;
            
            send_init_packet();
            

            last_packet.cmd = (COMMAND_ID)NULL;
            last_packet.status = (STATUS_CODE)NULL;
        }


    }

    check_cell_signal();

    display_update();
    lv_timer_handler();
    
    monitor_dev_tick(MONITOR);

    xbee_dev_tick(&my_xbee);
    if(((int)last_packet.status)<0) {
        on_xbee_error(last_packet.cmd, last_packet.status);
    }

    acecon_dev_tick();
    check_door_condition();

    ui_update_acecon();

}

