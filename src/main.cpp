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
//================================================== Temperature Variables =========================================
enum TempSign {ts_None,ts_Neg = 45,ts_Pos_IBox = 32, ts_Pos_VIM = 43,ts_Error = 69, ts_Opn = 53, ts_Sho = 52};// Pos = "+" or " ", Neg = "-", Error = "E", Open = "5", Sho = "4"
enum TempState {tst_OK,tst_Warning,tst_Over, tst_OverPlus,tst_Under};
struct TempValues{
    float leftTemp;
    bool leftTempError;
    TempSign leftTempSign;
    TempState leftTempState;
    float rightTemp;
    bool rightTempError;
    TempSign rightTempSign;
    TempState rightTempState;
    float avgTemp;
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
};

EngineValues enginevalues_current;
EngineValues enginevalues_previous;

//================================================== Aux Variables =========================================*/
struct AuxValues{
    bool aux1Active;
    bool aux2Active;
};

AuxValues auxvalues_current;
AuxValues auxvalues_previous;

//================================================== K9 Door Variables =========================================*/
struct DoorValues{
    bool DoorOpen;
    bool DoorPopped;
    bool DoorDisabled;
};

DoorValues doorvalues_current;
DoorValues doorvalues_previous;

//================================================== VIM Variables =========================================*/
String VIMSerialNumber;


//================================================== GLOBAL VARIABLES =========================================*/

struct last_info {
    STATUS_CODE status; // = STATUS_CODE::SUCCESS;
    COMMAND_ID cmd; // = (COMMAND_ID)0;
};
static last_info last = {STATUS_CODE::SUCCESS,(COMMAND_ID)0};

static config_packet config_data;

static command_packet command_data;

struct acecon {
    bool alm;
    bool hps;
    bool ppt;
    bool pps;
    bool ign;
    bool ValueChanged;
};
acecon aceconvalues_current = {false,false,false,false,false,false};
acecon aceconvalues_previous = {false,false,false,false,false,false};

struct acedata{
    TempSign leftTempSign;
    float leftTemp;
    TempSign rightTempSign;
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

enum PowerOpt {p_CarONCarOFF,p_CarONManOFF,p_ManONManOFF,p_NoK9Left,p_OFF,p_AutoStart};
enum Batt {b_10 = 1,b_105 = 2,b_11 = 3,b_115 =4,b_12 =5};

struct SystemSetting{
    bool TempAveragingEnabled;
    bool AutoSnoozeEnabled;
    bool StallMonitorEnabled;
    bool AuxInputEnabled;
    PowerOpt AlarmPower;
    PowerOpt DoorPower;
    Batt BatteryVoltage;

};

SystemSetting systemsetting_current;
SystemSetting systemsetting_previous;
SystemSetting systemsetting_default;

//================================================== XBEE STUFF =========================================*/
    xbee_serial_t XBEE_SERPORT;
    int status;
    xbee_dev_t my_xbee;

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
            memcpy(&pck,payload+1,payload_length-1);
            MONITOR.printf("Acknowledge Packet Received: %d\n",pck.status);
            last.cmd = pck.cmd_ID;
            last.status = pck.status;
            last_received = true;
        }
        break;
        case COMMAND_ID::COMMAND: {
            MONITOR.println("Command Packet Received");
            command_packet pck;
            memcpy(&pck,payload+1,payload_length-1);
            last.cmd = pck.cmd_ID;
            command_data = pck;
            last_received = true;
        }
        break;
        case COMMAND_ID::CONFIG: {
            MONITOR.println("Config Packet Received");
            config_packet pck;
            memcpy(&pck,payload+1,payload_length-1);
            
            last.cmd = pck.cmd_ID;
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
    MONITOR.printf("HPS Val Changed to  %s\r\n",aceconvalues_current.hps?"HIGH":"LOW");
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
    MONITOR.printf("PPS Val Changed to  %s\r\n",aceconvalues_current.pps?"HIGH":"LOW");
    digitalWrite(ACECON_PPS_OUT,value);
}

void set_PPT(bool value)
{    
    if (aceconvalues_current.ppt){
        lv_obj_add_state(ui_SwitchPPT, LV_STATE_CHECKED);
    }else{
        lv_obj_clear_state(ui_SwitchPPT, LV_STATE_CHECKED);
    }
    MONITOR.printf("PPT Val Changed to  %s\r\n",aceconvalues_current.ppt?"HIGH":"LOW");
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
    MONITOR.printf("ALM Val Changed to  %s\r\n",aceconvalues_current.alm?"HIGH":"LOW");
    digitalWrite(ACECON_ALM_OUT,value);
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
    last.status = (STATUS_CODE)0;
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
    last.cmd = COMMAND_ID::CONNECT;
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
    payload[0]=(uint8_t)last.cmd;
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
    last.cmd = COMMAND_ID::DATA;
    data_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.TopicName, "data");
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

    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last.cmd;
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
    
    last.cmd = COMMAND_ID::CONNECTION;

    connection_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.TopicName, "connection");
    data.qos = 1;
    data.retainFlag = ACE_TRUE;
    strcpy(data.status, "Online");
    
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last.cmd;
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
    
    last.cmd = COMMAND_ID::STATUS;

    status_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.TopicName, "status");
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
    payload[0]=(uint8_t)last.cmd;
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
    
    last.cmd = COMMAND_ID::LOG;

    log_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.TopicName, "logs");
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
    payload[0]=(uint8_t)last.cmd;
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
    
    last.cmd = COMMAND_ID::CONFIG;

    config_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.TopicName, "config");
    data.qos = 1;
    data.retainFlag = ACE_FALSE;
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last.cmd;
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

void on_monitor_command(const char* str) {
    
    last.cmd = COMMAND_ID::COMMAND;

    command_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.TopicName, "command");
    data.qos = 1;
    data.retainFlag = ACE_FALSE;
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last.cmd;
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
        } else if (cmd=="command"){
            on_monitor_command(str.c_str());
        } else if(cmd.substring(0,9)=="lefttemp "){
            MONITOR.printf("Left Temp Changed\n");
            acedata_current.leftTemp = cmd.substring(9).toFloat();
            static char szLeftTemp[6];
            sprintf(szLeftTemp,"%3.1f",tempvalues_current.leftTemp);
            MONITOR.printf(szLeftTemp);
            acedata_current.ValueChanged = true;
        }
    }
}

byte ASCII2Num(char asciival){
	return byte(asciival) - 48;
}

void ui_update_acecon() {

    if( tempvalues_previous.leftTemp!=tempvalues_current.leftTemp) {
        MONITOR.printf("Left Temp Changed\n");
        static char szLeftTemp[6];
        sprintf(szLeftTemp,"%3.1f",tempvalues_current.leftTemp);
        lv_label_set_text_static(ui_LabelTemp1Val,szLeftTemp);
    }
    if (tempvalues_previous.rightTemp!=tempvalues_current.rightTemp) {
        static char szRightTemp[6];
        sprintf(szRightTemp,"%3.1f",tempvalues_current.rightTemp);
        lv_label_set_text_static(ui_LabelTemp2Val,szRightTemp);
    }

    if (systemsetting_current.TempAveragingEnabled ==true){
        static char szAvgTemp[6];
        sprintf(szAvgTemp,"%3.1f",tempvalues_current.avgTemp);
        lv_label_set_text_static(ui_LabelTempAvg,szAvgTemp);
    }else{
        lv_label_set_text_static(ui_LabelTempAvg,"");
    }

    if (aceconvalues_current.ign) {
            lv_obj_clear_state(ui_ImgButtonKey, LV_STATE_PRESSED);            
        } else {
            lv_obj_add_state(ui_ImgButtonKey, LV_STATE_PRESSED);
        }

    if (battvalues_current.error == true){
        lv_obj_add_state(ui_ImgButtonBattery, LV_STATE_DISABLED);
    }else{
        lv_obj_clear_state(ui_ImgButtonBattery, LV_STATE_DISABLED);
    }

    if (enginevalues_current.engineStalled == true){
        lv_obj_add_state(ui_ImgButtonEngine, LV_STATE_DISABLED);
    }else{
        lv_obj_clear_state(ui_ImgButtonEngine, LV_STATE_DISABLED);
    }
    
    if (doorvalues_current.DoorDisabled == true){
        lv_obj_add_state(ui_ImgButtonEngine, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_ImgButtonEngine, LV_STATE_PRESSED);
    }else if  (doorvalues_current.DoorOpen == true){
        lv_obj_clear_state(ui_ImgButtonEngine, LV_STATE_DISABLED);
        lv_obj_add_state(ui_ImgButtonEngine, LV_STATE_PRESSED);
    }else{
        lv_obj_clear_state(ui_ImgButtonEngine, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_ImgButtonEngine, LV_STATE_PRESSED);
    }

       
}

void acedata_parse_temperature(String str){
       
    tempvalues_previous = tempvalues_current;
    
    //================================================== Left Temp =========================================*/
    acedata_current.leftTempSign = (TempSign)(str.charAt(ACEDATA_Temp1_Sign_POS));
    
    // Check for Error First
    if (acedata_current.leftTempSign == static_cast<int>(TempSign::ts_Error)){
        
        if (str.charAt(ACEDATA_Temp1_10X_POS) == static_cast<byte>(TempSign::ts_Opn)){
            acedata_current.leftTempSign = TempSign::ts_Opn;
        }
        else if (str.charAt(ACEDATA_Temp1_10X_POS) == static_cast<byte>(TempSign::ts_Sho)){
            acedata_current.leftTempSign = TempSign::ts_Sho;
        }

        tempvalues_current.leftTempError = true;
        tempvalues_current.leftTempSign = acedata_current.leftTempSign;

    }
    else 
    {
        // Parse Temperature Magnitude	
        acedata_current.leftTemp = static_cast<float>(ASCII2Num(str.charAt(ACEDATA_Temp1_1000X_POS)) * 1000 +
                                                        ASCII2Num(str.charAt(ACEDATA_Temp1_100X_POS)) * 100 +
                                                        ASCII2Num(str.charAt(ACEDATA_Temp1_10X_POS)) * 10 +
                                                        ASCII2Num(str.charAt(ACEDATA_Temp1_1X_POS)))  / 10;

        if(acedata_current.leftTempSign == (TempSign::ts_Neg)){
            acedata_current.leftTempSign = TempSign::ts_Neg;
            acedata_current.leftTemp = acedata_current.leftTemp * -1;

            tempvalues_current.leftTemp = acedata_current.leftTemp;
        }
        else{
            acedata_current.leftTempSign = TempSign::ts_Pos_IBox;
        } 

        tempvalues_current.leftTempError = false;

    }


    //================================================== Right Temp =========================================*/
    acedata_current.rightTempSign = (TempSign)(str.charAt(ACEDATA_Temp2_Sign_POS));
    
    // Check for Error First
    if (acedata_current.rightTempSign == static_cast<int>(TempSign::ts_Error)){
        
        if (str.charAt(ACEDATA_Temp2_10X_POS) == static_cast<byte>(TempSign::ts_Opn)){
            acedata_current.rightTempSign = TempSign::ts_Opn;
        }
        else if (str.charAt(ACEDATA_Temp2_10X_POS) == static_cast<byte>(TempSign::ts_Sho)){
            acedata_current.rightTempSign = TempSign::ts_Sho;
        }

        tempvalues_current.rightTempError = true;
        tempvalues_current.rightTempSign = acedata_current.rightTempSign;

    }
    else{
        // Parse Temperature Magnitude	
        acedata_current.rightTemp = static_cast<float>(ASCII2Num(str.charAt(ACEDATA_Temp2_1000X_POS)) * 1000 +
                                                        ASCII2Num(str.charAt(ACEDATA_Temp2_100X_POS)) * 100 +
                                                        ASCII2Num(str.charAt(ACEDATA_Temp2_10X_POS)) * 10 +
                                                        ASCII2Num(str.charAt(ACEDATA_Temp2_1X_POS)))  / 10;

        if(acedata_current.rightTempSign == (TempSign::ts_Neg)){
            acedata_current.rightTempSign = TempSign::ts_Neg;
            acedata_current.rightTemp = acedata_current.rightTemp * -1;

            tempvalues_current.rightTemp = acedata_current.rightTemp;
        }
        else{
            acedata_current.rightTempSign = TempSign::ts_Pos_IBox;
        } 

        tempvalues_current.rightTempError = false;

    }

    // If Both Sensors have not returned an error
    if (tempvalues_current.leftTempError == false && tempvalues_current.rightTempError == false){
        tempvalues_current.avgTemp = (tempvalues_current.leftTempError + tempvalues_current.rightTempError)/2;
    }
    else
    {
        // If a single sensor is failed, used the working sensor value as the average
        if (tempvalues_current.leftTempError == true && tempvalues_current.rightTempError == false)
        {
            tempvalues_current.avgTemp = tempvalues_current.leftTempError;
        }
        else if (tempvalues_current.leftTempError == false && tempvalues_current.rightTempError == true)
        {
            tempvalues_current.avgTemp = tempvalues_current.rightTempError;
        }
    }
}

void acedata_parse_battery(String str){
       
    battvalues_previous = battvalues_current;
    
    //================================================== Left Temp =========================================*/
    acedata_current.batteryVoltage = static_cast<float>(ASCII2Num(str.charAt(ACEDATA_Batt_100X_POS)) * 100 +
                                                        ASCII2Num(str.charAt(ACEDATA_Batt_10X_POS)) * 10 +
                                                        ASCII2Num(str.charAt(ACEDATA_Batt_1X_POS))  ) / 10;
    
    battvalues_current.voltage = acedata_current.batteryVoltage;

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

}

void acedata_parse_engine_stall(String str){

    enginevalues_previous = enginevalues_current;

    if (str.charAt(ACEDATA_Stall_Status_POS)=='0'){
        acedata_current.engineStalled = true;
    }else{
        acedata_current.engineStalled = false;
    }
    enginevalues_current.engineStalled = acedata_current.engineStalled;

    acedata_current.EngineStallCount = static_cast<float>(ASCII2Num(str.charAt(ACEDATA_Stall_Count_10X_POS)) * 10 +
                                                          ASCII2Num(str.charAt(ACEDATA_Stall_Count_1X_POS)));

    
    if (str.charAt(ACEDATA_Stall_Sensor_Present_POS) =='A'){
        acedata_current.engineStallSensorPresent = true;
    }else{
        acedata_current.engineStallSensorPresent = false;
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
        acedata_current.K9DoorOpen = true;
    }else{
        acedata_current.K9DoorOpen = false;
    }
    doorvalues_current.DoorOpen = acedata_current.K9DoorOpen;
    

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
    int i = trickle.indexOf("\n");
    while(i>-1) {
        String line = trickle.substring(0,i+1);
        trickle = trickle.substring(i+1);
        MONITOR.printf("Received: %s\n",line.c_str());
            String cmd = line;
            cmd.trim();
            int index=0;
            int length=10;
            if(cmd.substring(index,length)=="$ACEK9,IH2") {
                acedata_previous = acedata_current;
                acedata_parse_temperature(cmd);
                acedata_parse_battery(cmd);
                acedata_parse_engine_stall(cmd);
                acedata_parse_aux(cmd);
                acedata_parse_k9door(cmd);
                acedata_parse_serialnumber(cmd);

            }

            if(cmd.substring(index,length)=="$ACEK9,IP1"){

                MONITOR.printf("Sending TX String\n");
                
                // pinMode(ACEDATA_TX,OUTPUT);
                // for(int i=0;i<4;i++){
                //     digitalWrite(ACEDATA_TX,HIGH);
                //     delay(100);
                //     digitalWrite(ACEDATA_TX,LOW);
                //     delay(100);
                // }
                
                vim_write_sz("$ACEK9,CH1A,C502E5A63G-DEV0103B2C502E5A63GDADA00\r\n");
                

                //enable_ACEDATA_RX();
            } 

            i = trickle.indexOf("\n");
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
        if(aceconvalues_current.ppt) {
            lv_obj_add_state(ui_SwitchPPT, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchPPT, LV_STATE_CHECKED);
        }
    }
    if(aceconvalues_previous.pps != aceconvalues_current.pps) {
        MONITOR.printf("PPS Val Changed to  %s\r\n",aceconvalues_current.pps?"HIGH":"LOW");
        if(aceconvalues_current.pps) {
            lv_obj_add_state(ui_SwitchPPS, LV_STATE_CHECKED);
            doorvalues_current.DoorDisabled = false;
        } else {
            lv_obj_clear_state(ui_SwitchPPS, LV_STATE_CHECKED);
            doorvalues_current.DoorDisabled = true;
        }
    }
    if(aceconvalues_previous.ign != aceconvalues_current.ign) {
        MONITOR.printf("IGN Val Changed to  %s\r\n",aceconvalues_current.ign?"HIGH":"LOW");
        if(aceconvalues_current.ign) {
            lv_obj_add_state(ui_SwitchIGN, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchIGN, LV_STATE_CHECKED);
        }
        aceconvalues_current.ValueChanged = true;
    }
    if(aceconvalues_previous.hps != aceconvalues_current.hps) {
        
        if(aceconvalues_current.hps) {
            lv_obj_add_state(ui_SwitchHPS, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchHPS, LV_STATE_CHECKED);
        }
    }
    if(aceconvalues_previous.alm != aceconvalues_current.alm) {
        
        if(aceconvalues_current.alm) {
            lv_obj_add_state(ui_SwitchALM, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchALM, LV_STATE_CHECKED);
        }
    }


    
    //================================================== Read ACEDATA =========================================*/
    
    // Redraw Screen
    if(lv_scr_act() == ui_OperationScreen && ((acedata_current.ValueChanged) || (aceconvalues_current.ValueChanged))){


        acedata_current.ValueChanged = false;
        aceconvalues_current.ValueChanged = false;
        
        ui_update_acecon();

        lv_scr_load(lv_scr_act());
    } 

}

void setup() {
    
   Wire.begin( 1,42,100*1000); 
    lv_init();
    display_init();
    input_init();
    ui_init();
    vim_init(process_vim);
    
   //Wire.begin( 18,8,100*1000);
    //flood_it();
    lv_obj_add_event_cb(ui_SwitchPPT, ui_switch_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwitchPPS, ui_switch_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwitchALM, ui_switch_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwitchHPS, ui_switch_handler, LV_EVENT_ALL, NULL);
    dimmer.max_level(.0625);
    //================================================== XBEE Setup =========================================*/
#ifndef CUSTOM
    XBEE.begin(115200, SERIAL_8N1, 18, 17);
#else
    XBEE.begin(115200, SERIAL_8N1, 4, 5);
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
   
    //enable_ACEDATA_RX();

    //================================================== Set System Defaults =========================================*/
    systemsetting_default.AlarmPower = p_CarONCarOFF;
    systemsetting_default.AutoSnoozeEnabled = false;
    systemsetting_default.AuxInputEnabled = false;
    systemsetting_default.BatteryVoltage = b_12;
    systemsetting_default.DoorPower = p_CarONCarOFF;
    systemsetting_default.StallMonitorEnabled = false;
    systemsetting_default.TempAveragingEnabled = true;

    acedata_current.ValueChanged = false;

    set_HPS(HIGH);
    set_PPS(HIGH);

    MONITOR.printf("Exiting Setup()\n");

}



void loop() {

    display_update();
    lv_timer_handler();
    return;
    monitor_dev_tick(MONITOR);

    xbee_dev_tick(&my_xbee);
    if(((int)last.status)<0) {
        on_xbee_error(last.cmd, last.status);
    }

    acecon_dev_tick();

}

