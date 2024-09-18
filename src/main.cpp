#define XBEE Serial1
#define ACECONSerial Serial2
#define MONITOR Serial
#define DUMP_PACKETS
#include <Arduino.h>
#include <Wire.h>
#include <functional>
#include <lvgl.h>
#include <Squareline/ui.h>
#include <interface.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>  // Include to use variable argument functions
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
#include "init_config_packet.h"
#include "init_data_packet.h"
#include "init_status_packet.h"

//================================================== Temperature Variables =========================================
#define TempErrorGeneral    'E'
#define TempErrorOpen       '5'
#define TempErrorSho        '4'
#define TempSignPos         ' '
#define TempSignNeg         '-'


//================================================== Battery Variables =========================================*/
#define BadBatteryThreshold 9
#define MaxBadBattValCounter 5


//================================================== K9 Door Variables =========================================*/


bool UpdateNoK9Flag = false;
#define NoK9BlinkTime_ms 250
bool NoK9TimeoutFlag = false;
bool NoK9BlinkToggle = false;

//================================================== GLOBAL VARIABLES =========================================*/
 
struct last_packet_info {
    STATUS_CODE status; // = STATUS_CODE::SUCCESS;
    COMMAND_ID cmd; // = (COMMAND_ID)0;
	bool processed;
};
static last_packet_info last_packet = {STATUS_CODE::SUCCESS,(COMMAND_ID)0,true};

struct last_at_info {
    char commandstr[2];
    uint32_t value;
    bool value_received;
};

static last_at_info last_at_cmd;

static config_packet config_data;

static command_packet command_data;

static update_packet update_data;

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


#define Comm_Error_Timeout	60000// 8000 TODO: Revert this code
int COM_Retry_Attempts = 0;
#define COM_Retry_Threshold 3



enum PowerOpt {p_CarONCarOFF,p_CarONManOFF,p_ManONManOFF,p_NoK9Left,p_OFF};
enum DoorOpt {d_CarONCarOFF,d_CarONManOFF,d_ManONManOFF,d_OFF};
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
	bool ColdAlarmEnabled;
    bool doorDisabled;
    int AlarmHotSetIndex;
    int AlarmColdSetIndex;
    bool AlarmUnitF;
    PowerOpt AlarmPower;
    DoorOpt DoorPower;
    Batt BatteryVoltage;
    bool System_Sleep; 			// State Flag to determine if the system was been put to sleep 
	bool InitialPowerUpFlag; 	// State flag to determine if the system was just powered on, clears when an idle screen is entered

};

SystemSettings systemsettings_current;
SystemSettings systemsettings_previous;
SystemSettings systemsettings_default;

#define EngineStallThreshold  1

bool Previous_HotnPopFlag = false;
bool Current_HotnPopFlag = true; // Hot n Pop state flag
bool Current_HotnPopFlag_DEFAULT = false;
#define HotnPopPos 47 

#define HotnPopEnabled 53
#define HotnPopDisabled 50

// System Inputs
enum DoorPopCondition {dc_Popped = 0,dc_Unpopped = 1};
DoorPopCondition CurrentDoorPopCondition = DoorPopCondition::dc_Unpopped;

enum DoorPopTrigger {dt_None,dt_Salute,dt_Remote};
DoorPopTrigger Current_DoorPopTrigger = DoorPopTrigger::dt_None;

bool UpdateDoorFlag = false;

#define INGEAR	0
#define PARKED	1

bool GoToMenuFlag = false;


vim_data data_global;

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
	static bool last_received = false;

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

	// Preamble
	#define CCHTX_PreAmble "$ACEK9,CH1A,"

	enum CCHTX_ACC {TX_ACC_Off = 48, TX_ACC_On = 49};
	CCHTX_ACC ACC1State = TX_ACC_Off;
	CCHTX_ACC ACC2State = TX_ACC_Off;

	enum CCHTX_AutoStart { TX_AutoStart_Off = 48, TX_AutoStart_On = 49 };
	CCHTX_AutoStart AutoStart1State = TX_AutoStart_Off;

	#define CCHTX_IGNStatus_On	"D"
	#define CCHTX_IGNStatus_Off	"E"

	#define CCHTX_Toot_On	"5"
	#define CCHTX_Toot_Off	"A"

	#define LineNum_CH	2

	// Outputs
	bool TX_Toot_Needed = false;
	bool HeatAlarmEnabled;
	bool DoorPopEnabled;

	int TootCount = 0;


	bool SettingsLoaded = false;


//================================================== FOTA Stuff =========================================


//#define FOTA_MANUAL_CONTROL

int packetnum = 0;

enum FOTACode
{
	FOTA_CheckFW,
	FOTA_DeleteFW,
	FOTA_Initiate,
	FOTA_Done,
	FOTA_Downloading = 68,
	FOTA_Waiting = 87,
	FOTA_Begin = 80,
	FOTA_Success = 83,
	FOTA_Rollback = 82,
	FOTA_Fail = 70,
	FOTA_None = 78
};
FOTACode CurrentFOTACode = FOTA_None;
FOTACode PreviousFOTACode = CurrentFOTACode;
#define FOTA_Packet_Length 1024
bool FOTA_Allowed = false;

//================================================== State Machine Stuff =========================================

enum PowerOnTrigger {NoTrigger,Applied,PowerButtonPress,IgnitionOn};

PowerOnTrigger CurrentPowerOnTrigger;

enum StateID {ID_A0_Off,ID_A1_PowerApplied, ID_A3_IgnitionOn,
			  ID_B1_MenuHelp,
              ID_C1_HA_DP,ID_C2_HA_ONLY,ID_C3_DP_ONLY,
              ID_D1_NOK9LeftBehind,ID_D2_PressOKToConfirm,ID_D6_NoK9LeftBehindPowerDownByDoorOpened,ID_D7_PowerDownByOKPress,ID_D8_PowerDownByIgnitionOFF, ID_D9_PowerDownByHAandDPSetToAlwaysOFF, ID_D10_PowerDownByPowerPress, ID_D11_UpdatingFirmware,
              ID_E1_VIMCommunicationsError, ID_E3_UNKNOWN,
              ID_S1_Sleep, 
              ID_G1_SystemTestConfirm, ID_G2_SystemTest};

struct SystemState{
    StateID Index;
    String Name;
    //PowerOnTrigger PowerTrigger;

};

SystemState systemstate_current;
SystemState systemstate_previous;
SystemState StoredNextState; // Used for manually directing another state where to go next. Used very sparingly

SystemState A0_Off_State;
SystemState A1_PowerApplied_State;
SystemState A3_IgnitionOn_State;
SystemState B1_MenuHelp_State;
SystemState C1_HA_DP_State;
SystemState C2_HA_ONLY_State;
SystemState C3_DP_ONLY_State;
SystemState D1_NOK9LeftBehind_State;
SystemState D2_PressOKToConfirm_State;
SystemState D6_NoK9LeftBehindPowerDownByDoorOpened_State;
SystemState D7_PowerDownByOKPress_State;
SystemState D8_PowerDownByIgnitionOFF_State;
SystemState D9_PowerDownByHAandDPSetToAlwaysOFF_State;
SystemState D10_PowerDownByPowerPress_State;
SystemState D11_UpdatingFirmware_State;
SystemState E1_VIMCommunicationsError_State;
SystemState E3_Unknown_State;
SystemState S1_Sleep_State;
SystemState G1_SystemTestConfirm_State;
SystemState G2_SystemTest_State;



//================================================== Timer Stuff =========================================


enum TimerID {Timer_A0_Off,Timer_A1_PowerApplied, Timer_A3_IgnitionOn,
			  Timer_B1_MenuHelp,
              Timer_C1_HA_DP,Timer_C2_HA_ONLY,Timer_C3_DP_ONLY,
              Timer_D1_NOK9LeftBehind,Timer_D2_PressOKToConfirm,Timer_D6_NoK9LeftBehindPowerDownByDoorOpened,Timer_D7_PowerDownByOKPress,Timer_D8_PowerDownByIgnitionOFF, Timer_D9_PowerDownByHAandDPSetToAlwaysOFF, Timer_D10_PowerDownByPowerPress, Timer_D11_UpdatingFirmware,
              Timer_E1_VIMCommunicationsError, Timer_E3_UNKNOWN,
              Timer_S1_Sleep, 
              Timer_G1_SystemTestConfirm, Timer_G2_SystemTest,
			  Timer_Read_IBoxPopped,Timer_Clear_IBoxPopped,Timer_ReadGear,Timer_ClearGear,Timer_SystemAlarm,TimerPreAlarmNotification, Timer_SnoozeAlarm,
			  Timer_NoK9Blink, Timer_NoK9Beep, Timer_NoK9FirstAlert,Timer_NoK9SecondAlert, Timer_COMReset, Timer_COMError, Timer_InitialPowerUp, Timer_VIMAlarm};

struct Timer{
    TimerID Index;
    String Name;
    unsigned long TimeVal;
	unsigned long Threshold;
	bool OverFlowFlag;
	bool TimerEnable;
	
	void SetTimerVal(long timeval){
		TimeVal = timeval;
	}
	void SyncTimerVal(){
		TimeVal = millis();
	}
	void SetThreshold(unsigned long thresholdval){
		Threshold = thresholdval;
	}
	unsigned long ReadTimerVal(){
		return TimeVal;
	}
	bool CheckOverflow(){
		if (TimerEnable == true) {

			if (millis() - ReadTimerVal() >= Threshold)
			{
				OverFlowFlag = true;
				return true;
			}
		
		}
		return false;
	}
	void StartTimer(unsigned long thresholdval){

		// Set Threshold
		SetThreshold(thresholdval);

		// Clear OverFlow Flag
		OverFlowFlag = false;

		// Enable Timer
		TimerEnable = true;

		// Sync Timer Val
		SyncTimerVal();
		}
	void StopTimer(){

			// Clear OverFlow Flag
			OverFlowFlag = false;

			// Disable Timer
			TimerEnable = false;
		}

};

Timer A0_Off_Timer;
Timer A1_PowerApplied_Timer;
Timer A3_IgnitionOn_Timer;
Timer B1_MenuHelp_Timer;
Timer C1_HA_DP_Timer;
Timer C2_HA_Only_Timer;
Timer C3_DP_Only_Timer;
Timer D1_NoK9LeftBehind_Timer;
Timer D2_PressOKToConfirm_Timer;
Timer D6_NoK9LeftBehindPowerDownByDoorOpened_Timer;
Timer D7_PowerDownByOKPress_Timer;
Timer D8_PowerDownByIgnitionOFF_Timer;
Timer D9_PowerDownByHAandDPSetToAlwaysOFF_Timer;
Timer D10_PowerDownByPowerPress_Timer;
Timer D11_UpdatingFirmware_Timer;
Timer E1_VIMCommunicationsError_Timer;
Timer E3_Unknown_Timer;
Timer S1_Sleep_Timer;
Timer G1_SystemTestConfirm_Timer;
Timer G2_SystemTest_Timer;
Timer Read_IBoxPopped_Timer;
Timer Clear_IBoxPopped_Timer;
Timer Read_Gear_Timer;
Timer Clear_Gear_Timer;
Timer SystemAlarm_Timer;
Timer PreAlarmNotification_Timer;
Timer SnoozeAlarm_Timer;
Timer NoK9Blink_Timer;
Timer NoK9Beep_Timer;
Timer NoK9FirstAlert_Timer;
Timer NoK9SecondAlert_Timer;
Timer COMReset_Timer;
Timer COMError_Timer;
Timer InitialPowerUp_Timer;
Timer VIMAlarm_Timer;
Timer MainLoop_Timer;

//================================================== Alarm Stuff =========================================

//#define ALARM_MANUAL_CONTROL

enum AlarmState {a_None,a_PreAlarm,a_FullAlarm,a_Snooze,a_Init};
AlarmState Previous_System_Alarm_State = AlarmState::a_None;
AlarmState Current_System_Alarm_State = AlarmState::a_None;
bool AlarmStateEnabled = false;

#define AlarmTextMaxChars 50
String PreAlertText = "";
String FullAlarmText = "";


#define SingleTempOverrideAmount	1
#define TempWarningOffset	5

bool UpdatePreAlarmFlag = false;
long PreAlarmTriggerTime;
int CurrentPreAlarmCounter = 0;
int PreviousPreAlarmCounter = 0;
int MaxPreAlarmTime = 61; // TODO: REVERT THIS VALUE BACK TO NORMAL 61; // add 1 to account for decimal truncation
#define SystemAlarmTimerVal 61000 // TODO: REVERT THIS VALUE BACK TO NORMAL 61000; // Manual Conversion of MaxPreAlarmTime to ms, Shouldn't have to do this but it was the only way to keep the variable from overflowing
#define DisplayPreAlarmCounter_Default 60 // TODO: REVERT THIS VALUE BACK TO NORMAL 60
int DisplayPreAlarmCounter = DisplayPreAlarmCounter_Default;

bool UpdateSnoozeAlarmFlag = false;
long SnoozeAlarmTriggerTime;
int CurrentSnoozeAlarmCounter = 0;
int PreviousSnoozeAlarmCounter = 0;
int MaxSnoozeAlarmTime = 61; // TODO: REVERT THIS VALUE BACK TO NORMAL 480; // 8mins = 480s 
#define SnoozeAlarmTimerVal 61000 // TODO: REVERT THIS VALUE BACK TO NORMAL 480000; // Manual Conversion of MaxPreAlarmTime to ms, Shouldn't have to do this but it was the only way to keep the variable from overflowing
#define DisplaySnoozeAlarmCounter_Default 60 // TODO: REVERT THIS VALUE BACK TO NORMAL 480
long DisplaySnoozeAlarmCounter = DisplaySnoozeAlarmCounter_Default;

bool UpdateFullAlarmFlag = false;
long FullAlarmTriggerTime;
long CurrentFullAlarmCounter = 0;
long PreviousFullAlarmCounter = 0;

bool System_Test_Alarm_Active = false;

#define COLOR_RED		0xC2000D
#define COLOR_YELLOW	0xFFFF00
#define COLOR_BLUE		0x00FFFF
#define COLOR_GREEN		0x90EE90

void Init_Timers()
{
	// Initialize Timers
	
	// Read_IBoxPopped_Timer State Initialization
	Read_IBoxPopped_Timer.Name = "Read_IBoxPopped_Timer";
	Read_IBoxPopped_Timer.OverFlowFlag = false;
	Read_IBoxPopped_Timer.Threshold = 60;
	Read_IBoxPopped_Timer.TimerEnable = true;

	// Clear_IBoxPopped_Timer State Initialization
	Clear_IBoxPopped_Timer.Name = "Clear_IBoxPopped_Timer";
	Clear_IBoxPopped_Timer.OverFlowFlag = false;
	Clear_IBoxPopped_Timer.Threshold = 6000;
	Clear_IBoxPopped_Timer.TimerEnable = false;// Enabled later if the system detects a Popped Condition

	// Read_Gear_Timer State Initialization
	Read_Gear_Timer.Name = "Read_Gear_Timer";
	Read_Gear_Timer.OverFlowFlag = false;
	Read_Gear_Timer.Threshold = 60;
	Read_Gear_Timer.TimerEnable = true;

	// Clear_Gear_Timer State Initialization
	Clear_Gear_Timer.Name = "Clear_Gear_Timer";
	Clear_Gear_Timer.OverFlowFlag = false;
	Clear_Gear_Timer.Threshold = 3500; // 3.5s 3500ms
	Clear_Gear_Timer.TimerEnable = false; // Enabled later if the system detects an In Gear Condition

	// SystemAlarm_Timer State Initialization
	SystemAlarm_Timer.Name = "SystemAlarm_Timer";
	SystemAlarm_Timer.OverFlowFlag = false;
	SystemAlarm_Timer.Threshold = SystemAlarmTimerVal;
	SystemAlarm_Timer.TimerEnable = false; // Enabled later if the System count overflows

	// PreAlarmNotification_Timer State Initialization
	PreAlarmNotification_Timer.Name = "PreAlarmNotification_Timer";
	PreAlarmNotification_Timer.OverFlowFlag = false;
	PreAlarmNotification_Timer.Threshold = 1000;
	PreAlarmNotification_Timer.TimerEnable = false;

	// SnoozeAlarm_Timer State Initialization
	SnoozeAlarm_Timer.Name = "SnoozeAlarm_Timer";
	SnoozeAlarm_Timer.OverFlowFlag = false;
	SnoozeAlarm_Timer.Threshold = SnoozeAlarmTimerVal;
	SnoozeAlarm_Timer.TimerEnable = false;

	// NoK9BlinkTimer State Initialization
	NoK9Blink_Timer.Name = "NoK9Blink_Timer";
	NoK9Blink_Timer.OverFlowFlag = false;
	NoK9Blink_Timer.Threshold = 1000;
	NoK9Blink_Timer.TimerEnable = false; // Enabled later if the system goes to No K9 Screen

	// NoK9BeepTimer State Initialization
	NoK9Beep_Timer.Name = "NoK9Beep_Timer";
	NoK9Beep_Timer.OverFlowFlag = false;
	NoK9Beep_Timer.Threshold = 8000;
	NoK9Beep_Timer.TimerEnable = false; // Enabled later if the system goes to No K9 Screen

	// NoK9AlertTimer State Initialization
	NoK9FirstAlert_Timer.Name = "NoK9FirstAlert_Timer";
	NoK9FirstAlert_Timer.OverFlowFlag = false;
	NoK9FirstAlert_Timer.Threshold = 10000; //180000; // 3 mins == 180s == 180000ms // TODO: REVERT THIS CONSTANT VALUE
	NoK9FirstAlert_Timer.TimerEnable = false; // Enabled later if the system goes to No K9 Screen

	// NoK9AlertTimer State Initialization
	NoK9SecondAlert_Timer.Name = "NoK9SecondAlert_Timer";
	NoK9SecondAlert_Timer.OverFlowFlag = false;
	NoK9SecondAlert_Timer.Threshold = 20000; //360000; // 6 mins == 360s == 360000ms // TODO: REVERT THIS CONSTANT VALUE
	NoK9SecondAlert_Timer.TimerEnable = false; // Enabled later if the system goes to No K9 Screen

	// COM Error Timer Initialization
	COMError_Timer.Name = "COMError_Timer";
	COMError_Timer.OverFlowFlag = false;
	COMError_Timer.Threshold = 2000;
	COMError_Timer.TimerEnable = false;

	// COM Reset Timer Initialization
	COMReset_Timer.Name = "COMReset_Timer";
	COMReset_Timer.OverFlowFlag = false;
	COMReset_Timer.Threshold = 2000;
	COMReset_Timer.TimerEnable = false;

	// InitialPowerUp_Timer Initialization
	InitialPowerUp_Timer.Name = "InitialPowerUp_Timer";
	InitialPowerUp_Timer.OverFlowFlag = false;
	InitialPowerUp_Timer.Threshold = 1000;
	InitialPowerUp_Timer.TimerEnable = false;
	
	// VIMAlarm_Timer
	VIMAlarm_Timer.Name = "VIMAlarm_Timer";
	VIMAlarm_Timer.OverFlowFlag = false;
	VIMAlarm_Timer.Threshold = 300000; // 5min = 300 s = 300,000 ms
	VIMAlarm_Timer.TimerEnable = false;
	
	// MainLoop_Timer
	MainLoop_Timer.Name = "MainLoop_Timer";
	MainLoop_Timer.OverFlowFlag = false;
	MainLoop_Timer.Threshold = 1500; // 1s = 1,000 ms
	MainLoop_Timer.TimerEnable = true;

}

void Update_Timers()
{
	// System State Timers
	A0_Off_Timer.CheckOverflow();
	A1_PowerApplied_Timer.CheckOverflow();
	A3_IgnitionOn_Timer.CheckOverflow();
	B1_MenuHelp_Timer.CheckOverflow();
	C1_HA_DP_Timer.CheckOverflow();
	C2_HA_Only_Timer.CheckOverflow();
	C3_DP_Only_Timer.CheckOverflow();
	D1_NoK9LeftBehind_Timer.CheckOverflow();
	D2_PressOKToConfirm_Timer.CheckOverflow();
	D6_NoK9LeftBehindPowerDownByDoorOpened_Timer.CheckOverflow();
	D7_PowerDownByOKPress_Timer.CheckOverflow();
	D8_PowerDownByIgnitionOFF_Timer.CheckOverflow();
	D9_PowerDownByHAandDPSetToAlwaysOFF_Timer.CheckOverflow();
	D10_PowerDownByPowerPress_Timer.CheckOverflow();
	D11_UpdatingFirmware_Timer.CheckOverflow();
	E1_VIMCommunicationsError_Timer.CheckOverflow();
	S1_Sleep_Timer.CheckOverflow();
	G1_SystemTestConfirm_Timer.CheckOverflow();
	G2_SystemTest_Timer.CheckOverflow();

	// General Timers
	Read_IBoxPopped_Timer.CheckOverflow();
	Clear_IBoxPopped_Timer.CheckOverflow();
	Read_Gear_Timer.CheckOverflow();
	Clear_Gear_Timer.CheckOverflow();
	SystemAlarm_Timer.CheckOverflow();
	PreAlarmNotification_Timer.CheckOverflow();
	SnoozeAlarm_Timer.CheckOverflow();
	NoK9Blink_Timer.CheckOverflow();
	NoK9Beep_Timer.CheckOverflow();
	NoK9FirstAlert_Timer.CheckOverflow();
	NoK9SecondAlert_Timer.CheckOverflow();
	COMError_Timer.CheckOverflow();
	COMReset_Timer.CheckOverflow();
	InitialPowerUp_Timer.CheckOverflow();
	VIMAlarm_Timer.CheckOverflow();
	MainLoop_Timer.CheckOverflow();
}

void Init_SystemStates()
{

	// Initialize System States
	A0_Off_State.Name = "A0 Off State";
    A0_Off_State.Index = StateID::ID_A0_Off;
	// A0_Off_State.OverFlowFlag = false;
	// A0_Off_State.SyncTimerVal();
	// A0_Off_State.Threshold = 20;
	// A0_Off_State.TimerEnable = false;

	A1_PowerApplied_State.Name = "A1 Power Applied State";
	A1_PowerApplied_State.Index = StateID::ID_A1_PowerApplied;
	// A1_PowerApplied_State.OverFlowFlag = false;
	// A1_PowerApplied_State.SyncTimerVal();
	// A1_PowerApplied_State.Threshold = 0;
	// A1_PowerApplied_State.TimerEnable = false;

	A3_IgnitionOn_State.Name = "A3 Ignition On State";
	A3_IgnitionOn_State.Index = StateID::ID_A3_IgnitionOn;
	// A3_IgnitionOn_State.OverFlowFlag = false;
	// A3_IgnitionOn_State.SyncTimerVal();
	// A3_IgnitionOn_State.Threshold = 0;
	// A3_IgnitionOn_State.TimerEnable = false;

	B1_MenuHelp_State.Name = "B1 Menu Help State";
	B1_MenuHelp_State.Index = StateID::ID_B1_MenuHelp;
	// B1_MenuHelp_State.OverFlowFlag = false;
	// B1_MenuHelp_State.SyncTimerVal();
	// B1_MenuHelp_State.Threshold = 0;
	// B1_MenuHelp_State.TimerEnable = false;

	C1_HA_DP_State.Name = "C1 HA_DP State";
	C1_HA_DP_State.Index = StateID::ID_C1_HA_DP;
	// C1_HA_DP_State.OverFlowFlag = false;
	// C1_HA_DP_State.SyncTimerVal();
	// C1_HA_DP_State.Threshold = 0;
	// C1_HA_DP_State.TimerEnable = false;

	C2_HA_ONLY_State.Name = "C2 HA Only State";
	C2_HA_ONLY_State.Index = StateID::ID_C2_HA_ONLY;
	// C2_HAOnly_State.OverFlowFlag = false;
	// C2_HAOnly_State.SyncTimerVal();
	// C2_HAOnly_State.Threshold = 0;
	// C2_HAOnly_State.TimerEnable = false;

	C3_DP_ONLY_State.Name = "C3 DP Only State";
	C3_DP_ONLY_State.Index = StateID::ID_C3_DP_ONLY;
	// C3_DP_ONLY_State.OverFlowFlag = false;
	// C3_DP_ONLY_State.SyncTimerVal();
	// C3_DP_ONLY_State.Threshold = 0;
	// C3_DP_ONLY_State.TimerEnable = false;

	D1_NOK9LeftBehind_State.Name = "D1 No K9 Left Behind State";
	D1_NOK9LeftBehind_State.Index = StateID::ID_D1_NOK9LeftBehind;
	// D1_NoK9LeftBehind_State.OverFlowFlag = false;
	// D1_NoK9LeftBehind_State.SyncTimerVal();
	// D1_NoK9LeftBehind_State.Threshold = 0;
	// D1_NoK9LeftBehind_State.TimerEnable = false;

	D2_PressOKToConfirm_State.Name = "D2 Press OK To Confirm State";
	D2_PressOKToConfirm_State.Index = StateID::ID_D2_PressOKToConfirm;
	// D2_PressOKToConfirm_State.OverFlowFlag = false;
	// D2_PressOKToConfirm_State.SyncTimerVal();
	// D2_PressOKToConfirm_State.Threshold = 0;
	// D2_PressOKToConfirm_State.TimerEnable = false;

	D6_NoK9LeftBehindPowerDownByDoorOpened_State.Name = "No K9 Left Behind Power Down By Door Opened";
	D6_NoK9LeftBehindPowerDownByDoorOpened_State.Index = StateID::ID_D6_NoK9LeftBehindPowerDownByDoorOpened;
	// D6_NoK9LeftBehindPowerDownByDoorOpened_State.OverFlowFlag = false;
	// D6_NoK9LeftBehindPowerDownByDoorOpened_State.SyncTimerVal();
	// D6_NoK9LeftBehindPowerDownByDoorOpened_State.Threshold = 0;
	// D6_NoK9LeftBehindPowerDownByDoorOpened_State.TimerEnable = false;

	D7_PowerDownByOKPress_State.Name = "D7 Power Down By OK Press State";
	D7_PowerDownByOKPress_State.Index = StateID::ID_D7_PowerDownByOKPress;
	// D7_PowerDownByOKPress_State.OverFlowFlag = false;
	// D7_PowerDownByOKPress_State.SyncTimerVal();
	// D7_PowerDownByOKPress_State.Threshold = 0;
	// D7_PowerDownByOKPress_State.TimerEnable = false;

	D8_PowerDownByIgnitionOFF_State.Name = "D8 Power Down By Ignition OFF State";
	D8_PowerDownByIgnitionOFF_State.Index = StateID::ID_D8_PowerDownByIgnitionOFF;
	// D8_PowerDownByIgnitionOFF_State.OverFlowFlag = false;
	// D8_PowerDownByIgnitionOFF_State.SyncTimerVal();
	// D8_PowerDownByIgnitionOFF_State.Threshold = 0;
	// D8_PowerDownByIgnitionOFF_State.TimerEnable = false;

	D9_PowerDownByHAandDPSetToAlwaysOFF_State.Name = "D9 Power Down By HA and DP Set To Always OFF State";
	D9_PowerDownByHAandDPSetToAlwaysOFF_State.Index = StateID::ID_D9_PowerDownByHAandDPSetToAlwaysOFF;
	// D9_PowerDownByHAandDPSetToAlwaysOFF_State.OverFlowFlag = false;
	// D9_PowerDownByHAandDPSetToAlwaysOFF_State.SyncTimerVal();
	// D9_PowerDownByHAandDPSetToAlwaysOFF_State.Threshold = 0;
	// D9_PowerDownByHAandDPSetToAlwaysOFF_State.TimerEnable = false;

	D10_PowerDownByPowerPress_State.Name = "D10 Power Down By Power Press State";
	D10_PowerDownByPowerPress_State.Index = StateID::ID_D10_PowerDownByPowerPress;
	// D10_PowerDownByPowerPress_State.OverFlowFlag = false;
	// D10_PowerDownByPowerPress_State.SyncTimerVal();
	// D10_PowerDownByPowerPress_State.Threshold = 0;
	// D10_PowerDownByPowerPress_State.TimerEnable = false;

	D11_UpdatingFirmware_State.Name = "D11 Updating Firmware State";
	D11_UpdatingFirmware_State.Index = StateID::ID_D11_UpdatingFirmware;
	// D11_UpdatingFirmware_State.OverFlowFlag = false;
	// D11_UpdatingFirmware_State.SyncTimerVal();
	// D11_UpdatingFirmware_State.Threshold = 300000;// 5min = 300s = 300000ms
	// D11_UpdatingFirmware_State.TimerEnable = false;

	E1_VIMCommunicationsError_State.Name = "E1 VIM Communications Error State";
	E1_VIMCommunicationsError_State.Index = StateID::ID_E1_VIMCommunicationsError;
	// E1_VIMCommunicationsError_State.OverFlowFlag = false;
	// E1_VIMCommunicationsError_State.SyncTimerVal();
	// E1_VIMCommunicationsError_State.Threshold = 0;
	// E1_VIMCommunicationsError_State.TimerEnable = false;

	E3_Unknown_State.Name = "E3 Unknown State";
	E3_Unknown_State.Index = StateID::ID_E3_UNKNOWN;
	// E3_Unhandled_State.OverFlowFlag = false;
	// E3_Unhandled_State.SyncTimerVal();
	// E3_Unhandled_State.Threshold = 0;
	// E3_Unhandled_State.TimerEnable = false;

	S1_Sleep_State.Name = "S1 Sleep State";
	S1_Sleep_State.Index = StateID::ID_S1_Sleep;
	// S1_Sleep_State.OverFlowFlag = false;
	// S1_Sleep_State.SyncTimerVal();
	// S1_Sleep_State.Threshold = 0;
	// S1_Sleep_State.TimerEnable = false;

	G1_SystemTestConfirm_State.Name = "G1 System Test Confirm State";
	G1_SystemTestConfirm_State.Index = StateID::ID_G1_SystemTestConfirm;
	// G1_SystemTestConfirm_State.OverFlowFlag = false;
	// G1_SystemTestConfirm_State.SyncTimerVal();
	// G1_SystemTestConfirm_State.Threshold = 4000;
	// G1_SystemTestConfirm_State.TimerEnable = false;

	G2_SystemTest_State.Name = "G2 System Test State";
	G2_SystemTest_State.Index = StateID::ID_G2_SystemTest;
	// G2_SystemTest_State.OverFlowFlag = false;
	// G2_SystemTest_State.SyncTimerVal();
	// G2_SystemTest_State.Threshold = 5000;
	// G2_SystemTest_State.TimerEnable = false;

	// ReSet Current and Prev States
	systemstate_previous = E3_Unknown_State;
	systemstate_current = A0_Off_State;

}

void Set_Next_State(SystemState nextstate)
{

	// FOTA functionality needs to check if the system is trying to power down. If it is and FOTA operations are pending, the systems needs
	// to temporarily go to the FOTA state before continuing on to the previously requested power down state.
	if ((CurrentFOTACode == FOTACode::FOTA_Downloading || CurrentFOTACode == FOTACode::FOTA_Waiting) && 
		(nextstate.Index == D6_NoK9LeftBehindPowerDownByDoorOpened_State.Index ||
			nextstate.Index == D7_PowerDownByOKPress_State.Index ||
			nextstate.Index == D8_PowerDownByIgnitionOFF_State.Index ||
			nextstate.Index == D9_PowerDownByHAandDPSetToAlwaysOFF_State.Index ||
			nextstate.Index == D10_PowerDownByPowerPress_State.Index)
		
		)
	{

		StoredNextState = nextstate;
		
        MONITOR.println("Next System State: " + D11_UpdatingFirmware_State.Name);
		MONITOR.println("Stored Next System State: " + StoredNextState.Name);

        
		systemstate_previous = systemstate_current;
		systemstate_current = D11_UpdatingFirmware_State;


	}
	else
	{
		MONITOR.println("Next System State: " + nextstate.Name);
		systemstate_previous = systemstate_current;
		systemstate_current = nextstate;
	}

}

SystemState Determine_Next_State()
{
	MONITOR.println("Determine Next State");
	MONITOR.printf("IngitionOn:  %s\r\n",data_global.ignitionOn_current?"TRUE":"FALSE");
	switch (systemsettings_current.DoorPower)
	{
	case DoorOpt::d_CarONCarOFF:
		MONITOR.println("Door Power: CarOnCarOFF");
		break;
	case DoorOpt::d_CarONManOFF :
		MONITOR.println("Door Power: CarOnManOFF");
		break;
	case DoorOpt::d_ManONManOFF:
		MONITOR.println("Door Power: ManOnManOFF");
		break;
	case DoorOpt::d_OFF:
		MONITOR.println("Door Power: OFF");
		break;
	
	default:
		MONITOR.println("Door Power: Unknown");
		break;
	}			
	switch (systemsettings_current.AlarmPower)
	{
	case PowerOpt::p_CarONCarOFF:
		MONITOR.println("Alarm Power: CarOnCarOFF");
		break;
	case PowerOpt::p_CarONManOFF:
		MONITOR.println("Alarm Power: CarOnManOFF");
		break;
	case PowerOpt::p_ManONManOFF:
		MONITOR.println("Alarm Power: ManOnManOFF");
		break;
	case PowerOpt::p_OFF:
		MONITOR.println("Alarm Power: OFF");
		break;
	case PowerOpt::p_NoK9Left:
		MONITOR.println("Alarm Power: No K9");
		break;
	
	default:
		MONITOR.println("Alarm Power: Unknown");
		break;
	}		



	switch (CurrentPowerOnTrigger)
	{
	case PowerOnTrigger::NoTrigger:
		MONITOR.println("PowerOnTrigger: No Trigger");
		break;
	case PowerOnTrigger::IgnitionOn:
		MONITOR.println("PowerOnTrigger: Ignition On");
		break;
	case PowerOnTrigger::Applied:
		MONITOR.println("PowerOnTrigger: Applied");
		break;
	case PowerOnTrigger::PowerButtonPress:
		MONITOR.println("PowerOnTrigger: Power Button Press");
		break;
	
	default:
		MONITOR.println("PowerOnTrigger: Unknown");
		break;
	}			
	

	if (CurrentPowerOnTrigger == PowerOnTrigger::Applied ||
		CurrentPowerOnTrigger == PowerOnTrigger::PowerButtonPress)
	{
		MONITOR.println("Power Trigger: Applied");
		// Ignition OFF
		if (data_global.ignitionOn_current == false)
		{
			MONITOR.println("ignitionOn: false");
			// Door Popper Settings
			if (systemsettings_current.DoorPower == DoorOpt::d_OFF)
			{
				// Alarm Settings
				if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
				{
					return (D9_PowerDownByHAandDPSetToAlwaysOFF_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
				{
					return (D8_PowerDownByIgnitionOFF_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					return (D1_NOK9LeftBehind_State);
				}
				// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
				// {
				// 	return (StateID::C2_HA_ONLY);
				// }
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
				{
					return (C2_HA_ONLY_State);
				}
			}
			else if (systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF)
			{
				// Alarm Settings
				if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
				{
					return (D8_PowerDownByIgnitionOFF_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
				{
					return (D8_PowerDownByIgnitionOFF_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					return (D1_NOK9LeftBehind_State);
				}
				// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
				// {
				// 	return (StateID::C2_HA_ONLY);
				// }
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
				{
					return (C2_HA_ONLY_State);
				}
			}
			else if (systemsettings_current.DoorPower == DoorOpt::d_CarONManOFF)
			{
				// Alarm Settings
				if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					return (D1_NOK9LeftBehind_State);
				}
				// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
				// {
				// 	return (StateID::C1_HA_DP);
				// }
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
				{
					return (C1_HA_DP_State);
				}
			}
			else if (systemsettings_current.DoorPower == DoorOpt::d_ManONManOFF)
			{
				// Alarm Settings
				if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					return (C3_DP_ONLY_State);
				}
				// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
				// {
				// 	return (StateID::C1_HA_DP);
				// }
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
				{
					return (C1_HA_DP_State);
				}
			}

		}

		// Ignition ON
		else if (data_global.ignitionOn_current == true)
		{
			// Door Popper Settings
			if (systemsettings_current.DoorPower == DoorOpt::d_OFF)
			{
				// Alarm Settings
				if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
				{
					return (D9_PowerDownByHAandDPSetToAlwaysOFF_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
				{
					return (C2_HA_ONLY_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					return (C2_HA_ONLY_State);
				}
				// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
				// {
				// 	return (StateID::C2_HA_ONLY);
				// }
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
				{
					return (C2_HA_ONLY_State);
				}
			}
			else if (systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF)
			{
				// Alarm Settings
				if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
				{
					return (C1_HA_DP_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					return (C1_HA_DP_State);
				}
				// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
				// {
				// 	return (StateID::C1_HA_DP);
				// }
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
				{
					return (C1_HA_DP_State);

				}
			}
			else if (systemsettings_current.DoorPower == DoorOpt::d_CarONManOFF || systemsettings_current.DoorPower == DoorOpt::d_ManONManOFF)
			{
				// Alarm Settings
				if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
				{
					return (C1_HA_DP_State);
				}
				else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					return (C1_HA_DP_State);
				}
				// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
				// {
				// 	return (StateID::C1_HA_DP);
				// }
				else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
				{
					return (C1_HA_DP_State);

				}
			}

		}

	}
	else if (CurrentPowerOnTrigger == PowerOnTrigger::IgnitionOn)
	{
		MONITOR.println("Power Trigger: Ignition On");
		// Door Popper Settings
		if (systemsettings_current.DoorPower == DoorOpt::d_OFF)
		{
			// Alarm Settings
			if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
			{
				return (D8_PowerDownByIgnitionOFF_State);
			}
			else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF || systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF)
			{
				return (C2_HA_ONLY_State);
			}
			else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
			{
				return (C2_HA_ONLY_State);
			}
			// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
			// {
			// 	return (StateID::C2_HA_ONLY);
			// }
			else if (systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
			{
				return (E3_Unknown_State);
			}

		}
		else if (systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF || systemsettings_current.DoorPower == DoorOpt::d_CarONManOFF)
		{
			// Alarm Settings
			if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
			{
				return (C3_DP_ONLY_State);
			}
			else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF || systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF)
			{
				return (C1_HA_DP_State);
			}
			else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
			{
				return (C1_HA_DP_State);
			}
			// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
			// {
			// 	return (StateID::C1_HA_DP);
			// }
			else if (systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
			{
				return (C3_DP_ONLY_State);
			}

		}
		else if (systemsettings_current.DoorPower == DoorOpt::d_ManONManOFF)
		{
			// Alarm Settings
			if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
			{
				return (E3_Unknown_State);
			}
			else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF || systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF)
			{
				return (C2_HA_ONLY_State);
			}
			else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
			{
				return (C2_HA_ONLY_State);
			}
			// else if (systemsettings_current.AlarmPower == PowerOpt::p_AutoStart)
			// {
			// 	return (StateID::C2_HA_ONLY);
			// }
			else if (systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
			{
				return (E3_Unknown_State);
			}

		}
	}

	MONITOR.println("WTF?");

return (E3_Unknown_State);
}

void Set_Determined_State()
{	
	Set_Next_State(Determine_Next_State());
}

File settings_file;

void save_settings() {

    if(SPIFFS.exists("/settings")) {
        SPIFFS.remove("/settings");
    }
    settings_file = SPIFFS.open("/settings","wb",true);
    if(sizeof(systemsettings_current)>settings_file.write((uint8_t*)&systemsettings_current,sizeof(systemsettings_current))) {
        //MONITOR.println("Error writing settings file. Too few bytes written");
    }
    settings_file.close();

}

void print_settings(){
    //MONITOR.println("Printing System Settings");
    //MONITOR.println("AlarmPower: " + (String)(systemsettings_current.AlarmPower));
}

bool load_settings() {
    //MONITOR.println("Loading Settings File");
    memcpy(&systemsettings_current,&systemsettings_default,sizeof(systemsettings_current));
    if(SPIFFS.exists("/settings"))
	{
		settings_file = SPIFFS.open("/settings","rb",false);
	}
    if(!settings_file) {
		memcpy(&systemsettings_current,&systemsettings_default,sizeof(systemsettings_current));
        //MONITOR.println("Settings File Not Found");
        return false;
    }
    if(sizeof(systemsettings_current)>settings_file.read((uint8_t*)&systemsettings_current,sizeof(systemsettings_current))) {
        memcpy(&systemsettings_current,&systemsettings_default,sizeof(systemsettings_current));
        settings_file.close();
        //MONITOR.println("Settings File Invalid (read length)");
        return false;
    }
    settings_file.close();
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

    //MONITOR.printf("Message from %s interface:\n",xbee_user_data_interface(data->source));

    if(payload_length<1) {
        //MONITOR.println("No frame payload received");
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
			last_packet.processed = false;
            last_received = true;
        }
        break;
        case COMMAND_ID::COMMAND: {
            //MONITOR.println("Command Packet Received");
            command_packet pck;
            memcpy(&pck,payload+5,payload_length-5);
            last_packet.cmd = pck.cmd_ID;
            command_data = pck;
            last_received = true;
        }
        break;
        case COMMAND_ID::CONFIG: {
            //MONITOR.println("Config Packet Received");
            config_packet pck;
            memcpy(&pck,payload+5,payload_length-5);
            
            last_packet.cmd = pck.cmd_ID;
            config_data = pck;
            last_received = true;
            
        }
        break;
		case COMMAND_ID::UPDATE: {
            MONITOR.println("Update Packet Received");
            update_packet pck;
            memcpy(&pck,payload+5,payload_length-5);
            
            last_packet.cmd = pck.cmd_ID;
			last_packet.processed = false;

            update_data = pck;
            last_received = true;
            
        }
        break;  
		case COMMAND_ID::FOTA: {
			MONITOR.println("FOTA Packet Received");
            fota_packet pck;
            memcpy(&pck,payload+5,payload_length-5);
            //MONITOR.printf("Acknowledge Packet Received: %d\n",pck.status);
            last_packet.cmd = pck.cmd_ID;
			last_packet.processed = false;
            last_received = true;
        }
        break;
    }
#ifdef DUMP_PACKETS
    // If all characters of message are printable, just print it as a string
    // with printf().  Otherwise use hex_dump() for non-printable messages.
    // int printable = TRUE;
    // for (size_t i = 0; printable && i < payload_length; ++i) {
    //     if (!isprint(data->payload[i])) {
    //         printable = FALSE;
    //     }
    // }

    // if (printable) {
        //MONITOR.printf("%.*s\n\n", payload_length, data->payload);
    // } else {
        hex_dump(data->payload, payload_length, HEX_DUMP_FLAG_OFFSET);
    // }
#endif
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
    if (value){
        lv_obj_add_state(ui_SwitchALM, LV_STATE_CHECKED);
    }else{
        lv_obj_clear_state(ui_SwitchALM, LV_STATE_CHECKED);
    }
    //MONITOR.printf("ALM Val Set to  %s\r\n",aceconvalues_current.alm?"HIGH":"LOW");
    digitalWrite(ACECON_ALM_OUT,value);
}

void Sync_Alarm_States(){

	MONITOR.print("Previous Alarm State - ");
	switch (Previous_System_Alarm_State)
	{
	case a_None:
		MONITOR.println("None");
		break;
	case a_Init:
		MONITOR.println("Init");
		break;
	case a_PreAlarm:
		MONITOR.println("PreAlarm");
		break;
	case a_FullAlarm:
		MONITOR.println("FullAlarm");
		break;
	case a_Snooze:
		MONITOR.println("Snooze");
		break;
	
	default:
		break;
	}

	MONITOR.print("Current Alarm State - ");
	switch (Current_System_Alarm_State)
	{
	case a_None:
		MONITOR.println("None");
		break;
	case a_Init:
		MONITOR.println("Init");
		break;
	case a_PreAlarm:
		MONITOR.println("PreAlarm");
		break;
	case a_FullAlarm:
		MONITOR.println("FullAlarm");
		break;
	case a_Snooze:
		MONITOR.println("Snooze");
		break;
	
	default:
		break;
	}

	MONITOR.println("Syncing Alarm States");
	Previous_System_Alarm_State = Current_System_Alarm_State;

}

void Set_Alarm_State(AlarmState alarm){

	Sync_Alarm_States();

	#ifdef ALARM_MANUAL_CONTROL
		MONITOR.print("Manual Alarm: Ignoring request - ");
	#else
		MONITOR.print("Set Alarm State: ");
		Current_System_Alarm_State = alarm;
	#endif

	switch (alarm) {
		case a_None:
			MONITOR.println("a_None");
			break;
		case a_PreAlarm:
			MONITOR.println("a_PreAlarm");
			break;
		case a_FullAlarm:
			MONITOR.println("a_FullAlarm");
			break;
		case a_Snooze:
			MONITOR.println("a_Snooze");
			break;
		// Add other cases as needed
		default:
			MONITOR.println("Unknown");
			break;
	}

}

void ui_update_alarms(){
	// MONITOR.println("ui update alarms");
	// if (Current_System_Alarm_State == AlarmState::a_None)
	// {
	// 	// Check if the Alarm State has changed
	// 	if (Previous_System_Alarm_State != Current_System_Alarm_State)
	// 	{
	// 		MONITOR.println("Current Alarm State: None");

			

	// 	}

	// }
	// else if (Current_System_Alarm_State == AlarmState::a_PreAlarm)
	// {
		

	// 	// Check if the Alarm State has changed
	// 	if (Previous_System_Alarm_State != Current_System_Alarm_State)
	// 	{

	// 	}

	// 	if (UpdatePreAlarmFlag == true)
	// 	{
	// 		UpdatePreAlarmFlag = false;

			

	// 		// static char szPreAlarmCounter[6];
	// 		// sprintf(szPreAlarmCounter,"%3i",DisplayPreAlarmCounter);

	// 		// lv_label_set_text(ui_LabelAlarmCounter, szPreAlarmCounter);

	// 	}

	// }
	// else if (Current_System_Alarm_State == AlarmState::a_Snooze)
	// {
	// 	MONITOR.println("Current Alarm State: Snooze");
		
	// 	String strText;
	// 	char Text[AlarmTextMaxChars];
	// 	char timerOutput[9]; // "HH:MM:SS" is 8 characters + null terminator

	// 	// Check if the Alarm State has changed
	// 	if (Previous_System_Alarm_State != Current_System_Alarm_State)
	// 	{
			

	// 	}

	// 	if (UpdateSnoozeAlarmFlag == true)
	// 	{
	// 		UpdateSnoozeAlarmFlag = false;

	// 		// Display PreAlarm Counter
			

	// 	}

	// }
	// else if (Current_System_Alarm_State == AlarmState::a_FullAlarm)
	// 	{

			

	// 		// Check if the Alarm State has changed
	// 		if (Previous_System_Alarm_State != Current_System_Alarm_State)
	// 		{

				

	// 		}

	// 		if (UpdateFullAlarmFlag == true)
	// 		{
	// 			UpdateFullAlarmFlag = false;

				

	// 		}

	// 	}


}

void convertToTimerFormat_H_M_S(long seconds, char* buffer) {
  int hours = seconds / 3600;           // Calculate the hours
  int minutes = (seconds % 3600) / 60;  // Calculate the remaining minutes
  int secs = seconds % 60;              // Calculate the remaining seconds

  // Format the time as Hr:Min:Sec and store it in the buffer
  snprintf(buffer, 9, "%02d:%02d:%02d", hours, minutes, secs);
}

void convertToTimerFormat_M_S(long seconds, char* buffer) {
  //int hours = seconds / 3600;           // Calculate the hours
  int minutes = (seconds % 3600) / 60;  // Calculate the remaining minutes
  int secs = seconds % 60;              // Calculate the remaining seconds

  // Format the time as Hr:Min:Sec and store it in the buffer
  snprintf(buffer, 6, "%02d:%02d", minutes, secs);
}

void ui_Alarm_PreAlarm_Initial(){

	String strText;
	char Text[AlarmTextMaxChars];

	MONITOR.println("Current Alarm State: PreAlarm");
	lv_obj_clear_flag(ui_BtnSnooze,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmText,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmCounter,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmTextExtra,LV_OBJ_FLAG_HIDDEN);

	MONITOR.println("PreAlertText: " + PreAlertText);
	PreAlertText.toCharArray(Text,sizeof(Text));
	Text[sizeof(Text) - 1] = '\0';

	lv_label_set_text(ui_LabelAlarmText, Text);

	
	strText = "SNOOZE";
	strText.toCharArray(Text,sizeof(Text));
	Text[sizeof(Text) - 1] = '\0';

	lv_label_set_text(ui_LabelAlarmTextExtra, Text);


	DisplayPreAlarmCounter = DisplayPreAlarmCounter_Default;

	static char szPreAlarmCounter[6];
	convertToTimerFormat_M_S(DisplayPreAlarmCounter, szPreAlarmCounter);
	lv_label_set_text(ui_LabelAlarmCounter, szPreAlarmCounter);
}

void ui_Alarm_PreAlarm_Update(){

	char timerOutput[6]; // "HH:MM:SS" is 8 characters + null terminator
	convertToTimerFormat_M_S(DisplayPreAlarmCounter, timerOutput);
	lv_label_set_text(ui_LabelAlarmCounter, timerOutput);
}

void handlePreAlarmState(){

	if (Previous_System_Alarm_State != AlarmState::a_PreAlarm)
	{
		MONITOR.println("Transitioned to PreAlarm state");

		// Send HPT(ALM) Signal
		set_ALM(LOW);

		// Set Trigger Time and Counter
		PreAlarmTriggerTime = millis();
		MONITOR.printf("PreAlarmTriggerTime set to %lu\n", PreAlarmTriggerTime);
		PreviousPreAlarmCounter = 0;
		CurrentPreAlarmCounter = -1; // -1 Ensures that the Screen updates initially since it looks for a change between Current and Prev
		DisplayPreAlarmCounter = DisplayPreAlarmCounter_Default;

		// Enable Pre Alarm Notification Timer
		PreAlarmNotification_Timer.StartTimer(PreAlarmNotification_Timer.Threshold);
		MONITOR.printf("PreAlarmNotification_Timer started with threshold %lu\n", PreAlarmNotification_Timer.Threshold);

		// Enable Timer
		SystemAlarm_Timer.StartTimer(SystemAlarm_Timer.Threshold);
		MONITOR.printf("SystemAlarm_Timer started with threshold %lu\n", SystemAlarm_Timer.Threshold);

		// Set Prev State
		Sync_Alarm_States();
		
		ui_Alarm_PreAlarm_Initial();

	}

	// Check if the Alarm conditions have been cleared
	if (data_global.batt_error_current == false && data_global.engineStalled_current == false && data_global.temp_alarmFlag_current == false && data_global.temp_errorFlag_current == false && data_global.Aux1Input_current == false && data_global.Aux2Input_current == false)
	{
		MONITOR.println("All alarm conditions cleared in PreAlarm state");

		// Battery Flag
		if (data_global.batt_error_previous == true) { data_global.battchanged = true; }
		// Stall Flag
		if (data_global.engineStalled_previous == true) { data_global.engine_changed = true; }
		// Temperature Flag
		if (data_global.temp_alarmFlag_previous == true) { data_global.temp_changed = true; }
		// Temperature Sensor Flag
		if (data_global.temp_errorFlag_previous == true) { data_global.temp_changed = true; }
		// DaisyChain Flag
		if (data_global.Aux1Input_previous == true || data_global.Aux2Input_previous == true) { data_global.aux_changed = true; }

		// Set the general update flag
		data_global.updateIcons  = true;

		// Disable Pre Alarm Notification Timer
		PreAlarmNotification_Timer.StopTimer();

		// Clear the OverFlow Flag and Disable the Timer
		SystemAlarm_Timer.StopTimer();
		DisplayPreAlarmCounter = DisplayPreAlarmCounter_Default;

		// Set Alarm to None
		Set_Alarm_State(AlarmState::a_None);
		
	}
	// Check if the alarm needs to graduate to full
	else if (SystemAlarm_Timer.OverFlowFlag == true)
	{
		MONITOR.println("PreAlarm Timer overflow detected, transitioning to FullAlarm");

		// Check Battery
		if (data_global.batt_error_current == true) { data_global.battchanged = true; }
		// Check Stall
		if (data_global.engineStalled_current == true) { data_global.engine_changed = true; }
		// Temperature Flag
		if (data_global.temp_alarmFlag_current == true) { data_global.temp_changed = true; }
		// Temperature Sensor Flag
		if (data_global.temp_errorFlag_current == true) { data_global.temp_changed = true; }
		// DiasyChain Flag
		if (data_global.Aux1Input_current == true || data_global.Aux2Input_current == true) { data_global.aux_changed = true; }

		// Set the General Update Flag
		data_global.updateIcons = true;

		// Disable Pre Alarm Notification Timer
		PreAlarmNotification_Timer.StopTimer();
		DisplayPreAlarmCounter = DisplayPreAlarmCounter_Default;

		Set_Alarm_State(AlarmState::a_FullAlarm);
		MONITOR.println("Transitioning to FullAlarm from PreAlarm");
	}
	// Check if the Countdown Timer needs to be updated
	else if (PreAlarmNotification_Timer.OverFlowFlag == true)
	{
		MONITOR.println("PreAlarmNotification_Timer overflow detected");
		PreAlarmNotification_Timer.OverFlowFlag = false;
		PreAlarmNotification_Timer.SyncTimerVal();

		// Set Pre Alarm Sound Pulse
		//Set_SoundPulse(PulseProfile::pp_DoubleBeep);
	}

	// Check if the counter has changed enough to warrant an update to the screen
	
	CurrentPreAlarmCounter = static_cast<int>((millis() - PreAlarmTriggerTime) / 1000);
	MONITOR.printf("CurrentPreAlarmCounter updated to %d\n", CurrentPreAlarmCounter);

	if (CurrentPreAlarmCounter != PreviousPreAlarmCounter)
	{
		PreviousPreAlarmCounter = CurrentPreAlarmCounter;

		// Update Display Counter
		DisplayPreAlarmCounter = MaxPreAlarmTime - CurrentPreAlarmCounter;
		MONITOR.printf("DisplayPreAlarmCounter updated to %d\n", DisplayPreAlarmCounter);

		// Make sure it doesn't go below zero
		if (DisplayPreAlarmCounter < 0) { DisplayPreAlarmCounter = 0; }

		ui_Alarm_PreAlarm_Update();

	}

}

void ui_Alarm_Snooze_Initial(){

	String strText;
	char Text[AlarmTextMaxChars];
	char timerOutput[6]; // "MM:SS" is 5 characters + null terminator

	lv_obj_add_flag(ui_BtnSnooze,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmText,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmCounter,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmTextExtra,LV_OBJ_FLAG_HIDDEN);

	PreAlertText.toCharArray(Text,sizeof(Text));
	Text[sizeof(Text) - 1] = '\0';

	lv_label_set_text(ui_LabelAlarmText, Text);

	
	strText = "SNOOZED";
	strText.toCharArray(Text,sizeof(Text));
	Text[sizeof(Text) - 1] = '\0';

	lv_label_set_text(ui_LabelAlarmTextExtra, Text);


	DisplaySnoozeAlarmCounter = DisplaySnoozeAlarmCounter_Default;

	// Convert the counter to Hr:Min:Sec format
	
	convertToTimerFormat_M_S(DisplaySnoozeAlarmCounter, timerOutput);

	lv_label_set_text(ui_LabelAlarmCounter, timerOutput);

	// Set_SoundPulse(PulseProfile::pp_TripleBeep);

}

void ui_Alarm_Snooze_Update(){

	char timerOutput[6]; // "MM:SS" is 5 characters + null terminator
	
	convertToTimerFormat_M_S(DisplaySnoozeAlarmCounter, timerOutput);

	lv_label_set_text(ui_LabelAlarmCounter, timerOutput);
}

void handleSnoozeState(){
	
	if (Previous_System_Alarm_State != AlarmState::a_Snooze)
	{
		MONITOR.println("Transitioning to Snooze state");

		// Send HPT(ALM) Signal
		set_ALM(LOW);

		// Set Trigger Time and Counter
		SnoozeAlarmTriggerTime = millis();
		MONITOR.printf("SnoozeAlarmTriggerTime set to %lu\n", SnoozeAlarmTriggerTime);
		PreviousSnoozeAlarmCounter = 0;
		CurrentSnoozeAlarmCounter = -1; // -1 Ensures that the Screen updates initially since it looks for a change between Current and Prev
		DisplaySnoozeAlarmCounter = DisplaySnoozeAlarmCounter_Default;

		// Enable Pre Alarm Notification Timer
		SnoozeAlarm_Timer.StartTimer(SnoozeAlarm_Timer.Threshold);
		MONITOR.printf("SnoozeAlarm_Timer started with threshold %lu\n", SnoozeAlarm_Timer.Threshold);

		// Set Prev State
		Sync_Alarm_States();
		
		ui_Alarm_Snooze_Initial();
	}

	// Check if all Alarm conditions have been cleared
	if (data_global.batt_error_current == false && data_global.engineStalled_current == false && data_global.temp_alarmFlag_current == false && data_global.temp_errorFlag_current == false && data_global.Aux1Input_current == false && data_global.Aux2Input_current == false)
	{
		MONITOR.println("All alarm conditions cleared in Snooze state");

		// Check Battery
		if (data_global.batt_error_previous == true) { data_global.battchanged = true; }
		//Check Stall
		if (data_global.engineStalled_previous == true) { data_global.engine_changed  = true; }
		// Check Temperature
		if (data_global.temp_alarmFlag_previous == true) { data_global.temp_changed = true; }
		// Check Temperature Sensor Flag
		if (data_global.temp_errorFlag_previous == true) { data_global.temp_changed = true; }
		// Daisy Chain
		if (data_global.Aux1Input_previous == true || data_global.Aux2Input_previous == true) { data_global.aux_changed = true; }

		// Set the General Update Flag
		data_global.updateIcons = true;

		// Set Alarm to None
		Set_Alarm_State(AlarmState::a_None);
	}

	// Check if the Snooze Timer has Overflowed
	else if (SnoozeAlarm_Timer.OverFlowFlag == true)
	{
		MONITOR.println("SnoozeAlarm_Timer overflow detected, transitioning to PreAlarm");

		// Check Battery
		if (data_global.batt_error_current == true) { data_global.battchanged = true; }
		// Check Stall
		if (data_global.engineStalled_current == true) { data_global.engine_changed = true; }
		// Temperature Flag
		if (data_global.temp_alarmFlag_current == true) { data_global.temp_changed = true; }
		// TEmperature Sensor Flag
		if (data_global.temp_errorFlag_current == true) { data_global.temp_changed = true; }
		// Daisy Chain Flag
		if (data_global.Aux1Input_current == true || data_global.Aux2Input_current == true) { data_global.aux_changed = true; }

		// Set the General Update Flag
		data_global.updateIcons = true;

		// Clear the OverFlow Flag and Disable the Timer
		SnoozeAlarm_Timer.StopTimer();
		DisplaySnoozeAlarmCounter = DisplaySnoozeAlarmCounter_Default;

		// Set Alarm to Full
		Set_Alarm_State(AlarmState::a_PreAlarm);
		
	}

	// Check if the counter has changed enough to warrant an update to the screen
	PreviousSnoozeAlarmCounter = CurrentSnoozeAlarmCounter;
	CurrentSnoozeAlarmCounter = static_cast<int>((millis() - SnoozeAlarmTriggerTime) / 1000);
	MONITOR.printf("CurrentSnoozeAlarmCounter: %d, PreviousSnoozeAlarmCounter: %d\n", CurrentSnoozeAlarmCounter, PreviousSnoozeAlarmCounter);

	if (CurrentSnoozeAlarmCounter != PreviousSnoozeAlarmCounter)
	{
		PreviousSnoozeAlarmCounter = CurrentSnoozeAlarmCounter;

		// Update Display Counter
		DisplaySnoozeAlarmCounter = MaxSnoozeAlarmTime - CurrentSnoozeAlarmCounter;
		MONITOR.printf("DisplaySnoozeAlarmCounter updated to %d\n", DisplaySnoozeAlarmCounter);

		// Make sure it doesn't go below zero
		if (DisplaySnoozeAlarmCounter < 0) { DisplaySnoozeAlarmCounter = 0; }

		ui_Alarm_Snooze_Update();

	}

}

void ui_Alarm_FullAlarm_Initial(){

	String strText;
	char Text[AlarmTextMaxChars];
	char timerOutput[6]; // "MM:SS" is 5 characters + null terminator

	MONITOR.println("Current Alarm State: FullAlarm");
	lv_obj_clear_flag(ui_BtnSnooze,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmText,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmCounter,LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(ui_LabelAlarmTextExtra,LV_OBJ_FLAG_HIDDEN);

	FullAlarmText.toCharArray(Text,sizeof(Text));
	Text[sizeof(Text) - 1] = '\0';

	lv_label_set_text(ui_LabelAlarmText, Text);

	
	strText = "SNOOZE";
	strText.toCharArray(Text,sizeof(Text));
	Text[sizeof(Text) - 1] = '\0';

	lv_label_set_text(ui_LabelAlarmTextExtra, Text);


	CurrentFullAlarmCounter = 0;

	// Convert the counter to Hr:Min:Sec format
	
	convertToTimerFormat_M_S(CurrentFullAlarmCounter, timerOutput);

	lv_label_set_text(ui_LabelAlarmCounter, timerOutput);

}

void ui_Alarm_FullAlarm_Update(){
	
	char timerOutput[6]; // "HH:MM:SS" is 8 characters + null terminator

	convertToTimerFormat_M_S(CurrentFullAlarmCounter, timerOutput);

	lv_label_set_text(ui_LabelAlarmCounter, timerOutput);
}

void handleFullAlarmState(){

	if (Previous_System_Alarm_State != AlarmState::a_FullAlarm)
	{
		MONITOR.println("Transitioned to FullAlarm state");

		// Emit Alarm Pulse
		//Set_SoundPulse(PulseProfile::pp_SystemTest);

		// Send HPT(ALM) Signal
		set_ALM(HIGH);

		// Clear any previous Counters
		PreviousFullAlarmCounter = 0;
		CurrentFullAlarmCounter = 0;

		// Log the trigger time
		FullAlarmTriggerTime = millis();
		MONITOR.printf("FullAlarmTriggerTime set to %lu\n", FullAlarmTriggerTime);

		// Set Prev State
		Sync_Alarm_States();
		ui_Alarm_FullAlarm_Initial();
	}

	// Check if all Alarm conditions have been cleared and System Test is not Active. If System test is active then that will be handled by the system test state
	if (data_global.batt_error_current == false && data_global.engineStalled_current == false && data_global.temp_alarmFlag_current == false && data_global.temp_errorFlag_current == false && System_Test_Alarm_Active == false && data_global.Aux1Input_current == false && data_global.Aux2Input_current == false)
	{
		MONITOR.println("All alarm conditions cleared in FullAlarm state");

		// Check Battery
		if (data_global.batt_error_previous == true) { data_global.battchanged = true; }
		//Check Stall
		if (data_global.engineStalled_previous == true) { data_global.engine_changed = true; }
		// Check Temperature
		if (data_global.temp_alarmFlag_previous == true) { data_global.temp_changed = true; }
		// Check Temperature Sensor Flag
		if (data_global.temp_errorFlag_previous == true) { data_global.temp_changed = true; }
		// Daisy Chain Flag
		if (data_global.Aux1Input_previous == true || data_global.Aux2Input_previous == true) { data_global.aux_changed = true; }

		// Set the General Update Flag
		data_global.updateIcons = true;

		// Disable Timer and Clear OverFlow Flag
		SystemAlarm_Timer.StopTimer();
		MONITOR.println("SystemAlarm_Timer stopped");

		// Set Alarm to None
		Set_Alarm_State(AlarmState::a_None);
		MONITOR.println("Transitioning to None state from FullAlarm");
	}

	// Get Count
	PreviousFullAlarmCounter = CurrentFullAlarmCounter;
	CurrentFullAlarmCounter = static_cast<long>((millis() - FullAlarmTriggerTime) / 1000);
	//MONITOR.printf("CurrentFullAlarmCounter: %ld, PreviousFullAlarmCounter: %ld\n", CurrentFullAlarmCounter, PreviousFullAlarmCounter);

	if (CurrentFullAlarmCounter != PreviousFullAlarmCounter)
	{
		// Set Flag so display gets updated
		UpdateFullAlarmFlag = true;
		// Set General Update Flag
		ui_Alarm_FullAlarm_Update();
	}

}

void ui_Alarm_None_Initial(){

	lv_obj_add_flag(ui_BtnSnooze,LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(ui_LabelAlarmText,LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(ui_LabelAlarmCounter,LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(ui_LabelAlarmTextExtra,LV_OBJ_FLAG_HIDDEN);
}

void handleNoneState(){

	if (Previous_System_Alarm_State != AlarmState::a_None)
	{
		MONITOR.println("Transitioning to None state");

		// Send HPT(ALM) Signal
		set_ALM(LOW);

		// Set No Sound Pulse
		//Set_SoundPulse(PulseProfile::pp_NoSound);

		// Clear the Pre-Alarm, Snooze and Full Alarm Variables and Timers
		PreviousPreAlarmCounter = 0;
		CurrentPreAlarmCounter = -1; // -1 Ensures that the Screen updates initially since it looks for a change between Current and Prev
		DisplayPreAlarmCounter = DisplayPreAlarmCounter_Default;
		PreviousSnoozeAlarmCounter = 0;
		CurrentSnoozeAlarmCounter = -1; // -1 Ensures that the Screen updates initially since it looks for a change between Current and Prev
		DisplaySnoozeAlarmCounter = DisplaySnoozeAlarmCounter_Default;
		PreviousFullAlarmCounter = 0;
		CurrentFullAlarmCounter = 0;

		// Disable Timers
		PreAlarmNotification_Timer.StopTimer();
		SystemAlarm_Timer.StopTimer();
		SnoozeAlarm_Timer.StopTimer();
		MONITOR.println("All alarm-related timers stopped");

		// Set Prev State
		Sync_Alarm_States();
		
		// Draw Alarm UI
		ui_Alarm_None_Initial();

	}

	// Check Battery, Stall, Temp Alarm, Aux
	if (data_global.batt_error_current == true || data_global.engineStalled_current == true || data_global.temp_alarmFlag_current == true || data_global.temp_errorFlag_current == true || data_global.Aux1Input_current == true || data_global.Aux2Input_current == true)
	{
		MONITOR.println("Alarm condition detected in None state");

		// Check Battery
		if (data_global.batt_error_previous == true) { data_global.battchanged = true; }
		// Check Stall
		if (data_global.engineStalled_previous == true) { data_global.engine_changed = true; }
		// Check Temp
		if (data_global.temp_alarmFlag_previous == true) { data_global.temp_changed = true; }
		// Check Temperature Sensor Flag
		if (data_global.temp_errorFlag_previous == true) { data_global.temp_changed = true; }
		// DaisyChain Flag
		if (data_global.Aux1Input_previous == true || data_global.Aux2Input_previous == true) { data_global.aux_changed = true; }

		// Set the General Update Flag
		data_global.updateIcons = true;

		if (systemsettings_current.AutoSnoozeEnabled == true) { 
			MONITOR.println("AutoSnooze is enabled"); 
			}
		if (systemsettings_current.InitialPowerUpFlag == true) {
			 MONITOR.println("InitialPowerUpFlag is set"); 
			 }

		// AUTOSNOOZE FUNCTIONALITY HERE
		// If the system has just woken up then go to snooze
		if (systemsettings_current.InitialPowerUpFlag == true && systemsettings_current.AutoSnoozeEnabled == true)
		{
			// Go to Snooze Alarm State
			Set_Alarm_State(AlarmState::a_Snooze);
		}
		else
		{
			// Set Alarm to Pre ALarm
			Set_Alarm_State(AlarmState::a_PreAlarm);
		}
	}
}

void Process_System_Alarm_States()
{
	data_global = vim_load();

	// Make Sure Alarms are enabled (this is state dependant)
	if (AlarmStateEnabled == false)
	{
		
		if (Current_System_Alarm_State == AlarmState::a_None && Previous_System_Alarm_State == AlarmState::a_Init){

		}else{
			MONITOR.println("Alarm State Disabled");
			Current_System_Alarm_State = AlarmState::a_None; // Set to none
			Previous_System_Alarm_State = AlarmState::a_Init; // Setting previous to anything other than none will ensure that the alarm states will disable

		}
		
	}
	else
	{

		// System Alarm is in prealarm
		if (Current_System_Alarm_State == AlarmState::a_PreAlarm)
		{
			handlePreAlarmState();
		}
		// System is Snoozed
		else if (Current_System_Alarm_State == AlarmState::a_Snooze)
		{
			handleSnoozeState();
		}
		// System is in Full Alarm
		else if (Current_System_Alarm_State == AlarmState::a_FullAlarm)
		{
			handleFullAlarmState();
		}
		// System alarm is current off
		else if (Current_System_Alarm_State == AlarmState::a_None)
		{
			handleNoneState();
		}

		
	}

	vim_store(data_global);

}

void Reset_System_Alarm_State()
{
	data_global = vim_load();

	Previous_System_Alarm_State = AlarmState::a_None;
	Current_System_Alarm_State = AlarmState::a_None;

	MONITOR.println("Alarm set to None");
	//Set_SoundPulse(PulseProfile::pp_NoSound);

	// Clear HPT(ALM) Signal
	set_ALM(LOW);

	// Clear the Pre-Alarm, Snooze and Full Alarm Variables and Timers
	PreviousPreAlarmCounter = 0;
	CurrentPreAlarmCounter = -1; // -1 Ensures that the Screen updates initially since it looks for a change between Current and Prev
	DisplayPreAlarmCounter = DisplayPreAlarmCounter_Default;
	PreviousSnoozeAlarmCounter = 0;
	CurrentSnoozeAlarmCounter = -1; // -1 Ensures that the Screen updates initially since it looks for a change between Current and Prev
	DisplaySnoozeAlarmCounter = DisplaySnoozeAlarmCounter_Default;
	PreviousFullAlarmCounter = 0;
	CurrentFullAlarmCounter = 0;

	// Disable Timers
	PreAlarmNotification_Timer.StopTimer();
	SystemAlarm_Timer.StopTimer();
	SnoozeAlarm_Timer.StopTimer();

	// Clear any update flags
	data_global.batt_error_current = false;
	data_global.engineStalled_current = false;
	data_global.temp_alarmFlag_current = false;
	data_global.temp_errorFlag_current = false;
	data_global.Aux1Input_current = false;
	data_global.Aux2Input_current = false;

	// Set No Sound Pulse
	//Set_SoundPulse(PulseProfile::pp_NoSound);
	vim_store(data_global);
}

void Check_IBoxPopped()
{
	// if (Read_IBoxPopped_Timer.OverFlowFlag == true)
	// {

	// 	Read_IBoxPopped_Timer.OverFlowFlag = false;
	// 	Read_IBoxPopped_Timer.SyncTimerVal();


	// 	// Make Sure System will allow the door popper to work
	// 	if (Current_INGEAR == PARKED && Clear_IBoxPopped_Timer.TimerEnable == false && Current_HotnPopFlag == true)
	// 	{
	// 		// Determine Door Trigger. Only process a new trigger  after the None State has been set. This ensure the system fully process
	// 		// the previous trigger event
	// 		if (KeyPressDetected(KeyBCode::UpDown) && Current_DoorPopTrigger == DoorPopTrigger::dt_None && Current_System_Alarm_State == AlarmState::a_None)
	// 		{
	// 			Current_DoorPopTrigger = DoorPopTrigger::dt_Salute;
	// 		}
	// 		else if (digitalRead(ACECON_POP_IN) == DoorPopCondition::dc_Popped && Current_DoorPopTrigger == DoorPopTrigger::dt_None)
	// 		{
	// 			Current_DoorPopTrigger = DoorPopTrigger::dt_Remote;
	// 		}


	// 		if (Current_DoorPopTrigger != DoorPopTrigger::dt_None)
	// 		{

	// 			Set_SoundPulse(PulseProfile::pp_DoubleBeep);
	// 			CurrentDoorPopCondition = DoorPopCondition::dc_Popped;

	// 			if (Current_DoorPopTrigger == DoorPopTrigger::dt_Salute)
	// 			{
	// 				digitalWrite(ACECON_POP_OUT, HIGH);
	// 				SaluteHold_Timer.StartTimer(SaluteHold_Timer.Threshold);
	// 			}


	// 			data_global.updateIcons = true;
	// 			UpdateDoorFlag = true;

	// 			// Start Timer
	// 			Clear_IBoxPopped_Timer.StartTimer(Clear_IBoxPopped_Timer.Threshold);

	// 		}


	// 	}

	// 	if (SaluteHold_Timer.OverFlowFlag == true)
	// 	{
	// 		SaluteHold_Timer.OverFlowFlag = false;
	// 		SaluteHold_Timer.StopTimer();
	// 		digitalWrite(ACECON_POP_OUT, LOW);

	// 	}

	// 	// Check if its time to clear the popped condition
	// 	if (Clear_IBoxPopped_Timer.OverFlowFlag == true)
	// 	{
	// 		Clear_IBoxPopped_Timer.OverFlowFlag = false;

	// 		Clear_IBoxPopped_Timer.StopTimer();


	// 		Current_DoorPopTrigger = DoorPopTrigger::dt_None;
	// 		CurrentDoorPopCondition = DoorPopCondition::dc_Unpopped;
	// 		data_global.updateIcons = true;
	// 		UpdateDoorFlag = true;

	// 	}






	// }



}

void send_init_config_packet() {
    int len = strlen(INIT_CONFIG_PACKET_CSV);
	MONITOR.println(INIT_CONFIG_PACKET_CSV);
    char* sz = (char*)malloc(6+len)+5;
    strcpy(sz,INIT_CONFIG_PACKET_CSV);
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

void send_init_data_packet() {
    int len = strlen(INIT_DATA_PACKET_CSV);
	MONITOR.println(INIT_DATA_PACKET_CSV);
    char* sz = (char*)malloc(6+len)+5;
    strcpy(sz,INIT_DATA_PACKET_CSV);
    if(sz==(char*)5) {
        MONITOR.println("Out of memory. Tough luck.");
        return;
    }
    sz[len]=0;
    uint32_t crc = crc32(0,(uint8_t*)sz,len);
    uint8_t* p = (uint8_t*)(sz-5);
    *p=253;
    memcpy(p+1,&crc,4);
    sendUserDataRelayAPIFrame(&my_xbee,(char*)p,len+5);
    free(p);
}

void send_init_status_packet() {
    int len = strlen(INIT_STATUS_PACKET_CSV);
	MONITOR.println(INIT_STATUS_PACKET_CSV);

    char* sz = (char*)malloc(6+len)+5;
    strcpy(sz,INIT_STATUS_PACKET_CSV);
    if(sz==(char*)5) {
        MONITOR.println("Out of memory. Tough luck.");
        return;
    }
	strcpy(sz,INIT_STATUS_PACKET_CSV);

    sz[len]=0;
    uint32_t crc = crc32(0,(uint8_t*)sz,len);
    uint8_t* p = (uint8_t*)(sz-5);
    *p=254;
    memcpy(p+1,&crc,4);
    sendUserDataRelayAPIFrame(&my_xbee,(char*)p,len+5);
    free(p);

	// int len = strlen(INIT_STATUS_PACKET_CSV);
    // MONITOR.println(INIT_STATUS_PACKET_CSV);

    // // Allocate memory for the packet, including space for header and CRC.
    // char* sz = (char*)malloc(6 + len); // Allocate enough space for header (1 byte), CRC (4 bytes), and the data
    // if (sz == nullptr) {
    //     MONITOR.println("Out of memory. Tough luck.");
    //     return;
    // }

    // // Copy the data into the allocated memory with a 5-byte offset for header and CRC.
    // strcpy(sz + 1, INIT_STATUS_PACKET_CSV); // Shift data by 1 byte for header
    // sz[len + 1] = 0; // Null-terminate the data

    // // Calculate CRC32 over the data part only (excluding the header)
    // uint32_t crc = crc32(0, (uint8_t*)(sz + 1), len);

    // // Set the header (first byte) and copy the CRC32
    // uint8_t* p = (uint8_t*)sz;
    // *p = (uint8_t)COMMAND_ID::INIT_STATUS; // Use the enum value for the header
    // memcpy(p + 1 + len, &crc, 4); // Copy CRC32 at the end of the data

    // // Send the packet
    // sendUserDataRelayAPIFrame(&my_xbee, (char*)p, len + 5);

    // // Free the allocated memory
    // free(sz);

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

    //MONITOR.printf("\nResponse for: %s\n", response->command.str);
    
    last_at_cmd.commandstr[0] = response->command.str[0];
    last_at_cmd.commandstr[1] = response->command.str[1];

    if (response->flags & XBEE_CMD_RESP_FLAG_TIMEOUT)
    {
        //MONITOR.println("(timed out)");
        return XBEE_ATCMD_DONE;
    }

    status = response->flags & XBEE_CMD_RESP_MASK_STATUS;
    if (status != XBEE_AT_RESP_SUCCESS)
    {
        /*MONITOR.printf("(error: %s)\n",
                      (status == XBEE_AT_RESP_ERROR) ? "general" : (status == XBEE_AT_RESP_BAD_COMMAND) ? "bad command"
                                                               : (status == XBEE_AT_RESP_BAD_PARAMETER) ? "bad parameter"
                                                               : (status == XBEE_AT_RESP_TX_FAIL)       ? "Tx failure"
                                                                                                        : "unknown error");
        */
		return XBEE_ATCMD_DONE;
    }

    length = response->value_length;
    if (!length) // command sent successfully, no value to report
    {
        //MONITOR.println("(success)");
        return XBEE_ATCMD_DONE;
    }
    if (length <= 4)
    {
        
        last_at_cmd.value = response->value;
        last_at_cmd.value_received = true;

    }
#ifdef DUMP_PACKETS
    // check to see if we can print the value out as a string
    printable = 1;
    p = response->value_bytes;
    for (i = length; printable && i; ++p, --i)
    {
        printable = isprint(*p);
    }

    if (printable)
    {
        //MONITOR.printf("= \"%.*" PRIsFAR "\" ", length, response->value_bytes);
    }
    if (length <= 4)
    {
        // format hex string with (2 * number of bytes in value) leading zeros
        //MONITOR.printf("= 0x%0*" PRIX32 " (%" PRIu32 ")\n", length * 2, response->value, response->value);

        

    }
    else if (length <= 32)
    {
        // format hex string
        //MONITOR.printf("= 0x");
        for (i = length, p = response->value_bytes; i; ++p, --i)
        {
            //MONITOR.printf("%02X", *p);
        }
        //MONITOR.println("");
    }
    else
    {

        //MONITOR.printf("= %d bytes:\n", length);

        hex_dump(response->value_bytes, length, HEX_DUMP_FLAG_TAB);
    }
#endif
    return XBEE_ATCMD_DONE;
}

void on_xbee_error(COMMAND_ID id, STATUS_CODE code) {
    //MONITOR.printf("XBee Error: Command (%d), Status (%d)\n",(int)id,(int)code);
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
            //MONITOR.printf("Error: (%d) waiting for query to complete.\n",status);
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
        //MONITOR.printf("Error creating request: %d\n",request, strerror(-request));
    }
    else
    {

        //MONITOR.println("Sending command to xbee");
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
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending connect packet\n", status);
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
    data.newstuff = ACE_TRUE;

    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending data packet\n", status);
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
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending connection packet\n", status);
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
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending status packet\n", status);
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
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending log packet\n", status);
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
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
        
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending config packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_subscribe_config(const char* str) {
    
    last_packet.cmd = COMMAND_ID::SUBSCRIBE;

    subscribe_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "config");
    strcpy(data.handlerType, "config");
    
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));

    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending config packet\n", status);
    }
    else 
    {
        //MONITOR.println("Subscribed to config topic");
    }

    
}

void on_monitor_subscribe_command(const char* str) {
    
    last_packet.cmd = COMMAND_ID::SUBSCRIBE;

    subscribe_packet data;
    memset(&data, 0, sizeof(data));
    strcpy(data.topicName, "command");
    strcpy(data.handlerType, "command");
    
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));

    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending config packet\n", status);
    }
    else 
    {
        //MONITOR.println("Subscribed to command topic");
    }

    
}

void on_monitor_check_fota(const char* str) {
    
    last_packet.cmd = COMMAND_ID::FOTA;
	last_packet.status = STATUS_CODE::FOTA_CHECK_FW;
	last_packet.processed = true;
	
    fota_packet data;
    memset(&data, 0, sizeof(data));
	data.fotaStatus = STATUS_CODE::FOTA_CHECK_FW;    
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));

    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending fota packet\n", status);
    }
    else 
    {
        //MONITOR.println("fota packet sent");
    }

    
}

void on_monitor_begin_fota(const char* str) {
    
    last_packet.cmd = COMMAND_ID::FOTA;
	last_packet.status = STATUS_CODE::FOTA_BEGIN;
	last_packet.processed = true;
	
    fota_packet data;
    memset(&data, 0, sizeof(data));
	data.fotaStatus = STATUS_CODE::FOTA_BEGIN;    
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));

    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending fota packet\n", status);
    }
    else 
    {
        
    }

    
}

void on_monitor_fota_loop_begin(const char* str) {
    
    CurrentFOTACode = FOTA_Begin;    
}

void on_monitor_delete_file(const char* str) {
    
    last_packet.cmd = COMMAND_ID::FOTA;
	last_packet.status = STATUS_CODE::FOTA_DELETE_FILE;
	last_packet.processed = false;
	
    fota_packet data;
    memset(&data, 0, sizeof(data));
	data.fotaStatus = STATUS_CODE::FOTA_DELETE_FILE;    
        
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        //MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)last_packet.cmd;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));

    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);

    if (status < 0) 
    {
        //MONITOR.printf("Error %d sending fota packet\n", status);
    }
    else 
    {
        //MONITOR.println("fota packet sent");
    }

    
}

void on_monitor_alarm_none(const char* str){
	Current_System_Alarm_State = AlarmState::a_None;
}

void on_monitor_alarm_prealarm(const char* str){
	Current_System_Alarm_State = AlarmState::a_PreAlarm;
}

void on_monitor_alarm_snooze(const char* str){
	Current_System_Alarm_State = AlarmState::a_Snooze;
}

void on_monitor_alarm_fullalarm(const char* str){
	Current_System_Alarm_State = AlarmState::a_FullAlarm;
}

void monitor_dev_tick(HardwareSerial& s) {
    if(s.available()) {
        String str = s.readString();
        //MONITOR.printf("ECHO: %s\n",str.c_str());
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
        } else if (cmd=="check fota"){
			on_monitor_check_fota(str.c_str());
		} else if (cmd=="begin fota"){
			on_monitor_begin_fota(str.c_str());
		} else if (cmd=="delete file"){
			on_monitor_delete_file(str.c_str());
		}else if (cmd=="subscribe config"){
            on_monitor_subscribe_config(str.c_str()); 
        } else if (cmd=="subscribe command"){
            on_monitor_subscribe_command(str.c_str());
        } else if (cmd=="save default settings"){
            systemsettings_current = systemsettings_default;    
            save_settings();            
        } else if (cmd=="save custom settings"){
            //systemsettings_current.AlarmPower = PowerOpt::p_ManONManOFF;
            save_settings();
        } else if (cmd=="report settings"){
            print_settings();
        } else if (cmd=="init data packet"){
            send_init_data_packet();
        } else if (cmd=="init config packet"){
            send_init_config_packet();
        }else if (cmd=="init status packet"){
            send_init_status_packet();
        }else if (cmd=="fota loop begin"){
            on_monitor_fota_loop_begin(str.c_str());
        }else if (cmd=="alarm none"){
            on_monitor_alarm_none(str.c_str());
        } else if (cmd=="alarm prealarm"){
            on_monitor_alarm_prealarm(str.c_str());
        } else if (cmd=="alarm snooze"){
            on_monitor_alarm_snooze(str.c_str());
        } else if (cmd=="alarm fullalarm"){
            on_monitor_alarm_fullalarm(str.c_str());
        }
    }
}

float ConvertFtoC(float F)
{
	return (round((F - 32) * (5.0 / 9)));
}

void ui_update_acecon() {

	MONITOR.println("ui update acecon");
	data_global = vim_load();

    if (data_global.temp_changed || data_global.units_changed){
        MONITOR.println("UI: Temp Values Need Updating");
        data_global.updateIcons = true;

        if(data_global.leftTemp_previous!=data_global.leftTemp_current || data_global.units_changed) {
            ////MONITOR.println("UI: Updating Left Temp");
            static char szLeftTemp[6];
            if (systemsettings_current.AlarmUnitF){
                sprintf(szLeftTemp,"%3.1f",data_global.leftTemp_current);
            }else{
                sprintf(szLeftTemp,"%3.1f",ConvertFtoC(data_global.leftTemp_current));
            }
            
			if (data_global.leftTempState == TempState::tst_Over || data_global.leftTempState == TempState::tst_OverPlus)
			{
				lv_obj_set_style_text_color(ui_LabelLeftTemp, lv_color_hex(COLOR_RED), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else if (data_global.leftTempState == TempState::tst_Warning)
			{
				lv_obj_set_style_text_color(ui_LabelLeftTemp, lv_color_hex(COLOR_YELLOW), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else if (data_global.leftTempState == TempState::tst_Under)
			{
				lv_obj_set_style_text_color(ui_LabelLeftTemp, lv_color_hex(COLOR_BLUE), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else
			{
				lv_obj_set_style_text_color(ui_LabelLeftTemp, lv_color_hex(COLOR_GREEN), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			
            lv_label_set_text(ui_LabelTemp1Val,szLeftTemp);
            lv_label_set_text(ui_LabelLeftTemp,szLeftTemp);
        }
				
        if (data_global.rightTemp_previous!=data_global.rightTemp_current || data_global.units_changed) {
            ////MONITOR.println("UI: Updating Right Temp");
            static char szRightTemp[6];
            if (systemsettings_current.AlarmUnitF){
                sprintf(szRightTemp,"%3.1f",data_global.rightTemp_current);
            }else{
                sprintf(szRightTemp,"%3.1fleftTempState",ConvertFtoC(data_global.rightTemp_current));
            }

			if (data_global.rightTempState == TempState::tst_Over || data_global.rightTempState == TempState::tst_OverPlus)
			{
				lv_obj_set_style_text_color(ui_LabelRightTemp, lv_color_hex(COLOR_RED), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else if (data_global.rightTempState == TempState::tst_Warning)
			{
				lv_obj_set_style_text_color(ui_LabelRightTemp, lv_color_hex(COLOR_YELLOW), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else if (data_global.rightTempState == TempState::tst_Under)
			{
				lv_obj_set_style_text_color(ui_LabelRightTemp, lv_color_hex(COLOR_BLUE), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else
			{
				lv_obj_set_style_text_color(ui_LabelRightTemp, lv_color_hex(COLOR_GREEN), LV_PART_MAIN | LV_STATE_DEFAULT);
			}

            lv_label_set_text(ui_LabelTemp2Val,szRightTemp);
            lv_label_set_text(ui_LabelRightTemp,szRightTemp);
        }

        if (systemsettings_current.TempAveragingEnabled){
            ////MONITOR.println("UI: Updating Avg Temp");
            static char szAvgTemp[6];

            if (systemsettings_current.AlarmUnitF){
                sprintf(szAvgTemp,"%3.1f",data_global.avgTemp);
            }else{
                sprintf(szAvgTemp,"%3.1f",ConvertFtoC(data_global.avgTemp));
            }

			if (data_global.avgTempState == TempState::tst_Over || data_global.avgTempState == TempState::tst_OverPlus)
			{
				lv_obj_set_style_text_color(ui_LabelTempAvg, lv_color_hex(COLOR_RED), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else if (data_global.avgTempState == TempState::tst_Warning)
			{
				lv_obj_set_style_text_color(ui_LabelTempAvg, lv_color_hex(COLOR_YELLOW), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else if (data_global.avgTempState == TempState::tst_Under)
			{
				lv_obj_set_style_text_color(ui_LabelTempAvg, lv_color_hex(COLOR_BLUE), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			else
			{
				lv_obj_set_style_text_color(ui_LabelTempAvg, lv_color_hex(COLOR_GREEN), LV_PART_MAIN | LV_STATE_DEFAULT);
			}

            lv_label_set_text(ui_LabelTempAvg,szAvgTemp);

        }else{
			lv_obj_set_style_text_color(ui_LabelTempAvg, lv_color_hex(COLOR_GREEN), LV_PART_MAIN | LV_STATE_DEFAULT);
			lv_label_set_text(ui_LabelTempAvg,"Disabled");
        }



		// ============= TEMPERATURE EVALUATION =============

		// Check Temperature Values are within range
		if (systemsettings_current.TempAveragingEnabled == true)
		{
			MONITOR.println("Temp Averaging Enabled");
			// Sensor 1
			if (data_global.leftTemp_current >= (HotTempOpt_F[systemsettings_current.AlarmHotSetIndex]) + SingleTempOverrideAmount)
			{
				MONITOR.println("Left Temp: Over");

				PreAlertText = "HOT ALERT";
				FullAlarmText = "HOT  ALARM";
				data_global.leftTempState = TempState::tst_OverPlus;
			}
			else if (data_global.leftTemp_current >= ((HotTempOpt_F[systemsettings_current.AlarmHotSetIndex]) + SingleTempOverrideAmount) - TempWarningOffset)
			{
				MONITOR.println("Left Temp: Warning");
				data_global.leftTempState = TempState::tst_Warning;
			}
			else if (systemsettings_current.ColdAlarmEnabled == true && data_global.leftTemp_current <= ColdTempOpt_F[systemsettings_current.AlarmColdSetIndex])
			{
				MONITOR.println("Left Temp: Under");

				PreAlertText = "COLD ALERT";
				FullAlarmText = "COLD  ALARM";
				data_global.leftTempState = TempState::tst_Under;
			}
			else
			{
				PreAlertText = "";
				FullAlarmText = "";
				MONITOR.println("Left Temp: OK");
				data_global.leftTempState = TempState::tst_OK;
			}

			// Sensor 2
			if (data_global.rightTemp_current >= (HotTempOpt_F[systemsettings_current.AlarmHotSetIndex]) + SingleTempOverrideAmount)
			{
				MONITOR.println("Right Temp: Over");
				PreAlertText = "HOT ALERT";
				FullAlarmText = "HOT  ALARM";
				data_global.rightTempState = TempState::tst_OverPlus;
			}
			else if (data_global.rightTemp_current >= ((HotTempOpt_F[systemsettings_current.AlarmHotSetIndex]) + SingleTempOverrideAmount) - TempWarningOffset)
			{
				MONITOR.println("Right Temp: Warning");
				data_global.rightTempState = TempState::tst_Warning;
			}
			else if (systemsettings_current.ColdAlarmEnabled == true && data_global.rightTemp_current <= ColdTempOpt_F[systemsettings_current.AlarmColdSetIndex])
			{
				MONITOR.println("Right Temp: Under");
				PreAlertText = "COLD ALERT";
				FullAlarmText = "COLD  ALARM";
				data_global.rightTempState = TempState::tst_Under;
			}
			else
			{
				PreAlertText = "";
				FullAlarmText = "";
				MONITOR.println("Right Temp: OK");
				data_global.rightTempState = TempState::tst_OK;
			}

			// Check if the Average Value is out of range
			if (data_global.avgTemp >= HotTempOpt_F[systemsettings_current.AlarmHotSetIndex])
			{
				MONITOR.println("Average Temp: Over");

				PreAlertText = "HOT ALERT";
				FullAlarmText = "HOT  ALARM";
				
				data_global.avgTempState = TempState::tst_Over;
			}
			else if (data_global.avgTemp >= (HotTempOpt_F[systemsettings_current.AlarmHotSetIndex] - TempWarningOffset))
			{
				MONITOR.println("Average Temp: Warning");
				data_global.avgTempState = TempState::tst_Warning;
			}
			else if (systemsettings_current.ColdAlarmEnabled == true && data_global.avgTemp <= ColdTempOpt_F[systemsettings_current.AlarmColdSetIndex])
			{
				PreAlertText = "COLD ALERT";
				FullAlarmText = "COLD  ALARM";
				MONITOR.println("Average Temp: Under");
				data_global.avgTempState = TempState::tst_Under;
			}
			else
			{
				PreAlertText = "";
				FullAlarmText = "";
				MONITOR.println("Average Temp: OK");
				data_global.avgTempState = TempState::tst_OK;
			}

			// Clear Sensor Errors if needed
			if (data_global.temp_errorFlag_current == true)
			{
				// If averaging is enabled 
				if (data_global.leftTempError_current == false || data_global.rightTempError_current == false)
				{
					// Both Sensors are fine
					data_global.temp_errorFlag_previous = data_global.temp_errorFlag_current;
					data_global.temp_errorFlag_current = false;

				}
			}
			else
			{
				// Check if both sensors have thrown an error
				if (data_global.leftTempError_current == true && data_global.rightTempError_current == true)
				{
					// Both Sensors failed
					PreAlertText = "SENSOR ALERT";
					FullAlarmText = "SENSOR  ALARM";

					data_global.temp_errorFlag_previous = data_global.temp_errorFlag_current;
					data_global.temp_errorFlag_current = true;


				}
			}

		}
		else
		{
			// Sensor 1
			if (data_global.leftTemp_current >= (HotTempOpt_F[systemsettings_current.AlarmHotSetIndex]))
			{
				PreAlertText = "HOT ALERT";
				FullAlarmText = "HOT  ALARM";
				data_global.leftTempState = TempState::tst_Over;
			}
			else if (data_global.leftTemp_current >= (HotTempOpt_F[systemsettings_current.AlarmHotSetIndex] - TempWarningOffset))
			{
				data_global.leftTempState = TempState::tst_Warning;
			}
			else if (systemsettings_current.ColdAlarmEnabled == true && data_global.leftTemp_current <= ColdTempOpt_F[systemsettings_current.AlarmColdSetIndex])
			{
				PreAlertText = "COLD ALERT";
				FullAlarmText = "COLD  ALARM";
				data_global.leftTempState = TempState::tst_Under;
			}
			else
			{
				data_global.leftTempState = TempState::tst_OK;
			}

			// Sensor 2
			if (data_global.rightTemp_current >= (HotTempOpt_F[systemsettings_current.AlarmHotSetIndex]))
			{
				PreAlertText = "HOT ALERT";
				FullAlarmText = "HOT  ALARM";
				data_global.rightTempState = TempState::tst_Over;
			}
			else if (data_global.rightTemp_current >= (HotTempOpt_F[systemsettings_current.AlarmHotSetIndex] - TempWarningOffset))
			{
				data_global.rightTempState = TempState::tst_Warning;
			}
			else if (systemsettings_current.ColdAlarmEnabled == true && data_global.rightTemp_current <= ColdTempOpt_F[systemsettings_current.AlarmColdSetIndex])
			{
				PreAlertText = "COLD ALERT";
				FullAlarmText = "COLD  ALARM";
				data_global.rightTempState = TempState::tst_Under;
			}
			else
			{
				data_global.rightTempState = TempState::tst_OK;
			}

			// Clear Sensor Errors if needed
			if (data_global.temp_errorFlag_current == true)
			{
				if (data_global.leftTempError_current == false && data_global.rightTempError_current == false)
				{
					// Both Sensors are fine
					data_global.temp_errorFlag_previous = data_global.temp_errorFlag_current;
					data_global.temp_errorFlag_current = false;

				}
			}
			else
			{
				// Check if both sensors have thrown an error
				if (data_global.leftTempError_current == true || data_global.rightTempError_current == true)
				{
					// Either Sensor failed

					PreAlertText = "SENSOR ALERT";
					FullAlarmText = "SENSOR  ALARM";

					data_global.temp_errorFlag_previous = data_global.temp_errorFlag_current;
					data_global.temp_errorFlag_current = true;


				}
			}

		}


		// Check if Alarms need to be notified
		if (systemsettings_current.TempAveragingEnabled == true)
		{
			// If Average Temp Value is OVer or Under
			if (data_global.avgTempState == tst_Over || data_global.avgTempState == tst_Under)
			{
				
				data_global.temp_alarmFlag_previous = data_global.temp_alarmFlag_current;
				data_global.temp_alarmFlag_current = true;
			}
			// If either individual sensor is over the +10 threshold
			else if (data_global.leftTempState == tst_OverPlus || data_global.rightTempState == tst_OverPlus)
			{
				data_global.temp_alarmFlag_previous = data_global.temp_alarmFlag_current;
				data_global.temp_alarmFlag_current = true;
			}
			else
			{
				data_global.temp_alarmFlag_previous = data_global.temp_alarmFlag_current;
				data_global.temp_alarmFlag_current = false;
			}
		}
		else
		{
			// If either sensor is over or under
			if (data_global.leftTempState == tst_Over || data_global.leftTempState == tst_Under ||
				data_global.rightTempState == tst_Over || data_global.rightTempState == tst_Under)
			{
				data_global.temp_alarmFlag_previous = data_global.temp_alarmFlag_current;
				data_global.temp_alarmFlag_current = true;
			}
			else
			{
				data_global.temp_alarmFlag_previous = data_global.temp_alarmFlag_current;
				data_global.temp_alarmFlag_current = false;
			}


		}

		// Check if Datas need to be updated
		// Only update temp when a integer change is detected
		if (static_cast<int>(data_global.leftTemp_previous) != static_cast<int>(data_global.leftTemp_current) ||
			static_cast<int>(data_global.rightTemp_previous) != static_cast<int>(data_global.rightTemp_current) ||
			(systemsettings_current.TempAveragingEnabled == true && static_cast<int>(data_global.avgTemp) != static_cast<int>(data_global.avgTemp)) ||
			data_global.temp_errorFlag_previous != data_global.temp_errorFlag_current ||
			data_global.leftTempError_current != data_global.leftTempError_previous ||
			data_global.rightTempError_current != data_global.rightTempError_previous)
		{
			
			data_global.updateIcons = true;

		}

        if (data_global.units_changed){

			data_global.updateIcons = true;

            data_global.units_changed = false;
        }
        
		if (data_global.temp_changed){

			data_global.updateIcons = true;

            data_global.temp_changed = false;
        }

        
    }

    if (data_global.engine_changed){
        ////MONITOR.println("UI: Engine Values Need Updating");
        data_global.updateIcons = true;

        ////MONITOR.println("UI: Updating Ignition");
        if (data_global.ignitionOn_current) {
            MONITOR.println("UI: Clearing Ignition State");
            lv_obj_clear_state(ui_ImgButtonKey, LV_STATE_PRESSED); 
                      
        } else {
        	MONITOR.println("UI: Adding Ignition State");
            lv_obj_add_state(ui_ImgButtonKey, LV_STATE_PRESSED);

                       
        }

        if(data_global.ignitionOn_current) {
            lv_obj_add_state(ui_SwitchIGN, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchIGN, LV_STATE_CHECKED);
        }

        
        if (data_global.engineStalled_current && systemsettings_current.StallMonitorEnabled){
            ////MONITOR.println("UI: Adding Engine Stalled State");
            lv_obj_add_state(ui_ImgButtonEngine, LV_STATE_PRESSED);
        } else{
            ////MONITOR.println("UI: Clearing Engine Stalled State");
            lv_obj_clear_state(ui_ImgButtonEngine, LV_STATE_PRESSED);
        }


        data_global.engine_changed = false;
    }
    
    if (data_global.battchanged){
        ////MONITOR.println("UI: Temp Values Need Updating");
        data_global.updateIcons = true;

        if (data_global.batt_error_current == true){
			PreAlertText = "LO BATT ALERT";
			FullAlarmText = "LO BATT ALARM";
            lv_obj_add_state(ui_ImgButtonBattery, LV_STATE_PRESSED);
        }else{
            lv_obj_clear_state(ui_ImgButtonBattery, LV_STATE_PRESSED);
        }
        
        data_global.battchanged = false;
    }

    if (data_global.k9_door_changed){

        data_global.updateIcons = true;

        if (systemsettings_current.doorDisabled){
            //MONITOR.println("UI: Door Disabled");
            lv_obj_add_flag(ui_ImgButtonDoorClosed,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorOpen,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorPopped,LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_ImgButtonDoorDisabled,LV_OBJ_FLAG_HIDDEN);
        }else if  (data_global.k9_door_popped){
            //MONITOR.println("UI: Door Popped");
            lv_obj_add_flag(ui_ImgButtonDoorClosed,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorOpen,LV_OBJ_FLAG_HIDDEN);            
            lv_obj_add_flag(ui_ImgButtonDoorDisabled,LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_ImgButtonDoorPopped,LV_OBJ_FLAG_HIDDEN);
        }else if  (data_global.k9_door_open_current){
            //MONITOR.println("UI: Door Opened");
            lv_obj_add_flag(ui_ImgButtonDoorClosed,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorPopped,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorDisabled,LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_ImgButtonDoorOpen,LV_OBJ_FLAG_HIDDEN);           
        }else{
            //MONITOR.println("UI: Door Closed");
            lv_obj_add_flag(ui_ImgButtonDoorOpen,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorPopped,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonDoorDisabled,LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_ImgButtonDoorClosed,LV_OBJ_FLAG_HIDDEN);
        }

        data_global.k9_door_changed = false;

    }

    if (xbee_signal_strength_changed){
        data_global.updateIcons = true;

        //MONITOR.print("Changing WiFi Icon:");

        if (xbee_signal_strength_current >= XBEE_SIGNALSTRENGTH0){
            //MONITOR.println("0");
                lv_obj_clear_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }else if (xbee_signal_strength_current >= XBEE_SIGNALSTRENGTH1){
            //MONITOR.println("1");
            lv_obj_add_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }else if (xbee_signal_strength_current >= XBEE_SIGNALSTRENGTH2){
            //MONITOR.println("2");
            lv_obj_add_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }else if (xbee_signal_strength_current >= XBEE_SIGNALSTRENGTH3){
            //MONITOR.println("3");
            lv_obj_add_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }else{
            //MONITOR.println("4");
            lv_obj_add_flag(ui_ImgButtonWiFi0,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi1,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi2,LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_ImgButtonWiFi3,LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(ui_ImgButtonWiFi4,LV_OBJ_FLAG_HIDDEN);
        }

        xbee_signal_strength_changed = false;
    }

	vim_store(data_global);
       
}

void ui_update_icons() {

	data_global = vim_load();

    if(data_global.updateIcons){

		MONITOR.println("Redrawing Screen");
        lv_scr_load(lv_scr_act());
        data_global.updateIcons = false;

    } 

	vim_store(data_global);
       
}

void acedata_parse_temperature(String str){
       
    // tempvalues_previous = tempvalues_current;
    
    // //================================================== Left Temp =========================================*/
    // ////MONITOR.printf("Left Temp Sign: %c\n",str.charAt(ACEDATA_Temp1_Sign_POS));
    // acedata_current.leftTempSign = (str.charAt(ACEDATA_Temp1_Sign_POS));
    
    // // Check for Error First
    // if (acedata_current.leftTempSign == TempErrorGeneral){
                
    //     if (str.charAt(ACEDATA_Temp1_10X_POS) == TempErrorOpen || str.charAt(ACEDATA_Temp1_10X_POS) == TempErrorSho){
    //         acedata_current.leftTempSign = str.charAt(ACEDATA_Temp1_10X_POS);
    //     }
        
    //     data.leftTempError = true;
    //     data.leftTempSign = acedata_current.leftTempSign;

    // }
    // else 
    // {
        
    //     // Parse Temperature Magnitude	
    //     String ia =str.substring(ACEDATA_Temp1_1000X_POS,ACEDATA_Temp1_1X_POS+1);
       
    //     acedata_current.leftTemp = ((float)atoi(ia.c_str()))/10.0f;
        
    //     if(acedata_current.leftTempSign == TempSignNeg){
    //         acedata_current.leftTemp = acedata_current.leftTemp * -1;
    //     }

    //     data.leftTemp_current= acedata_current.leftTemp;

    //     data.leftTempError = false;

    // }

    // // //MONITOR.print("Left Temp Value: ");
    // // //MONITOR.println(data.leftTemp);

    // //================================================== Right Temp =========================================*/
    // ////MONITOR.printf("Right Temp Sign: %c\n",str.charAt(ACEDATA_Temp2_Sign_POS));
    // acedata_current.rightTempSign = (str.charAt(ACEDATA_Temp2_Sign_POS));
    
    // // Check for Error First
    // if (acedata_current.rightTempSign == TempErrorGeneral){
                
    //     if (str.charAt(ACEDATA_Temp2_10X_POS) == TempErrorOpen || str.charAt(ACEDATA_Temp2_10X_POS) == TempErrorSho){
    //         acedata_current.rightTempSign = str.charAt(ACEDATA_Temp2_10X_POS);
    //     }
        
    //     data.rightTempError = true;
    //     data.rightTempSign = acedata_current.rightTempSign;

    // }
    // else 
    // {
        
    //     // Parse Temperature Magnitude	
    //     String ia =str.substring(ACEDATA_Temp2_1000X_POS,ACEDATA_Temp2_1X_POS+1);
       
    //     acedata_current.rightTemp = ((float)atoi(ia.c_str()))/10.0f;
        
    //     if(acedata_current.rightTempSign == TempSignNeg){
    //         acedata_current.rightTemp = acedata_current.rightTemp * -1;
    //     }

    //     data.rightTemp = acedata_current.rightTemp;

    //     data.rightTempError = false;

    // }

    // ////MONITOR.print("Right Temp Value: ");
    // ////MONITOR.println(data.rightTemp);

    // //================================================== Average Temp =========================================*/

    // // If Both Sensors have not returned an error
    // if (data.leftTempError == false && data.rightTempError == false){
        
    //     data.avgTemp = (data.leftTemp_current+ data.rightTemp)/2;
    // }
    // else
    // {
    //     // If a single sensor is failed, used the working sensor value as the average
    //     if (data.leftTempError == true && data.rightTempError == false)
    //     {
    //         data.avgTemp = data.rightTemp;
    //     }
    //     else if (data.leftTempError == false && data.rightTempError == true)
    //     {
    //         data.avgTemp = data.leftTemp;
    //     }
    // }

    // ////MONITOR.print("Average Temp Value: ");
    // ////MONITOR.println(data.avgTemp);

    // if (tempvalues_previous.leftTemp!=data.leftTemp_current|| tempvalues_previous.rightTemp!=data.rightTemp){
    //     data.valueChanged = true;
    // }

}

void acedata_parse_battery(String str){
       
    // battvalues_previous = battvalues_current;
    
    // //================================================== Battery Value =========================================*/
    	
    //     String ia =str.substring(ACEDATA_Batt_100X_POS,ACEDATA_Batt_1X_POS+1);
       
    //     acedata_current.batteryVoltage = ((float)atoi(ia.c_str()))/10.0f;
    
    // battvalues_current.voltage = acedata_current.batteryVoltage;

    // // //MONITOR.print("Battery Voltage: ");
    // // //MONITOR.println(battvalues_current.voltage);

    // // Check if the Batt Voltage is out of range
	// if (acedata_current.batteryVoltage < battvalues_current.voltage/10)
    // {
    //     BadBatteryCounter++;
    //     if (BadBatteryCounter > MaxBadBattValCounter)
    //     {
    //         battvalues_current.error = true;
    //     }
    // }
    // else
    // {
    //     BadBatteryCounter = 0;
    //     battvalues_current.error = false;
    // }

    // if (battvalues_previous.error != battvalues_current.error){
    //     battvalues_current.valueChanged = true;
    // }

}

void acedata_parse_engine_stall(String str){

    // enginevalues_previous = enginevalues_current;

    // if (str.charAt(ACEDATA_Stall_Status_POS)=='0'){
    //     ////MONITOR.println("Engine Stalled");
    //     acedata_current.engineStalled = true;
    // }else{
    //     ////MONITOR.println("Engine NOT Stalled");
    //     acedata_current.engineStalled = false;
    // }

    // String ia =str.substring(ACEDATA_Stall_Count_10X_POS,ACEDATA_Stall_Count_1X_POS+1);
       
    //     acedata_current.EngineStallCount = ((float)atoi(ia.c_str()))/10.0f;
    //     enginevalues_current.engineStallCount = acedata_current.EngineStallCount;
    
    // ////MONITOR.print("Engine Stall Count:");
    // ////MONITOR.println(enginevalues_current.engineStallCount);


    // if (str.charAt(ACEDATA_Stall_Sensor_Present_POS) =='A'){
    //     ////MONITOR.println("Engine Stall Sensor Present");
    //     acedata_current.engineStallSensorPresent = true;
    // }else{
    //     ////MONITOR.println("Engine Stall Sensor Not Present");
    //     acedata_current.engineStallSensorPresent = false;
    // }

    // enginevalues_current.engineStallSensorPresent = acedata_current.engineStallSensorPresent;

    // if (enginevalues_current.engineStallCount >= EngineStallThreshold){
    //     ////MONITOR.println("Stall Condition Achieved");
    //     enginevalues_current.engineStalled = true;
        
    // }else{
    //     enginevalues_current.engineStalled = false;
    // }

    // if (enginevalues_previous.engineStalled != enginevalues_current.engineStalled){
    //     ////MONITOR.println("Engine Stalled Valued Changed ");
    //     enginevalues_current.valueChanged = true;
    // }


}

void acedata_parse_aux(String str){
    
    // auxvalues_previous = auxvalues_current;

    // if (str.charAt(ACEDATA_Aux1_Input_POS)=='1'){
    //     acedata_current.Aux1Input = true;
    // }else{
    //     acedata_current.Aux1Input = false;
    // }
    // auxvalues_current.aux1Active = acedata_current.Aux1Input;

    // if (str.charAt(ACEDATA_Aux2_Input_POS)=='1'){
    //     acedata_current.Aux2Input = true;
    // }else{
    //     acedata_current.Aux2Input = false;
    // }
    // auxvalues_current.aux2Active = acedata_current.Aux2Input;

}

void acedata_parse_k9door(String str){
    
    // doorvalues_previous = doorvalues_current;

    // if (str.charAt(ACEDATA_K9Door_POS)=='1'){
    //     ////MONITOR.println("Door Open");
    //     acedata_current.K9DoorOpen = true;
	// 	doorvalues_current.doorOpen = DOOROPENState::Open;
    // }else{
    //     ////MONITOR.println("Door Closed");
    //     acedata_current.K9DoorOpen = false;
	// 	doorvalues_current.doorOpen = DOOROPENState::Closed;
    // }
    
    // if (doorvalues_previous.doorOpen != doorvalues_current.doorOpen){
    //     ////MONITOR.println("Door Value Changed");
    //     doorvalues_current.valueChanged = true;
    // }

}

static void ui_switch_handler(lv_event_t * e)
{
    
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        bool checked = lv_obj_has_state(obj,LV_STATE_CHECKED);
        if(obj==ui_SwitchPPT) {
            //MONITOR.printf("PPT Switch %s\r\n",checked?"on":"off");
            if (aceconvalues_previous.pps != aceconvalues_current.pps){
                set_PPT(checked?HIGH:LOW);
            }
        } else if(obj==ui_SwitchPPS) {
            //MONITOR.printf("PPS Switch %s\r\n",checked?"on":"off");
            set_PPS(checked?HIGH:LOW);
        } else if(obj==ui_SwitchALM) {
            //MONITOR.printf("ALM Switch %s\r\n",checked?"on":"off");
            set_ALM(checked?HIGH:LOW);
        } else if(obj==ui_SwitchHPS) {
            //MONITOR.printf("HPS Switch %s\r\n",checked?"on":"off");
            set_HPS(checked?HIGH:LOW);
        }
    }
}

static void ui_snooze_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        
		MONITOR.println("snooze button pressed");
        Set_Alarm_State(a_Snooze);


    }

}

static void menu_AlarmPower_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        //MONITOR.print("Alarm Power Changed To: ");
        systemsettings_previous.AlarmPower = systemsettings_current.AlarmPower;

        switch(lv_dropdown_get_selected(obj)){

            case 0:
                //MONITOR.println("CarON/CarOFF");
                systemsettings_current.AlarmPower = PowerOpt::p_CarONCarOFF;
            break;

            case 1:
                //MONITOR.println("CarON/ManOFF");
                systemsettings_current.AlarmPower = PowerOpt::p_CarONManOFF;
            break;

            case 2:
                //MONITOR.println("ManON/ManOFF");
                systemsettings_current.AlarmPower = PowerOpt::p_ManONManOFF;
            break;

            case 3:
                //MONITOR.println("NO K9 LEFT BEHIND");
                systemsettings_current.AlarmPower = PowerOpt::p_NoK9Left;
            break;

            case 4:
                //MONITOR.println("Always OFF");
                systemsettings_current.AlarmPower = PowerOpt::p_OFF;
            break;

            default:
                //MONITOR.println("Error: Unknown Power Option");
            break;

        }
       
    }

}

static void button_GoToMenu_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        
		MONITOR.println("Go To Menu Button Pressed");
        GoToMenuFlag = true;


    }

}

static void menu_DoorPower_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        //MONITOR.print("Door Power Changed To: ");
        systemsettings_previous.DoorPower = systemsettings_current.DoorPower;

        switch(lv_dropdown_get_selected(obj)){

            case 0:
                //MONITOR.println("CarON/CarOFF");
                systemsettings_current.DoorPower = DoorOpt::d_CarONCarOFF;
            break;

            case 1:
                //MONITOR.println("CarON/ManOFF");
                systemsettings_current.DoorPower = DoorOpt::d_CarONManOFF;
            break;

            case 2:
                //MONITOR.println("ManON/ManOFF");
                systemsettings_current.DoorPower = DoorOpt::d_ManONManOFF;
            break;

            case 3:
                //MONITOR.println("Always OFF");
                systemsettings_current.DoorPower = DoorOpt::d_OFF;
            break;

            default:
                //MONITOR.println("Error: Unknown Power Option");
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
        //MONITOR.print("Battery Voltage Setting Changed To: ");
        systemsettings_previous.BatteryVoltage = systemsettings_current.BatteryVoltage;

        switch (systemsettings_current.BatteryVoltage){

            case Batt::b_10:
                // Minimum Value
            break;

            case Batt::b_105:
                //MONITOR.println("10.0V");
                systemsettings_current.BatteryVoltage = Batt::b_10;
            break;

            case Batt::b_11:
                //MONITOR.println("10.5V");
                systemsettings_current.BatteryVoltage = Batt::b_105;                
            break;

            case Batt::b_115:
                //MONITOR.println("11.0V");
                systemsettings_current.BatteryVoltage = Batt::b_11;
            break;

            case Batt::b_12:
                //MONITOR.println("11.5V");
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
        //MONITOR.print("Battery Voltage Setting Changed To: ");
        systemsettings_previous.BatteryVoltage = systemsettings_current.BatteryVoltage;

        switch (systemsettings_current.BatteryVoltage){

            case Batt::b_10:
                //MONITOR.println("10.5V");
                systemsettings_current.BatteryVoltage = Batt::b_105;
            break;

            case Batt::b_105:
                //MONITOR.println("11.0V");
                systemsettings_current.BatteryVoltage = Batt::b_11;
            break;

            case Batt::b_11:
                //MONITOR.println("11.5V");
                systemsettings_current.BatteryVoltage = Batt::b_115;                
            break;

            case Batt::b_115:
                //MONITOR.println("12.0V");
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
        //MONITOR.print("Alarm Hot Set Changed To: ");
        systemsettings_previous.AlarmHotSetIndex = systemsettings_current.AlarmHotSetIndex;

        if (systemsettings_current.AlarmHotSetIndex == HotTempArraySize-1){
            //MONITOR.println("MAXIMUM VALUE");
        }else{
            systemsettings_current.AlarmHotSetIndex++;
            //MONITOR.println(print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex));
        }
        lv_label_set_text(ui_LabelAlarmHotSetValue,print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex).c_str());
        
    }

}

static void menu_AlarmHotSetDown_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        //MONITOR.print("Alarm Hot Set Changed To: ");
        systemsettings_previous.AlarmHotSetIndex = systemsettings_current.AlarmHotSetIndex;

        if (systemsettings_current.AlarmHotSetIndex == 0){
            // Minimum Value
        }else{
            systemsettings_current.AlarmHotSetIndex--;
            //MONITOR.println(print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex));
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
        //MONITOR.print("Alarm Cold Set Changed To: ");
        systemsettings_previous.AlarmColdSetIndex = systemsettings_current.AlarmColdSetIndex;
		systemsettings_current.ColdAlarmEnabled = true;
        if (systemsettings_current.AlarmColdSetIndex == ColdTempArraySize-1){
            //MONITOR.println("MAXIMUM VALUE");
        }else{
            systemsettings_current.AlarmColdSetIndex++;
            //MONITOR.println(print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex));
        }
        lv_label_set_text(ui_LabelAlarmColdSetValue,print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex).c_str());
        
    }

}

static void menu_AlarmColdSetDown_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        //MONITOR.print("Alarm Cold Set Changed To: ");
        systemsettings_previous.AlarmColdSetIndex = systemsettings_current.AlarmColdSetIndex;

        if (systemsettings_current.AlarmColdSetIndex == 0){
            // Minimum Value
        }else{
            systemsettings_current.AlarmColdSetIndex--;
            //MONITOR.println(print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex));
        }
        if (systemsettings_current.AlarmColdSetIndex==0){

			systemsettings_current.ColdAlarmEnabled = false;
            lv_label_set_text(ui_LabelAlarmColdSetValue,"OFF");
        }else{
            lv_label_set_text(ui_LabelAlarmColdSetValue,print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex).c_str());
        }
        
        
    }

}

static void menu_TempUnits_handler(lv_event_t * e)
{
	data_global = vim_load();

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        //MONITOR.print("Temp Units Changed To: ");
        systemsettings_previous.AlarmUnitF = systemsettings_current.AlarmUnitF;

        switch(lv_dropdown_get_selected(obj)){

            case 0:
                //MONITOR.println("Farenheit");
                systemsettings_current.AlarmUnitF = true;
            break;

            case 1:
                //MONITOR.println("Celsius");
                systemsettings_current.AlarmUnitF = false;
            break;
           
            default:
                //MONITOR.println("Error: Unknown Temp Units");
            break;

        }
    
    lv_label_set_text(ui_LabelAlarmColdSetValue,print_Alarm_Cold_Set(systemsettings_current.AlarmColdSetIndex).c_str());
    lv_label_set_text(ui_LabelAlarmHotSetValue,print_Alarm_Hot_Set(systemsettings_current.AlarmHotSetIndex).c_str()); 

    data_global.units_changed = true;

	vim_store(data_global);

    ui_update_acecon();

    }

}

static void menu_TempAveraging_handler(lv_event_t * e)
{
    data_global = vim_load();

	lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        //MONITOR.print("Temp Averaging Changed To: ");
        systemsettings_previous.TempAveragingEnabled = systemsettings_current.TempAveragingEnabled;


        if (lv_obj_get_state(obj) & LV_STATE_CHECKED){
            //MONITOR.println("True");
            systemsettings_current.TempAveragingEnabled = true;
        }else{
            //MONITOR.println("False");
            systemsettings_current.TempAveragingEnabled = false;
        }
            
    data_global.temp_changed = true;

	vim_store(data_global);

    ui_update_acecon();

    }

}

static void menu_AutoSnooze_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        //MONITOR.print("Auto Snooze Changed To: ");
        systemsettings_previous.AutoSnoozeEnabled = systemsettings_current.AutoSnoozeEnabled;

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED){
            //MONITOR.println("True");
            systemsettings_current.AutoSnoozeEnabled = true;
        }else{
            //MONITOR.println("False");
            systemsettings_current.AutoSnoozeEnabled= false;
        }
 
    }

}

static void menu_AuxInput_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        //MONITOR.print("Aux Input Changed To: ");
        systemsettings_previous.AuxInputEnabled = systemsettings_current.AuxInputEnabled;

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED){
            //MONITOR.println("True");
            systemsettings_current.AuxInputEnabled = true;
			data_global.AuxEnabled = true;
        }else{
            //MONITOR.println("False");
            systemsettings_current.AuxInputEnabled= false;
			data_global.AuxEnabled = false;
        }
            

    }

}

static void menu_StallMonitor_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        //MONITOR.print("Stall Monitor Enabled Changed To: ");
        systemsettings_previous.StallMonitorEnabled = systemsettings_current.StallMonitorEnabled;

        if (lv_obj_get_state(obj) & LV_STATE_CHECKED){
            //MONITOR.println("True");
            systemsettings_current.StallMonitorEnabled = true;
        }else{
            //MONITOR.println("False");
            systemsettings_current.StallMonitorEnabled= false;
        }
            
        data_global.engine_changed = true;
        ui_update_acecon();

    }

}

static void menu_ExitMenu_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        //MONITOR.println("Exiting Menu");

        //MONITOR.println("Saving Settings");
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

static void menu_RestoreDefaults_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        //MONITOR.println("Restoring Default Values");

        systemsettings_previous = systemsettings_current;
        systemsettings_current = systemsettings_default;

        update_menu_settings();
       
    }

}

void acedata_parse_serialnumber(String str){
        
    // systemsettings_current.VIMSerialNumber = str.substring(ACEDATA_VIM_SN_POS,ACEDATA_VIM_SN_POS+ACEDATA_VIM_SN_LENGTH);

    //     ////MONITOR.println("VIM SN:" + systemsettings_current.VIMSerialNumber);
    // if (systemsettings_previous.VIMSerialNumber != systemsettings_current.VIMSerialNumber){
    //     systemsettings_previous.VIMSerialNumber = systemsettings_current.VIMSerialNumber;
    //     ////MONITOR.println("VIM SN Changed:");
    //     lv_label_set_text(ui_LabelIntelaBoxSNValue,systemsettings_current.VIMSerialNumber.c_str());
    // }

}

void process_vim(const char* incoming, void* state) {
    static String trickle;
    if(*incoming) {
        String s(incoming);
        ////MONITOR.printf("Incoming! %s\n",incoming);
        trickle +=s;
        
    }
    int newlineIndex = trickle.indexOf("\n");
    int dollarsignIndex = trickle.indexOf("$");

    ////MONITOR.printf("Received trickle: %s\n",trickle.c_str());

    while(newlineIndex > -1) {

        if (dollarsignIndex == -1){
            
            trickle = trickle.substring(newlineIndex+1);
            ////MONITOR.printf("No Dollar Sign: Reset trickle: %s\n",trickle.c_str());
        }else{
            String line = trickle.substring(dollarsignIndex,newlineIndex+1);
            line.trim();
            ////MONITOR.printf("Parsed line: %s\n",line.c_str());

            trickle = trickle.substring(newlineIndex+1);
            ////MONITOR.printf("Reset trickle: %s\n",trickle.c_str());
                    
            int index=0;
            int length=10;

            if(line.substring(index,length)=="$ACEK9,IH1") {
				
				data_global = vim_load();

				data_global.Comms_OK = true;
			

				//================================================== K9 Door  =========================================*/
				
				data_global.k9_door_open_previous = data_global.k9_door_open_current;

				data_global.k9_door_open_current = (line.charAt(ACEDATA_K9Door_POS)=='1');
					
				if (data_global.k9_door_open_previous != data_global.k9_door_open_current){
					////MONITOR.println("Door Value Changed");
					data_global.k9_door_changed = true;
					data_global.updateIcons = true;
				}
				
				//================================================== Temperature  =========================================*/
				
				
				//================================================== Left Temp =========================================*/
				////MONITOR.printf("Left Temp Sign: %c\n",line.charAt(ACEDATA_Temp1_Sign_POS));
				data_global.leftTempSign = (line.charAt(ACEDATA_Temp1_Sign_POS));
				
				// Check for Error First
				if (data_global.leftTempSign == TempErrorGeneral){
							
					if (line.charAt(ACEDATA_Temp1_10X_POS) == TempErrorOpen || line.charAt(ACEDATA_Temp1_10X_POS) == TempErrorSho){
						data_global.leftTempSign = line.charAt(ACEDATA_Temp1_10X_POS);
					}
					
					data_global.leftTempError_previous = data_global.leftTempError_current;
					data_global.leftTempError_current= true;
					data_global.leftTempSign = data_global.leftTempSign;
					data_global.temp_errorFlag_previous = data_global.temp_errorFlag_current;
					data_global.temp_errorFlag_current = true;

				}
				else 
				{

					data_global.leftTemp_previous = data_global.leftTemp_current;

					// Parse Temperature Magnitude	
					String ia = line.substring(ACEDATA_Temp1_1000X_POS,ACEDATA_Temp1_1X_POS+1);
									
					data_global.leftTemp_current= ((float)atoi(ia.c_str()))/10.0f;
					
					if(data_global.leftTempSign == TempSignNeg){
						data_global.leftTemp_current= data_global.leftTemp_current* -1;
					}

					data_global.leftTempError_previous = data_global.leftTempError_current;
					data_global.leftTempError_current= false;

				}

				// //MONITOR.print("Left Temp Value: ");
				// //MONITOR.println(data_global.leftTemp);

				//================================================== Right Temp =========================================*/
				////MONITOR.printf("Right Temp Sign: %c\n",line.charAt(ACEDATA_Temp2_Sign_POS));
				data_global.rightTempSign = (line.charAt(ACEDATA_Temp2_Sign_POS));
				
				// Check for Error First
				if (data_global.rightTempSign == TempErrorGeneral){
							
					if (line.charAt(ACEDATA_Temp2_10X_POS) == TempErrorOpen || line.charAt(ACEDATA_Temp2_10X_POS) == TempErrorSho){
						data_global.rightTempSign = line.charAt(ACEDATA_Temp2_10X_POS);
					}
					
					data_global.rightTempError_previous = data_global.rightTempError_current;
					data_global.rightTempError_current = true;
					data_global.rightTempSign = data_global.rightTempSign;
					data_global.temp_errorFlag_current = true;

				}
				else 
				{
					
					data_global.rightTemp_previous = data_global.rightTemp_current;

					// Parse Temperature Magnitude	
					String ia =line.substring(ACEDATA_Temp2_1000X_POS,ACEDATA_Temp2_1X_POS+1);
				
					data_global.rightTemp_current = ((float)atoi(ia.c_str()))/10.0f;
					
					if(data_global.rightTempSign == TempSignNeg){
						data_global.rightTemp_current = data_global.rightTemp_current * -1;
					}

					data_global.rightTempError_previous = data_global.rightTempError_current;
					data_global.rightTempError_current = false;

				}

				////MONITOR.print("Right Temp Value: ");
				////MONITOR.println(data_global.rightTemp);

				//================================================== Average Temp =========================================*/

				// If Both Sensors have not returned an error
				if (data_global.leftTempError_current== false && data_global.rightTempError_current == false){
					
					data_global.avgTemp = (data_global.leftTemp_current+ data_global.rightTemp_current)/2;
				}
				else
				{
					// If a single sensor is failed, used the working sensor value as the average
					if (data_global.leftTempError_current== true && data_global.rightTempError_current == false)
					{
						data_global.avgTemp = data_global.rightTemp_current;
					}
					else if (data_global.leftTempError_current== false && data_global.rightTempError_current == true)
					{
						data_global.avgTemp = data_global.leftTemp_current;
					}
				}

				////MONITOR.print("Average Temp Value: ");
				////MONITOR.println(data_global.avgTemp);


				if (data_global.leftTemp_previous != data_global.leftTemp_current || data_global.rightTemp_previous != data_global.rightTemp_current){
					data_global.temp_changed = true;
					data_global.updateIcons = true;
				}


				//================================================== Battery Value =========================================*/

				data_global.batt_error_previous = data_global.batt_error_current;

				String ia =line.substring(ACEDATA_Batt_100X_POS,ACEDATA_Batt_1X_POS+1);
			
				data_global.voltage = ((float)atoi(ia.c_str()))/10.0f;
			
				// //MONITOR.print("Battery Voltage: ");
				// //MONITOR.println(battvalues_current.voltage);

				// Check if the Batt Voltage is out of range
				if (data_global.voltage < BadBatteryThreshold)
				{
					data_global.BadBatteryCounter++;
					if (data_global.BadBatteryCounter > MaxBadBattValCounter)
					{
						data_global.batt_error_current = true;
					}
				}
				else 
				{
					data_global.BadBatteryCounter = 0;
					data_global.batt_error_current = false;
				}

				// Everytime this loops and the batt error flag is true, increment count. if not clear the counter
				if (data_global.batt_error_current == true)
				{
					// increment counter
					data_global.BadBatteryCounter++;

					// Check the Counter
					if (data_global.BadBatteryCounter < MaxBadBattValCounter)
					{
						// Counter has not gone over yet, the system has not had enough time to truly process the battery state

						// Manually clear the Batt Error Flag and do not update the icons
						data_global.batt_error_current = false;
					}
					else
					{
						// Don't allow the counter to go over the max
						data_global.BadBatteryCounter = 0;

						// Battery Value has been "bad" for sufficient time to be valid

						//CurrentDebug_Priority = Debug_Priority::High;
						//MONITOR.println("Setting Pre Alert Text: LO BATT");

					}

				}
				else{

				}


				if (data_global.batt_error_previous != data_global.batt_error_current){
					data_global.battchanged = true;
					data_global.updateIcons = true;
				}


				//================================================== Engine Value =========================================*/

				data_global.engineStalled_previous = data_global.engineStalled_current;

				if (line.charAt(ACEDATA_Stall_Status_POS)=='0'){
					////MONITOR.println("Engine Stalled");
					data_global.engineStalled_current = true;
				}else{
					////MONITOR.println("Engine NOT Stalled");
					data_global.engineStalled_current = false;
				}

					ia =line.substring(ACEDATA_Stall_Count_10X_POS,ACEDATA_Stall_Count_1X_POS+1);
				
					data_global.enginestallcount = ((float)atoi(ia.c_str()))/10.0f;
				
				////MONITOR.print("Engine Stall Count:");
				////MONITOR.println(enginevalues_current.engineStallCount);


				if (line.charAt(ACEDATA_Stall_Sensor_Present_POS) =='A'){
					////MONITOR.println("Engine Stall Sensor Present");
					data_global.engineStallSensorPresent = true;
				}else{
					////MONITOR.println("Engine Stall Sensor Not Present");
					data_global.engineStallSensorPresent = false;
				}

				if (data_global.enginestallcount >= EngineStallThreshold){
					////MONITOR.println("Stall Condition Achieved");
					data_global.engineStalled_current = true;
					
				}else{
					data_global.engineStalled_current = false;
				}

				if (data_global.engineStalled_previous != data_global.engineStalled_current){
					////MONITOR.println("Engine Stalled Valued Changed ");
					data_global.engine_changed = true;
					data_global.updateIcons = true;
				}


				//================================================== Aux Value =========================================*/

				
				
				if(data_global.AuxEnabled){

					data_global.Aux1Input_previous = data_global.Aux1Input_current;

					if (line.charAt(ACEDATA_Aux1_Input_POS)=='1'){
						data_global.Aux1Input_current = true;
					}else{
						data_global.Aux1Input_current = false;
					}

					data_global.Aux2Input_previous = data_global.Aux2Input_current;

					if (line.charAt(ACEDATA_Aux2_Input_POS)=='1'){
						data_global.Aux2Input_current = true;
					}else{
						data_global.Aux2Input_current = false;
					}

				}

				
				



				if (data_global.Aux1Input_previous!=data_global.Aux1Input_current || data_global.Aux2Input_previous!=data_global.Aux2Input_current){
					data_global.aux_changed = true;
					data_global.updateIcons = true;
				}
				
				
				const char* str = line.substring(ACEDATA_VIM_SN_POS,ACEDATA_VIM_SN_POS+ACEDATA_VIM_SN_LENGTH).c_str();
				//================================================== Serial Number Value =========================================*/
				strncpy(data_global.VIMSerialNumber,str,sizeof(data_global.VIMSerialNumber));

        
				
				vim_store(data_global);
				
	
                
               

            }

            if(line.substring(index,length)=="$ACEK9,IH2") {
                ////MONITOR.printf("Parsed line: %s\n",line.c_str());
                
            }

            if(line.substring(index,length)=="$ACEK9,IP1"){
                ////MONITOR.printf("Parsed line: %s\n",line.c_str());
                vim_write_sz("$ACEK9,CH1A,C502E5A63G-DEV0103B2C502E5A63GDADA00\r\n");

            } 

            newlineIndex = trickle.indexOf("\n");
        
        }
    
    }
      

    //vim_write_sz()
}

void acecon_dev_tick() {
    data_global = vim_load();
    //================================================== Read ACECON Inputs =========================================*/
    aceconvalues_previous = aceconvalues_current;
    
	
	aceconvalues_current.ppt = digitalRead(ACECON_POP_IN);
	aceconvalues_current.alm = digitalRead( ACECON_ALM_OUT);

	aceconvalues_current.pps = digitalRead(ACECON_PPS_IN);
	//system is in gear
	if (aceconvalues_current.pps == INGEAR)
	{

		// Check if a screen update is needed
		if (data_global.inGear == PARKED)
		{
			// Set Update Flags
			data_global.engine_changed = true;
		}

		// Set to In Gear
		data_global.inGear = INGEAR;

		// Enable the clear timer
		Clear_Gear_Timer.StartTimer(Clear_Gear_Timer.Threshold);

	}

	if (Clear_Gear_Timer.OverFlowFlag == true || Clear_Gear_Timer.TimerEnable == false)
	{
		// Check if a screen update is needed
		if (data_global.inGear == INGEAR)
		{
			// Set Update Flags
			data_global.engine_changed = true;
		}

		// Stop Timer
		Clear_Gear_Timer.StopTimer();

		// Set to In Park
		data_global.inGear = PARKED;

	}
	
	
	
	aceconvalues_current.ign = digitalRead(ACECON_IGN_IN);
	// Rising Edge Detected
	if (aceconvalues_previous.ign == false && aceconvalues_current.ign == true)
	{
		data_global.ignitionEdge_current = IgnEdge::it_Rising;
		MONITOR.println("Ignition Rising Edge Detected");
	}
	// Falling Edge Detected
	else if (aceconvalues_previous.ign == true && aceconvalues_current.ign == false)
	{
		
		data_global.ignitionEdge_current = IgnEdge::it_Falling;
		MONITOR.println("Ignition Falling Edge Detected");
	}
	// No change
	else
	{
		
	}
	

    if(aceconvalues_previous.ppt != aceconvalues_current.ppt) {
        //MONITOR.printf("PPT Val Changed to  %s\r\n",aceconvalues_current.ppt?"HIGH":"LOW");
        aceconvalues_current.valueChanged = true;

        if(aceconvalues_current.ppt) {
            lv_obj_add_state(ui_SwitchPPT, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchPPT, LV_STATE_CHECKED);
        }
    }
    if(aceconvalues_previous.pps != aceconvalues_current.pps) {
        //MONITOR.printf("PPS Val Changed to  %s\r\n",aceconvalues_current.pps?"HIGH":"LOW");
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
        data_global.ignitionOn_previous = data_global.ignitionOn_current;
        data_global.ignitionOn_current = aceconvalues_current.ign;
        MONITOR.printf("IGN Val Changed to  %s\r\n",data_global.ignitionOn_current?"HIGH":"LOW");
        data_global.engine_changed = true; 
        aceconvalues_current.valueChanged = true;
       
    }
    if(aceconvalues_previous.hps != aceconvalues_current.hps) {
        //MONITOR.printf("HPS Val Changed to  %s\r\n",aceconvalues_current.hps?"HIGH":"LOW");
        aceconvalues_current.valueChanged = true;

        if(aceconvalues_current.hps) {
            lv_obj_add_state(ui_SwitchHPS, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchHPS, LV_STATE_CHECKED);
        }
    }
    if(aceconvalues_previous.alm != aceconvalues_current.alm) {
        //MONITOR.printf("ALM Val Changed to  %s\r\n",aceconvalues_current.alm?"HIGH":"LOW");
        aceconvalues_current.valueChanged = true;

        if(aceconvalues_current.alm) {
            lv_obj_add_state(ui_SwitchALM, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(ui_SwitchALM, LV_STATE_CHECKED);
        }
    }


  vim_store(data_global);  
    

}

void check_door_condition(){

	data_global = vim_load();
		
    // TODO: fill in Door POPPED and DOOR Disabled condition checks
    // Place Holder Values
    systemsettings_current.doorDisabled = false;
    data_global.k9_door_popped = false;

	vim_store(data_global);
}

void check_cell_signal(){
    
    if (xbee_cell_connected && (millis() - signal_strength_timer > signal_strength_timer_threshold )){
        signal_strength_timer = millis();
        
        //MONITOR.println("Checking Cell Signal");
        String str = "atdb";
        //MONITOR.println("Sending:" + str);
        
        on_monitor_at(str.c_str()); 
    }

    if (last_at_cmd.commandstr[0] == 'd' && last_at_cmd.commandstr[1] == 'b' && last_at_cmd.value_received){
        //MONITOR.println("Received db Response");
        if (xbee_signal_strength_current != last_at_cmd.value){
            xbee_signal_strength_changed = true;
        }
        xbee_signal_strength_current = last_at_cmd.value;
        
        last_at_cmd.value_received = false;

    }

}

void Determine_HPS_PPS(SystemState state)
{
	if (state.Index == C1_HA_DP_State.Index)
	{
		set_HPS(HIGH);
		set_PPS(HIGH);
	}
	else if (state.Index == C2_HA_ONLY_State.Index)
	{
		set_HPS(HIGH);
		set_PPS(LOW);
	}
	else if (state.Index == C3_DP_ONLY_State.Index)
	{
		set_HPS(LOW);
		set_PPS(HIGH);
	}
	else if (state.Index == D10_PowerDownByPowerPress_State.Index)
	{
		set_HPS(HIGH);
	}
	else if (state.Index == D1_NOK9LeftBehind_State.Index)
	{
		set_HPS(HIGH);
	}
	else if (state.Index == E1_VIMCommunicationsError_State.Index)
	{
		set_HPS(HIGH);
	}


}

boolean Check_Comms()
{

	// Only check Comms if HPS is set. otherwise COMs are not needed therefore no need to check them.

	if (HeatAlarmEnabled == true)
	{
		if (data_global.Comms_OK == true)
		{
			// Restart the Timer
			COMError_Timer.StartTimer(Comm_Error_Timeout);

			// Clear COMMS flag. SerialEvent will reset this flag when sucessful comms occur again
			data_global.Comms_OK = false;

		}
		else if (COMError_Timer.OverFlowFlag == true)
		{

			return false;

		}
	}
	else
	{
		// COMMs are not needed. 



	}


	return true;

}

void Process_State_Machine(){

	data_global = vim_load();

	if (A0_Off_State.Index == systemstate_current.Index)
	{
		/*
		The A0-OFF state is the initial state that the the system is in when it is unpowered. No code operates in this state as a result.
		Once power is applied the system will move on to the power applied state.
		*/

		// First Time Section
		if (systemstate_previous.Index != systemstate_current.Index)
		{
            MONITOR.println("Entered State: A0_Off_State");

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Set Initial Power Up Flag
			systemsettings_current.InitialPowerUpFlag = true;
			//MONITOR.println("InitialPowerUpFlag: TRUE");

			// Draw Initial Screen Details
			lv_scr_load(ui_PowerOffScreen);
			//MONITOR.println("Screen Loaded");
			lv_textarea_set_text(ui_PowerOffTextArea,"A0 Power Off State");
			//MONITOR.println("Text Loaded");

			// Start Update Timer
			A0_Off_Timer.StartTimer(A0_Off_Timer.Threshold);
			//MONITOR.println("Timer Started");

			// Notification Beep
			//Set_SoundPulse(PulseProfile::pp_SingleBeep);

			// Normalize State
			systemstate_previous = systemstate_current;
        }

		// Looping Section
		else
		{
			//MONITOR.println("Entered State: A0_Off_State LOOPING");
				if (A0_Off_Timer.OverFlowFlag == true)
				{
					// Reset Timer
					A0_Off_Timer.StopTimer();
					//MONITOR.println("A0 Timer Overflow");
					
					// Power Applied to System - Go to A1
					Set_Next_State(A1_PowerApplied_State);
					//MONITOR.println("Setting Next State");

				}
		}

	}
	else if (A1_PowerApplied_State.Index == systemstate_current.Index)
	{
		/*
		The A1 - Powered Applied State notifies the user that the system is powering on.
		It also reads the EEPROM and stores those values so that the system can operate in accordance with the user defined settings.
		*/

		// First Time Section
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: A1_PowerApplied_State");

			// Store Power On Trigger
			CurrentPowerOnTrigger = PowerOnTrigger::Applied;

			// ================== Display =======================
			lv_scr_load(ui_PowerAppliedScreen);
			lv_label_set_text(ui_LabelPowerAppliedScreen,"Power Applied To Control Head");
			

			// =================== Sound ======================== 

			// Emit Power Up Sound
			//Set_SoundPulse(pp_PowerUp);

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Set Initial Power Up Flag
			systemsettings_current.InitialPowerUpFlag = true;
			//MONITOR.println("InitialPowerUpFlag: TRUE");

			// Normalize State
			systemstate_previous = systemstate_current;

			// Enable the Timer for the transition
			A1_PowerApplied_Timer.StartTimer(1500);


		}
		// Looping Section
		else
		{
			// Check Timer
			if (A1_PowerApplied_Timer.OverFlowFlag == true)
			{
				// Stop Timer
				A1_PowerApplied_Timer.StopTimer();

				// ================== Transitions =======================


					// switch (CurrentPowerOnTrigger)
					// {
					// case PowerOnTrigger::NoTrigger:
					// 	MONITOR.println("===== PowerOnTrigger: No Trigger");
					// 	break;
					// case PowerOnTrigger::IgnitionOn:
					// 	MONITOR.println("===== PowerOnTrigger: Ignition On");
					// 	break;
					// case PowerOnTrigger::Applied:
					// 	MONITOR.println("===== PowerOnTrigger: Applied");
					// 	break;
					// case PowerOnTrigger::PowerButtonPress:
					// 	MONITOR.println("===== PowerOnTrigger: Power Button Press");
					// 	break;
					
					// default:
					// 	MONITOR.println("===== PowerOnTrigger: Unknown");
					// 	break;
					// }		

				Set_Next_State(B1_MenuHelp_State);

				

			}

		}



	}
	else if (A3_IgnitionOn_State.Index == systemstate_current.Index)
	{
		/*
		The A3 - Ignition On State notifies the user that the system is powering on via The ignition being on.
		It also reads the EEPROM and stores those values so that the system can operate in accordance with the user defined settings.
		*/

		// First Time Section
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: A3_IgnitionOn_State");

			// Store Power On Trigger
			CurrentPowerOnTrigger = PowerOnTrigger::Applied;

			// ================== Display =======================
			lv_scr_load(ui_PowerAppliedScreen);
			lv_label_set_text(ui_LabelPowerAppliedScreen,"Power On by Ignition");


			// =================== Sound ======================== 

			// Emit Power Up Sound
			//Set_SoundPulse(pp_PowerUp);

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Set Initial Power Up Flag
			systemsettings_current.InitialPowerUpFlag = true;
			//MONITOR.println("InitialPowerUpFlag: TRUE");

			// Normalize State
			systemstate_previous = systemstate_current;

			// Enable the Timer for the transition
			A1_PowerApplied_Timer.StartTimer(1500);


		}
		// Looping Section
		else
		{
			// Check Timer
			if (A1_PowerApplied_Timer.OverFlowFlag == true)
			{
				// Stop Timer
				A1_PowerApplied_Timer.StopTimer();

				// ================== Transitions =======================

				Set_Next_State(B1_MenuHelp_State);

			}

		}


	}
	else if (B1_MenuHelp_State.Index == systemstate_current.Index)
	{	
		
		if (systemstate_previous.Index != systemstate_current.Index)
		{	
			MONITOR.println("Entered State: B1_MenuHelp_State");

			// ================== Display =======================
			lv_scr_load(ui_MenuHelpScreen);

			//MONITOR.println("Determine Next State: B1_MenuHelp_State");
			// ================== Operation =======================
			Determine_HPS_PPS(Determine_Next_State());

			//MONITOR.println("Starting Timer: B1_MenuHelp_State");
			// Enable the Timers
			B1_MenuHelp_Timer.StartTimer(3000);

			// Reset NoK9Timeout Flag
			NoK9TimeoutFlag = false;

			// VIM Communications Timer
			COMError_Timer.StartTimer(Comm_Error_Timeout);

			// Enable Initial Power Up Timer
			InitialPowerUp_Timer.StartTimer(InitialPowerUp_Timer.Threshold);

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			//MONITOR.println("Normalizing State: B1_MenuHelp_State");
			// Normalize State
			systemstate_previous= systemstate_current;
			//MONITOR.println("Exiting State Initial Loop");

		}
		else
		{
			// ================== Transitions =======================
			//MONITOR.println("Looping State: B1_MenuHelp_State");
			
			// Clear Initial Power Up Flag 
			if (systemsettings_current.InitialPowerUpFlag == true)
			{
				if (InitialPowerUp_Timer.CheckOverflow())
				{
					InitialPowerUp_Timer.OverFlowFlag = false;

					MONITOR.println("InitialPowerUpFlag: FALSE");

					// Only Update after the Flag has Overflowed. to ensure the system has had enough time to stabilize.
					systemsettings_current.InitialPowerUpFlag = false;
				}

			}

			// Communications error detected
			if (Check_Comms() == false)
			{
				// Stop Timers
				COMError_Timer.StopTimer();
				B1_MenuHelp_Timer.StopTimer();

				// Go To E1 VIM Communications Error
				Set_Next_State(E1_VIMCommunicationsError_State);

			}
			// Menu Help State Timer Overflowed
			else if (B1_MenuHelp_Timer.OverFlowFlag == true)
			{
				MONITOR.println("Leaving State: B1_MenuHelp_State");
				// Stop Timers
				COMError_Timer.StopTimer();
				B1_MenuHelp_Timer.StopTimer();

				Set_Determined_State();

			}

		}


	}
	else if (C1_HA_DP_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: C1_HA_DP_State");

			// Assign HPs and PPS as necessary
			set_HPS(HIGH);
			set_PPS(HIGH);

			// ================== Display =======================
			if (!GoToMenuFlag){
				lv_scr_load(ui_OperationScreen);
			}
			

			// Enable Comm Error Timer if not allready enabled
			if (E1_VIMCommunicationsError_Timer.TimerEnable == false)
			{
				COMError_Timer.StartTimer(Comm_Error_Timeout);
			}

			// Set Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = true;



			// Normalize State
			systemstate_previous = systemstate_current;

		}
		else
		{
			
			// Communications error detected
			if (Check_Comms() == false)
			{
				// Stop Timers
				COMError_Timer.StopTimer();

				// Go To E1 VIM Communications Error
				Set_Next_State(E1_VIMCommunicationsError_State);

			}

			// Ignition is OFF
			if (data_global.ignitionOn_current == false)
			{
				if (systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF)
				{
					if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
					{
						Set_Next_State(D8_PowerDownByIgnitionOFF_State);
					}
					else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left && NoK9TimeoutFlag == false)
					{
						Set_Next_State(D1_NOK9LeftBehind_State);
					}
					else if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF ||
						systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
					{
						// CODE EXCEPTION! NEEDED TO PREVENT ISSUE #2 for Rev 019
						// Prevents the door icon from briefly displaying as red for this transition.
						data_global.inGear = PARKED;
						

						Set_Next_State(C2_HA_ONLY_State);
					}


				}
				else if (systemsettings_current.DoorPower == DoorOpt::d_CarONManOFF)
				{
					if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
					{
						Set_Next_State(C3_DP_ONLY_State);
					}
					else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left && NoK9TimeoutFlag == false)
					{
						Set_Next_State(D1_NOK9LeftBehind_State);
					}

				}
				else if (systemsettings_current.DoorPower == DoorOpt::d_ManONManOFF)
				{
					if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
					{
						Set_Next_State(C3_DP_ONLY_State);
					}
					else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left && NoK9TimeoutFlag == false)
					{
						Set_Next_State(D1_NOK9LeftBehind_State);
					}

				}



			}

			// Ignition is ON
			if (data_global.ignitionOn_current == true)
			{

				// Reset NoK9 Timeout Flag
				NoK9TimeoutFlag = false;
			}


		}

	}
	else if (C2_HA_ONLY_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: C2_HAOnly_State");

			// Assign HPs and PPS as necessary
			set_HPS(HIGH);
			set_PPS(LOW);
			

			// ================== Display =======================
			if (!GoToMenuFlag){
				lv_scr_load(ui_OperationScreen);
			}

			// Enable Comm Error Timer if not allready enabled
			if (E1_VIMCommunicationsError_Timer.TimerEnable == false)
			{
				COMError_Timer.StartTimer(Comm_Error_Timeout);
			}

			// Enable Initial Power Up Timer
			InitialPowerUp_Timer.StartTimer(InitialPowerUp_Timer.Threshold);

			// Set Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = true;


			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{

			
			// Communications error detected
			if (Check_Comms() == false)
			{
				// Stop Timers
				COMError_Timer.StopTimer();

				// Go To E1 VIM Communications Error
				Set_Next_State(E1_VIMCommunicationsError_State);

			}

			// Ignition is OFF
			if (data_global.ignitionOn_current == false)
			{
				if (systemsettings_current.DoorPower == DoorOpt::d_OFF)
				{
					if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF)
					{
						Set_Next_State(D8_PowerDownByIgnitionOFF_State);
					}
					else if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left && NoK9TimeoutFlag == false)
					{
						Set_Next_State(D1_NOK9LeftBehind_State);
					}
				}
			}

			if (data_global.ignitionOn_current == true)
			{

				// Reset NoK9 Timeout Flag
				NoK9TimeoutFlag = false;

				if (systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF)
				{
					if (systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_ManONManOFF)
					{
						// CODE EXCEPTION! NEEDED TO PREVENT ISSUE #2 for Rev 019
						// PRevents the door icon from briefly displaying as red for this transition.
						data_global.inGear = PARKED;
						

						Set_Next_State(C1_HA_DP_State);
					}
				}
			}


		}

	}
	else if (C3_DP_ONLY_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: C3_DP_ONLY_State");

			// Assign HPs and PPS as necessary
			set_HPS(LOW);
			set_PPS(HIGH);
			
			// ================== Display =======================
			if (!GoToMenuFlag){
				lv_scr_load(ui_OperationScreen);
			}

			// Enable Comm Error Timer if not allready enabled
			if (E1_VIMCommunicationsError_Timer.TimerEnable == false)
			{
				COMError_Timer.StartTimer(Comm_Error_Timeout);
			}

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;



			// Normalize State
			systemstate_previous = systemstate_current;

		}
		else
		{

			// Ignition falling edge detected
			if (data_global.ignitionEdge_current == IgnEdge::it_Falling)
			{
				data_global.ignitionEdge_previous = data_global.ignitionEdge_current;
				data_global.ignitionEdge_current = IgnEdge::it_None;

				if (systemsettings_current.AlarmPower == PowerOpt::p_OFF)
				{
					if (systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF)
					{
						Set_Next_State(D8_PowerDownByIgnitionOFF_State);
					}


				}
			}
			else if (data_global.ignitionEdge_current == IgnEdge::it_Rising)
			{
				data_global.ignitionEdge_previous = data_global.ignitionEdge_current;
				data_global.ignitionEdge_current = IgnEdge::it_None;

				if (systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF || systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					if (systemsettings_current.DoorPower == DoorOpt::d_CarONManOFF || systemsettings_current.DoorPower == DoorOpt::d_ManONManOFF)
					{
						Set_Next_State(C1_HA_DP_State);
					}
				}
			}


		}

	}
	else if (D1_NOK9LeftBehind_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: D1_NoK9LeftBehind_State");

			// ================== Display =======================
			lv_scr_load(ui_NoK9Screen);

			// Enable Comm Error Timer if not allready enabled
			if (E1_VIMCommunicationsError_Timer.TimerEnable == false)
			{
				COMError_Timer.StartTimer(Comm_Error_Timeout);
			}

			// Initialize Timers
			NoK9Blink_Timer.StartTimer(NoK9Blink_Timer.Threshold);
			NoK9Beep_Timer.StartTimer(NoK9Beep_Timer.Threshold);
			NoK9FirstAlert_Timer.StartTimer(NoK9FirstAlert_Timer.Threshold);
			NoK9SecondAlert_Timer.StartTimer(NoK9SecondAlert_Timer.Threshold);

			TootCount = 0;

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{

			// Communications error detected
			if (Check_Comms() == false)
			{
				// Stop Timers
				COMError_Timer.StopTimer();

				// Go To E1 VIM Communications Error
				Set_Next_State(E1_VIMCommunicationsError_State);

			}
			

			//Ignition turned on
			if (data_global.ignitionOn_current == true)
			{
				if (systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left)
				{
					if (systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF || systemsettings_current.DoorPower == DoorOpt::d_CarONManOFF)
					{
						Set_Next_State(C1_HA_DP_State);
					}
					else
					{
						Set_Next_State(C2_HA_ONLY_State);
					}
				}
			}

			// Door Opened
			if (data_global.k9_door_open_current)
			{
				Set_Next_State(D6_NoK9LeftBehindPowerDownByDoorOpened_State);

			}

			// Blink the "Remove K9" Text
			if (NoK9Blink_Timer.OverFlowFlag == true)
			{
				

			}

			// Beep Twice
			if (NoK9Beep_Timer.OverFlowFlag == true)
			{
				// Beep Twice
				//Set_SoundPulse(PulseProfile::pp_TripleBeep);

				// Reset Timer
				NoK9Beep_Timer.StartTimer(NoK9Beep_Timer.Threshold);

			}

			if (NoK9FirstAlert_Timer.OverFlowFlag == true)
			{
				// Send Toot Trigger
				TX_Toot_Needed = true;

				TootCount = 1;

				// Stop Timer
				NoK9FirstAlert_Timer.StopTimer();
			}

			if (NoK9SecondAlert_Timer.OverFlowFlag == true)
			{
				// Send Toot Trigger
				TX_Toot_Needed = true;

				TootCount = 2;

				// Stop All Timers
				NoK9Blink_Timer.StopTimer();
				NoK9Beep_Timer.StopTimer();
				NoK9FirstAlert_Timer.StopTimer();
				NoK9SecondAlert_Timer.StopTimer();

			}

			// Wait for Toot to occur before leaving
			if (TX_Toot_Needed == false && TootCount == 2)
			{

				if (systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF || systemsettings_current.DoorPower == DoorOpt::d_CarONManOFF)
				{
					// Set TimeoutFlag to prevent C1/C2 Flash Rev 029
					NoK9TimeoutFlag = true;

					Set_Next_State(C1_HA_DP_State);
				}
				else
				{
					// Set TimeoutFlag to prevent C1/C2 Flash Rev 029
					NoK9TimeoutFlag = true;

					Set_Next_State(C2_HA_ONLY_State);
				}
			}






		}

	}
	else if (D6_NoK9LeftBehindPowerDownByDoorOpened_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: D6_NoK9LeftBehindPowerDownByDoorOpened_State");

			// ================== Display =======================
			lv_scr_load(ui_PowerDownScreen);
			lv_label_set_text(ui_LabelPowerDownScreen,"D6_NoK9LeftBehind PowerDown By Door Opened");

			// Start Timer
			D6_NoK9LeftBehindPowerDownByDoorOpened_Timer.StartTimer(4000);


			//MONITOR.println("No K9 Timeout Flag Set To FALSE");
			// Reset NoK9Timeout Flag
			NoK9TimeoutFlag = false;

			// Emit Power Down Sound
			//Set_SoundPulse(pp_PowerDown);

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{

			// Timer Overflowed
			if (D6_NoK9LeftBehindPowerDownByDoorOpened_Timer.OverFlowFlag == true)
			{
				// Stop Timers
				D6_NoK9LeftBehindPowerDownByDoorOpened_Timer.StopTimer();

				Set_Next_State(S1_Sleep_State);

			}

		}

	}
	else if (D7_PowerDownByOKPress_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: D7_PowerDownByOKPress_State");

			// ================== Display =======================
			lv_scr_load(ui_PowerDownScreen);
			lv_label_set_text(ui_LabelPowerDownScreen,"D7_PowerDownByOKPress");

			// Start Timer
			D7_PowerDownByOKPress_Timer.StartTimer(4000);


			//MONITOR.println("No K9 Timeout Flag Set To FALSE");
			// Reset NoK9Timeout Flag
			NoK9TimeoutFlag = false;

			// Emit Power Down Sound
			//Set_SoundPulse(pp_PowerDown);

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{

			// Timer Overflowed
			if (D7_PowerDownByOKPress_Timer.OverFlowFlag == true)
			{
				// Stop Timers
				D7_PowerDownByOKPress_Timer.StopTimer();

				Set_Next_State(S1_Sleep_State);

			}

		}

	}
	else if (D8_PowerDownByIgnitionOFF_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: D8_PowerDownByIgnitionOFF_State");

			// ================== Display =======================
			lv_scr_load(ui_PowerDownScreen);
			lv_label_set_text(ui_LabelPowerDownScreen,"D8_PowerDownByIgnitionOFF");

			// Start Timer
			D8_PowerDownByIgnitionOFF_Timer.StartTimer(4000);


			//MONITOR.println("No K9 Timeout Flag Set To FALSE");
			// Reset NoK9Timeout Flag
			NoK9TimeoutFlag = false;

			// Emit Power Down Sound
			//Set_SoundPulse(pp_PowerDown);

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{
			// Timer Overflowed
			if (D8_PowerDownByIgnitionOFF_Timer.OverFlowFlag == true)
			{
				// Stop Timers
				D8_PowerDownByIgnitionOFF_Timer.StopTimer();

				Set_Next_State(S1_Sleep_State);

			}
		}

	}
	else if (D9_PowerDownByHAandDPSetToAlwaysOFF_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: D9_PowerDownByHAandDPSetToAlwaysOFF_State");

			// ================== Display =======================
			lv_scr_load(ui_PowerDownScreen);
			lv_label_set_text(ui_LabelPowerDownScreen,"D9_PowerDownByHAandDPSetToAlwaysOFF");

			// Start Timer
			D9_PowerDownByHAandDPSetToAlwaysOFF_Timer.StartTimer(4000);


			//MONITOR.println("No K9 Timeout Flag Set To FALSE");
			// Reset NoK9Timeout Flag
			NoK9TimeoutFlag = false;

			// Emit Power Down Sound
			//Set_SoundPulse(pp_PowerDown);

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{
			// Timer Overflowed
			if (D9_PowerDownByHAandDPSetToAlwaysOFF_Timer.OverFlowFlag == true)
			{
				// Stop Timers
				D9_PowerDownByHAandDPSetToAlwaysOFF_Timer.StopTimer();

				Set_Next_State(S1_Sleep_State);

			}
		}

	}
	else if (D10_PowerDownByPowerPress_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: D10 Power Down By Power Press");

			// ================== Display =======================
			lv_scr_load(ui_PowerDownScreen);
			lv_label_set_text(ui_LabelPowerDownScreen,"D10 Power Down By Power Press");

			// Start Timer
			D10_PowerDownByPowerPress_Timer.StartTimer(4000);


			//MONITOR.println("No K9 Timeout Flag Set To FALSE");
			// Reset NoK9Timeout Flag
			NoK9TimeoutFlag = false;

			// Emit Power Down Sound
			//Set_SoundPulse(pp_PowerDown);

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{
			// Timer Overflowed
			if (D10_PowerDownByPowerPress_Timer.OverFlowFlag == true)
			{
				// Stop Timers
				D10_PowerDownByPowerPress_Timer.StopTimer();

				Set_Next_State(S1_Sleep_State);

			}
		}

	}
	else if (E3_Unknown_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: E3_Unknown_State");

			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{

		}

	}
	else if (S1_Sleep_State.Index == systemstate_current.Index)
	{
		if (systemstate_previous.Index != systemstate_current.Index)
		{
			MONITOR.println("Entered State: S1_Sleep_State");

			lv_scr_load(ui_SleepScreen);
			lv_textarea_set_text(ui_SleepTextArea,"S1_Sleep");

			// Clear Alarm Enabled Flag
			Reset_System_Alarm_State();
			AlarmStateEnabled = false;

			// Clear HPS and PPS
			set_HPS(LOW);
			set_PPS(LOW);

			

			// Normalize State
			systemstate_previous = systemstate_current;
		}
		else
		{
			

			// Ignition Rising Edge Detected
			if (data_global.ignitionEdge_current == IgnEdge::it_Rising &&
				(systemsettings_current.AlarmPower == PowerOpt::p_CarONManOFF || systemsettings_current.AlarmPower == PowerOpt::p_CarONCarOFF || systemsettings_current.AlarmPower == PowerOpt::p_NoK9Left ||
					systemsettings_current.DoorPower == DoorOpt::d_CarONCarOFF || systemsettings_current.DoorPower == DoorOpt::d_CarONManOFF))
			{
				Set_Next_State(A3_IgnitionOn_State);
			}
			

		}

	}
	
	vim_store(data_global);
}

bool Check_ACK()
{
	last_packet.processed = true;

	if (last_packet.cmd == COMMAND_ID::ACKNOWLEDGE)
	{
		MONITOR.println("Check ACK Received");
		
		if (last_packet.status == STATUS_CODE::SUCCESS)
		{

			 MONITOR.println("Check ACK Returned True");

			last_packet.cmd = (COMMAND_ID)NULL;
			last_packet.status = (STATUS_CODE)NULL;

			return true;
		}
	}
	MONITOR.println("Check ACK Returned False");
	return false;
}

bool Check_FOTA_FW()
{
	last_packet.processed = true;

	if (last_packet.cmd == COMMAND_ID::FOTA)
	{

		if (last_packet.status == STATUS_CODE::SUCCESS)
		{

			// MONITOR.println("XBEE Cell Connected");

			last_packet.cmd = (COMMAND_ID)NULL;
			last_packet.status = (STATUS_CODE)NULL;

			return true;
		}
	}

	return false;
}

bool Check_FOTA_DOWNLOAD_DONE()
{
	last_packet.processed = true;

	if (last_packet.cmd == COMMAND_ID::UPDATE && update_data.size == 0)
	{
		// MONITOR.println("XBEE Cell Connected");

			last_packet.cmd = (COMMAND_ID)NULL;
			last_packet.status = (STATUS_CODE)NULL;

			return true;
	}

	return false;
}

void FOTA_Loop(){

	if (CurrentFOTACode != FOTA_Done)
	{
		switch (CurrentFOTACode)
		{
		case FOTA_None:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA None");
				PreviousFOTACode = CurrentFOTACode;
			}else
			{
				
			}

			break;

		case FOTA_Begin:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA Begin");
				PreviousFOTACode = CurrentFOTACode;
			}
			else
			{

				// Move on to CheckFW
				
				
				PreviousFOTACode = CurrentFOTACode;
				CurrentFOTACode = FOTA_CheckFW;
			}

			break;

		case FOTA_CheckFW:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA CheckFW");

				String str = "";
              	on_monitor_check_fota(str.c_str());

				PreviousFOTACode = CurrentFOTACode;
			}
			else
			{
				if (last_packet.processed == true){

					// WAIT FOR PACKET
					// TODO: ADD IN TIMEOUT CODE
					MONITOR.println("FOTA CheckFW: Waiting for Packet");

				}else{
					
					if (Check_ACK())
					{						

						#ifdef FOTA_MANUAL_CONTROL
							MONITOR.println("FOTA MANUAL CONTROL ENABLED: 'FOTA Next' to Continue");
						#else
							
							PreviousFOTACode = CurrentFOTACode;
							CurrentFOTACode = FOTA_Initiate;
						#endif
						
					}else{
							MONITOR.println("FOTA CheckFW:CheckACK: false");
							
							PreviousFOTACode = CurrentFOTACode;
							CurrentFOTACode = FOTA_Fail;
							
					}
				}
				
			}

			break;

		case FOTA_Initiate:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA Initiate");
				String str = "";
              	on_monitor_begin_fota(str.c_str());
				PreviousFOTACode = CurrentFOTACode;
			}
			else
			{

				if (last_packet.processed == true){

					// WAIT FOR PACKET
					// TODO: ADD IN TIMEOUT CODE
					MONITOR.println("FOTA Initiate: Waiting for Packet");

				}else{

					if (Check_FOTA_FW())
					{
						MONITOR.println("FOTA Initiate:Received Update Packet: true");

						#ifdef FOTA_MANUAL_CONTROL
							MONITOR.println("FOTA MANUAL CONTROL ENABLED: 'FOTA Next' to Continue");
						#else
							
							PreviousFOTACode = CurrentFOTACode;
							CurrentFOTACode = FOTA_Downloading;
							
						#endif
						
					}else if (last_packet.cmd == COMMAND_ID::UPDATE){
						MONITOR.println("FOTA Initiate:Check_FOTA_FW: false");

						
						PreviousFOTACode = CurrentFOTACode;
						CurrentFOTACode = FOTA_Fail;
					}

				}

				


				
			}

			break;

		case FOTA_Downloading:

			

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA Downloading");
				//esp_ota_begin(esp_ota_get_next_update_partition(NULL), OTA_SIZE_UNKNOWN, &ota_handle);

				packetnum = 0;
				PreviousFOTACode = CurrentFOTACode;
			}
			else
			{
				if (last_packet.processed == true){

					// WAIT FOR PACKET
					// TODO: ADD IN TIMEOUT CODE

				}else{

					if (Check_FOTA_DOWNLOAD_DONE())
					{
						// Move on to CheckFW
						MONITOR.println("FOTA Downloading: File Downloaded: true");
						
						// if(ESP_OK!=esp_ota_set_boot_partition(esp_ota_get_next_update_partition(NULL))) {
						// 	// TODO: Put error handling here
						// 	// if it's an error, the update will be discarded
						// }
						
						PreviousFOTACode = CurrentFOTACode;
						CurrentFOTACode = FOTA_Success;
						
					}
					else if (last_packet.cmd == COMMAND_ID::UPDATE)
					{
						packetnum++;
						last_packet.cmd = (COMMAND_ID)NULL;
						MONITOR.println("FOTA Downloading: Packet Received: " + packetnum);
						// FOTA LOOPING CODE SECTION
						
						// if(ESP_OK!=esp_ota_write(ota_handle,update_data.data,(size_t)update_data.size)) {
						// 	MONITOR.println("Failed to open file for appending");
						// 	PreviousFOTACode = CurrentFOTACode;
						// 	CurrentFOTACode = FOTACode::FOTA_Fail;
						// }
						
					}else{

						// TODO: ADD IN A TIMEOUT SECTION

					}
				}
			}

			break;

		case FOTA_Fail:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA Fail");
				PreviousFOTACode = CurrentFOTACode;
			}
			else
			{
				// Move on to CheckFW
				// CurrentFOTACode = FOTA_Downloading;

				
				PreviousFOTACode = CurrentFOTACode;
			}

			break;

		case FOTA_Success:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA Success: Waiting for User to Allow FOTA Update");
				PreviousFOTACode = CurrentFOTACode;
			}
			else
			{
				MONITOR.println("FOTA Success: Validate File: true");
				CurrentFOTACode = FOTACode::FOTA_Done;
				PreviousFOTACode = CurrentFOTACode;
			}

			break;

		case FOTA_Done:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA Done:: Reset ESP");
			}
			else
			{

				
				PreviousFOTACode = CurrentFOTACode;
			}

			break;

		case FOTA_DeleteFW:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA DeleteFW");
				PreviousFOTACode = CurrentFOTACode;
			}
			else
			{
				// Move on to CheckFW
				// CurrentFOTACode = FOTA_Downloading;

				
				PreviousFOTACode = CurrentFOTACode;
			}

			break;

		case FOTA_Rollback:

			// Initial Checks
			if (PreviousFOTACode != CurrentFOTACode)
			{
				MONITOR.println("FOTA Rollback");
				PreviousFOTACode = CurrentFOTACode;
			}
			else
			{
				// Move on to CheckFW
				// CurrentFOTACode = FOTA_Downloading;

				
				PreviousFOTACode = CurrentFOTACode;
			}

			break;

		default:
			MONITOR.println("FOTA Unknown Switch Case");
			break;
		}
	}



}

void setup() {
	MONITOR.begin(115200);
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
    lv_obj_add_event_cb(ui_BTNRestoreDefaults, menu_RestoreDefaults_handler, LV_EVENT_ALL, NULL);
	lv_obj_add_event_cb(ui_btnGoToMenu, button_GoToMenu_handler, LV_EVENT_ALL, NULL);

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

	//================================================== Alarm Screen Events =========================================*/
    lv_obj_add_event_cb(ui_BtnSnooze, ui_snooze_handler, LV_EVENT_ALL, NULL);


    dimmer.max_level(.0625);


    //================================================== XBEE Setup =========================================*/
#ifndef CUSTOM
    XBEE.begin(115200, SERIAL_8N1, 18, 17);
#else
    XBEE.begin(115200, SERIAL_8N1, XBEE_RXD, XBEE_TXD);
#endif
    //MONITOR.begin(115200);
    //MONITOR.printf("Booted\n");
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
        //MONITOR.printf("Failed to initialize device.\n");
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
    
    systemsettings_default.AlarmPower = p_OFF;
    systemsettings_default.AutoSnoozeEnabled = false;
    systemsettings_default.AuxInputEnabled = false;
    systemsettings_default.BatteryVoltage = b_12;
    systemsettings_default.DoorPower = d_CarONCarOFF;
    systemsettings_default.StallMonitorEnabled = true;
    systemsettings_default.TempAveragingEnabled = true;
    systemsettings_default.doorDisabled = false;
    systemsettings_default.AlarmUnitF = true;
    systemsettings_default.AlarmHotSetIndex = 3;
    systemsettings_default.AlarmColdSetIndex = 7;
	systemsettings_default.System_Sleep = false;		// State Flag to determine if the system was been put to sleep
	systemsettings_default.InitialPowerUpFlag = true;   // State flag to determine if the system was just powered on, clears when an idle screen is entered
	
    SettingsLoaded = load_settings();

    update_menu_settings();


    //================================================== State Machine Initialization =========================================*/

    Init_SystemStates();
	Init_Timers();

    systemsettings_previous = systemsettings_current;
	//MONITOR.printf("vim_load\n");
	data_global = vim_load();

    data_global.acedata_changed = false;
    data_global.temp_changed = true;
    data_global.engine_changed = true;
    data_global.k9_door_changed = true;

	ui_Alarm_None_Initial();

    set_HPS(HIGH);
    set_PPS(HIGH);

	/*while(1) {
		loop2();
	}*/
	
	//MONITOR.printf("vim_store\n");
	vim_store(data_global);

	// ADD Date
    MONITOR.println("Firmware Date " __DATE__ );

    
}

void loop() {

    //Check XBee Initialized
    if (last_packet.cmd == COMMAND_ID::ACKNOWLEDGE){
                
        if (last_packet.status == STATUS_CODE::XBEE_INITIALIZED){
            
            if (xbee_initialized){
                //MONITOR.println("====================WARNING=============================");
                //MONITOR.println("XBEE Reset During Operation");
                xbee_reset = true;
            }

            //MONITOR.println("XBEE Initialized");
            xbee_initialized = true;
            
            last_packet.cmd = (COMMAND_ID)NULL;
            last_packet.status = (STATUS_CODE)NULL;

        }else if (last_packet.status == STATUS_CODE::XBEE_CELL_CONNECTED){
            
            MONITOR.println("XBEE Cell Connected");
             xbee_cell_connected = true;
            
            //send_init_packet();
            String blank;
            // on_monitor_connect(blank.c_str());
            // on_monitor_subscribe_command(blank.c_str());
            // on_monitor_subscribe_config(blank.c_str());

            last_packet.cmd = (COMMAND_ID)NULL;
            last_packet.status = (STATUS_CODE)NULL;
        }


    }
	

    check_cell_signal();

	//MONITOR.printf("Display Update\n");
    display_update();

    lv_timer_handler();

    //MONITOR.printf("Calling Monitor Dev Tick\n");
    monitor_dev_tick(MONITOR);

	// FOTA Code Loop
	FOTA_Loop();

    xbee_dev_tick(&my_xbee);

    if(((int)last_packet.status)<0) {
        on_xbee_error(last_packet.cmd, last_packet.status);
    }
		
	//MONITOR.printf("Calling AceCON Dev Tick\n");
    acecon_dev_tick();

	Update_Timers();

	vTaskDelay(5);

	// if (MainLoop_Timer.OverFlowFlag){

	// 	MONITOR.println("==================LOOP====================");

	// 	// //MONITOR.printf("PLACEHOLDER:Checking Door Condition\n");
	// 	check_door_condition();

	// 	// //MONITOR.printf("ui_update_ACECON\n");
	// 	ui_update_acecon();
		
	// 	// //================================================== State Machine Logic =========================================*/
				
	// 	MainLoop_Timer.StartTimer(MainLoop_Timer.Threshold);

	// 	Process_System_Alarm_States();
	
	// 	Process_State_Machine();

	// 	ui_update_icons();

	// }
	

	//MONITOR.printf("Exiting State Machine\n");
	

}

