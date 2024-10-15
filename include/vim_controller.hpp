#pragma once
#include <Arduino.h>
#include "SystemSettings.hpp"

/// @brief vim_data: the VIM information goes here
struct vim_data {
    // System Settings
    char VIMSerialNumber[24];

    // ACEDATA Variables
    bool Comms_OK;
    bool acedata_changed;

    // Door Variables
    bool k9_door_open_previous;
    bool k9_door_open_current;
    bool k9_door_changed;
    bool k9_door_popped;

    // Temperature Variables
    bool tempvalues_previous;
    bool tempvalues_current;
    float leftTemp_current;
    float leftTemp_previous;
    bool leftTempError_current;
    bool leftTempError_previous;
    char leftTempSign;
    TempState leftTempState;
    float rightTemp_current;
    float rightTemp_previous;
    bool rightTempError_current;
    bool rightTempError_previous;
    char rightTempSign;
    TempState rightTempState;
    float avgTemp;
    TempState avgTempState;
    bool temp_changed;
    bool units_changed;
	bool temp_alarmFlag_current;
    bool temp_alarmFlag_previous;
	bool temp_errorFlag_current;
    bool temp_errorFlag_previous;

    // Battery Variables
    Batt BatterySetting = Batt::b_10;
    float voltage;
    int BadBatteryCounter=0;
    bool batt_error_previous;
    bool batt_error_current;
    bool battchanged;

    // Engine Variables
    bool engineStallSensorPresent;
    int enginestallcount;
    bool engineStalled_previous;
    bool engineStalled_current;
    bool ignitionOn_current;
    bool ignitionOn_previous;
	IgnEdge ignitionEdge_current;
    IgnEdge ignitionEdge_previous;
    bool inGear;
    bool engine_changed;

    // Aux Variables
    bool AuxEnabled;
    bool Aux1Input_previous;
    bool Aux1Input_current;
    bool Aux2Input_previous;
    bool Aux2Input_current;
    bool aux_changed;

    // General Variables
    bool updateIcons;

};
/// @brief The VIM callback. Your function should follow this signature
typedef void(*vim_on_receive_cb)(const char* data, void* state);
/// @brief Write a string to the VIM serial port
/// @param sz The string to write
void vim_write_sz(const char* sz);
/// @brief Initialize the VIM serial
/// @param callback The callback when a VIM message is received
/// @param state Any associated user state (optional)
void vim_init(vim_on_receive_cb callback,void* state);
/// @brief Store VIM state for access from on_receive
/// @param newData the data to store
void vim_store(const vim_data& newData);
/// @brief Load the VIM state
/// @return The stored VIM state
vim_data vim_load();
/// @brief Exchange the old VIM state with a new one
/// @param newData The new VIM state
/// @return The previous VIM state
vim_data vim_exchange(const vim_data& newData);