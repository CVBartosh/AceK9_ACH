#include "SystemSettings.hpp"

// Define the temperature arrays
const int HotTempOpt_F[HotTempArraySize] = {77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95};
const int HotTempOpt_C[HotTempArraySize] = {25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35};

const int ColdTempOpt_F[ColdTempArraySize] = {0, 10, 14, 18, 21, 25, 28, 32, 38, 42, 46, 50, 54, 58};
const int ColdTempOpt_C[ColdTempArraySize] = {0, -12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12};

// Constructor implementation
SystemSettings::SystemSettings()
    : tempAveragingEnabled(false),
      autoSnoozeEnabled(false),
      stallMonitorEnabled(false),
      auxInputEnabled(false),
      coldAlarmEnabled(false),
      doorDisabled(false),
      alarmHotSetIndex(0),
      alarmColdSetIndex(0),
      alarmUnitF(true),
      alarmPower(p_OFF),
      doorPower(d_OFF),
      batteryVoltage(b_10),
      systemSleep(false),
      initialPowerUpFlag(true) {
    // Additional initialization if needed
}

// Method implementations

void SystemSettings::setTempAveragingEnabled(bool enabled) {
    tempAveragingEnabled = enabled;
}

bool SystemSettings::isTempAveragingEnabled() const {
    return tempAveragingEnabled;
}

void SystemSettings::setAutoSnoozeEnabled(bool enabled) {
    autoSnoozeEnabled = enabled;
}

bool SystemSettings::isAutoSnoozeEnabled() const {
    return autoSnoozeEnabled;
}

void SystemSettings::setStallMonitorEnabled(bool enabled) {
    stallMonitorEnabled = enabled;
}

bool SystemSettings::isStallMonitorEnabled() const {
    return stallMonitorEnabled;
}

void SystemSettings::setAuxInputEnabled(bool enabled) {
    auxInputEnabled = enabled;
}

bool SystemSettings::isAuxInputEnabled() const {
    return auxInputEnabled;
}

void SystemSettings::setColdAlarmEnabled(bool enabled) {
    coldAlarmEnabled = enabled;
}

bool SystemSettings::isColdAlarmEnabled() const {
    return coldAlarmEnabled;
}

void SystemSettings::setDoorDisabled(bool disabled) {
    doorDisabled = disabled;
}

bool SystemSettings::isDoorDisabled() const {
    return doorDisabled;
}

void SystemSettings::setAlarmHotSetIndex(int index) {
    if (index >= 0 && index < HotTempArraySize) {
        alarmHotSetIndex = index;
    }
}

void SystemSettings::incAlarmHotSetIndex(){
    setAlarmHotSetIndex(alarmHotSetIndex++);
}

void SystemSettings::decAlarmHotSetIndex(){
    setAlarmHotSetIndex(alarmHotSetIndex--);
}

int SystemSettings::getAlarmHotSetIndex() const {
    return alarmHotSetIndex;
}

void SystemSettings::setAlarmColdSetIndex(int index) {
    if (index >= 0 && index < ColdTempArraySize) {
        alarmColdSetIndex = index;
    }
}

void SystemSettings::incAlarmColdSetIndex(){
    setAlarmColdSetIndex(alarmColdSetIndex++);
}

void SystemSettings::decAlarmColdSetIndex(){
    setAlarmColdSetIndex(alarmColdSetIndex--);
}

int SystemSettings::getAlarmColdSetIndex() const {
    return alarmColdSetIndex;
}

void SystemSettings::setAlarmUnitF(bool isFahrenheit) {
    alarmUnitF = isFahrenheit;
}

bool SystemSettings::isAlarmUnitF() const {
    return alarmUnitF;
}

void SystemSettings::setAlarmPower(PowerOpt powerOption) {
    alarmPower = powerOption;
}

PowerOpt SystemSettings::getAlarmPower() const {
    return alarmPower;
}

void SystemSettings::setDoorPower(DoorOpt doorOption) {
    doorPower = doorOption;
}

DoorOpt SystemSettings::getDoorPower() const {
    return doorPower;
}

void SystemSettings::setBatteryVoltage(Batt voltage) {
    batteryVoltage = voltage;
}

Batt SystemSettings::getBatteryVoltage() const {
    return batteryVoltage;
}

void SystemSettings::setSystemSleep(bool sleep) {
    systemSleep = sleep;
}

bool SystemSettings::isSystemSleep() const {
    return systemSleep;
}

void SystemSettings::setInitialPowerUpFlag(bool flag) {
    initialPowerUpFlag = flag;
}

bool SystemSettings::isInitialPowerUp() const {
    return initialPowerUpFlag;
}

const char* powerOptToString(PowerOpt power) {
    switch (power) {
        case p_CarONCarOFF: return "p_CarONCarOFF";
        case p_CarONManOFF: return "p_CarONManOFF";
        case p_ManONManOFF: return "p_ManONManOFF";
        case p_NoK9Left: return "p_NoK9Left";
        case p_OFF: return "p_OFF";
        default: return "Unknown PowerOpt";
    }
}

const char* doorOptToString(DoorOpt door) {
    switch (door) {
        case d_CarONCarOFF: return "d_CarONCarOFF";
        case d_CarONManOFF: return "d_CarONManOFF";
        case d_ManONManOFF: return "d_ManONManOFF";
        case d_OFF: return "d_OFF";
        default: return "Unknown DoorOpt";
    }
}

const char* battToString(Batt battLevel) {
    switch (battLevel) {
        case b_10: return "b_10";
        case b_105: return "b_105";
        case b_11: return "b_11";
        case b_115: return "b_115";
        case b_12: return "b_12";
        default: return "Unknown Batt";
    }
}


// Global instances of SystemSettings
SystemSettings systemSettingsCurrent;
SystemSettings systemSettingsPrevious;
SystemSettings systemSettingsDefault;
