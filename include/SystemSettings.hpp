#ifndef SYSTEMSETTINGS_HPP
#define SYSTEMSETTINGS_HPP

#include <Arduino.h>
#include "SystemInput.hpp"

// Define the size of temperature arrays
#define HotTempArraySize 19
#define ColdTempArraySize 14

// Temperature options in Fahrenheit and Celsius
extern const int HotTempOpt_F[HotTempArraySize];
extern const int HotTempOpt_C[HotTempArraySize];
extern const int ColdTempOpt_F[ColdTempArraySize];
extern const int ColdTempOpt_C[ColdTempArraySize];

enum TempState {tst_OK,tst_Warning,tst_Over, tst_OverPlus,tst_Under};
enum IgnEdge{it_None,it_Falling,it_Rising};

// Enumerations for power options, door options, and battery voltage
enum PowerOpt {
    p_CarONCarOFF,
    p_CarONManOFF,
    p_ManONManOFF,
    p_NoK9Left,
    p_OFF
};


enum DoorOpt {
    d_CarONCarOFF,
    d_CarONManOFF,
    d_ManONManOFF,
    d_OFF
};

enum Batt {
    b_10 = 1,
    b_105 = 2,
    b_11 = 3,
    b_115 = 4,
    b_12 = 5
};

// SystemSettings class definition
class SystemSettings {
public:
    // Constructor
    SystemSettings();

    // Public methods
    void setTempAveragingEnabled(bool enabled);
    bool isTempAveragingEnabled() const;

    void setAutoSnoozeEnabled(bool enabled);
    bool isAutoSnoozeEnabled() const;

    void setStallMonitorEnabled(bool enabled);
    bool isStallMonitorEnabled() const;

    void setAuxInputEnabled(bool enabled);
    bool isAuxInputEnabled() const;

    void setColdAlarmEnabled(bool enabled);
    bool isColdAlarmEnabled() const;

    void setDoorDisabled(bool disabled);
    bool isDoorDisabled() const;

    void setAlarmHotSetIndex(int index);
    void incAlarmHotSetIndex();
    void decAlarmHotSetIndex();
    int getAlarmHotSetIndex() const;

    void setAlarmColdSetIndex(int index);
    void incAlarmColdSetIndex();
    void decAlarmColdSetIndex();
    int getAlarmColdSetIndex() const;

    void setAlarmUnitF(bool isFahrenheit);
    bool isAlarmUnitF() const;

    void setAlarmPower(PowerOpt powerOption);
    PowerOpt getAlarmPower() const;

    void setDoorPower(DoorOpt doorOption);
    DoorOpt getDoorPower() const;

    void setBatteryVoltage(Batt voltage);
    Batt getBatteryVoltage() const;

    void setSystemSleep(bool sleep);
    bool isSystemSleep() const;

    void setInitialPowerUpFlag(bool flag);
    bool isInitialPowerUp() const;

private:
    // Member variables
    bool tempAveragingEnabled;
    bool autoSnoozeEnabled;
    bool stallMonitorEnabled;
    bool auxInputEnabled;
    bool coldAlarmEnabled;
    bool doorDisabled;
    int alarmHotSetIndex;
    int alarmColdSetIndex;
    bool alarmUnitF;
    PowerOpt alarmPower;
    DoorOpt doorPower;
    Batt batteryVoltage;
    bool systemSleep;           // State flag to determine if the system has been put to sleep
    bool initialPowerUpFlag;    // State flag to determine if the system was just powered on
};

const char* doorOptToString(DoorOpt door);
const char* powerOptToString(PowerOpt powerr);
const char* battToString(Batt battLevel);

// Global instances of SystemSettings
extern SystemSettings systemSettingsCurrent;
extern SystemSettings systemSettingsPrevious;
extern SystemSettings systemSettingsDefault;

#endif // SYSTEMSETTINGS_HPP
