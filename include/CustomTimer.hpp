#ifndef CUSTOMTIMER_HPP
#define CUSTOMTIMER_HPP

#include <Arduino.h>

// Enum for Timer IDs
enum TimerID {
    Timer_A0_Off,
    Timer_A1_PowerApplied,
    Timer_A3_IgnitionOn,
    Timer_B1_MenuHelp,
    Timer_C1_HA_DP,
    Timer_C2_HA_ONLY,
    Timer_C3_DP_ONLY,
    Timer_D1_NOK9LeftBehind,
    Timer_D2_PressOKToConfirm,
    Timer_D6_NoK9LeftBehindPowerDownByDoorOpened,
    Timer_D7_PowerDownByOKPress,
    Timer_D8_PowerDownByIgnitionOFF,
    Timer_D9_PowerDownByHAandDPSetToAlwaysOFF,
    Timer_D10_PowerDownByPowerPress,
    Timer_D11_UpdatingFirmware,
    Timer_E1_VIMCommunicationsError,
    Timer_E3_UNKNOWN,
    Timer_S1_Sleep,
    Timer_G1_SystemTestConfirm,
    Timer_G2_SystemTest,
    Timer_Read_IBoxPopped,
    Timer_Clear_IBoxPopped,
    Timer_ReadGear,
    Timer_ClearGear,
    Timer_SystemAlarm,
    Timer_PreAlarmNotification,
    Timer_SnoozeAlarm,
    Timer_NoK9Blink,
    Timer_NoK9Beep,
    Timer_NoK9FirstAlert,
    Timer_NoK9SecondAlert,
    Timer_COMReset,
    Timer_COMError,
    Timer_InitialPowerUp,
    Timer_VIMAlarm,
    Timer_MainLoop
};

// CustomTimer class definition
class CustomTimer {
public:
    // Constructor
    CustomTimer();

    // Public methods
    void setTimerID(TimerID id);
    void setName(const String& name);
    void setTimerVal(unsigned long timeval);
    void syncTimerVal();
    void setThreshold(unsigned long thresholdval);
    unsigned long readTimerVal() const;
    bool checkOverflow();
    void startTimer();
    void stopTimer();
    bool isOverflowed() const;
    bool isEnabled() const;
    TimerID getTimerID() const;
    String getName() const;

    // New methods to set overflowFlag and timerEnable directly
    void setOverflowFlag(bool flag);
    void setTimerEnable(bool enable);

private:
    // Member variables
    TimerID index;
    String name;
    unsigned long timeVal;
    unsigned long threshold;
    bool overflowFlag;
    bool timerEnable;
};

#endif // CUSTOMTIMER_HPP