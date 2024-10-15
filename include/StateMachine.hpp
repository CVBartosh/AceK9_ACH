#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <Arduino.h>
#include "SystemSettings.hpp"
#include "vim_controller.hpp"



// Forward declaration of external dependencies (adjust as needed)
extern vim_data data_global;

extern PowerOnTrigger CurrentPowerOnTrigger;

enum StateID {ID_A0_Off,ID_A1_PowerApplied, ID_A3_IgnitionOn,
			  ID_B1_MenuHelp,
              ID_C1_HA_DP,ID_C2_HA_ONLY,ID_C3_DP_ONLY,
              ID_D1_NOK9LeftBehind,ID_D2_PressOKToConfirm,ID_D6_NoK9LeftBehindPowerDownByDoorOpened,ID_D7_PowerDownByOKPress,ID_D8_PowerDownByIgnitionOFF, ID_D9_PowerDownByHAandDPSetToAlwaysOFF, ID_D10_PowerDownByPowerPress, ID_D11_UpdatingFirmware,
              ID_E1_VIMCommunicationsError, ID_E3_UNKNOWN,
              ID_S1_Sleep, 
              ID_G1_SystemTestConfirm, ID_G2_SystemTest};

// StateMachine class definition
class StateMachine {
public:
    // Constructor
    StateMachine();

    // Accessor methods
    StateID getCurrentState() const;
    StateID getPreviousState() const;

    // Mutator methods
    void setCurrentState(StateID state);
    void setPreviousState(StateID state);
    void syncStates();
    
    void setNextState(const StateID& nextState);
    StateID determineNextState();
    void setDeterminedState();
    bool isCurrentState(const StateID& state);
    bool isPreviousState(const StateID& state);

    const char* systemStateToString(StateID state);

private:

    StateID systemStateCurrent;
    StateID systemStatePrevious;
    
};

#endif // STATEMACHINE_HPP
