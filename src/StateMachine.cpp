#include "StateMachine.hpp"
#include "SerialComms.hpp"
#include "SystemSettings.hpp"

extern SystemSettings systemSettingsCurrent;

// StateMachine constructor
StateMachine::StateMachine()
    : systemStateCurrent(),
      systemStatePrevious(){
}

// Accessor methods for StateMachine
StateID StateMachine::getCurrentState() const {
    return systemStateCurrent;
}

StateID StateMachine::getPreviousState() const {
    return systemStatePrevious;
}

void StateMachine::setCurrentState(StateID state) {
    systemStateCurrent = state;
}

void StateMachine::setPreviousState(StateID state) {
    systemStatePrevious = state;
}

const char* StateMachine::systemStateToString(StateID state) {
    switch (state) {
        case ID_A0_Off: return "ID_A0_Off";
        case ID_A1_PowerApplied: return "ID_A1_PowerApplied";
        case ID_A3_IgnitionOn: return "ID_A3_IgnitionOn";
        case ID_B1_MenuHelp: return "ID_B1_MenuHelp";
        case ID_C1_HA_DP: return "ID_C1_HA_DP";
        case ID_C2_HA_ONLY: return "ID_C2_HA_ONLY";
        case ID_C3_DP_ONLY: return "ID_C3_DP_ONLY";
        case ID_D1_NOK9LeftBehind: return "ID_D1_NOK9LeftBehind";
        case ID_D2_PressOKToConfirm: return "ID_D2_PressOKToConfirm";
        case ID_D6_NoK9LeftBehindPowerDownByDoorOpened: return "ID_D6_NoK9LeftBehindPowerDownByDoorOpened";
        case ID_D7_PowerDownByOKPress: return "ID_D7_PowerDownByOKPress";
        case ID_D8_PowerDownByIgnitionOFF: return "ID_D8_PowerDownByIgnitionOFF";
        case ID_D9_PowerDownByHAandDPSetToAlwaysOFF: return "ID_D9_PowerDownByHAandDPSetToAlwaysOFF";
        case ID_D10_PowerDownByPowerPress: return "ID_D10_PowerDownByPowerPress";
        case ID_D11_UpdatingFirmware: return "ID_D11_UpdatingFirmware";
        case ID_E1_VIMCommunicationsError: return "ID_E1_VIMCommunicationsError";
        case ID_E3_UNKNOWN: return "ID_E3_UNKNOWN";
        case ID_S1_Sleep: return "ID_S1_Sleep";
        case ID_G1_SystemTestConfirm: return "ID_G1_SystemTestConfirm";
        case ID_G2_SystemTest: return "ID_G2_SystemTest";
        default: return "Unknown State";
    }
}


bool StateMachine::isCurrentState(const StateID& state){

	if (systemStateCurrent == state){return true;}
	
	return false;

}

bool StateMachine::isPreviousState(const StateID& state){

	if (systemStatePrevious == state){return true;}
	
	return false;

}

void StateMachine::syncStates(){
	setPreviousState(getCurrentState());
}

// Implementing Set_Next_State as setNextState method
void StateMachine::setNextState(const StateID& nextState) {

	MONITOR.println("Next System State: " + String(systemStateToString(nextState)));
	setPreviousState(getCurrentState());
	setCurrentState(nextState);
    
}

// Implementing Determine_Next_State as determineNextState method
StateID StateMachine::determineNextState()
{
	MONITOR.println("Determine Next State");
	MONITOR.printf("IngitionOn:  %s\r\n",data_global.ignitionOn_current?"TRUE":"FALSE");
	MONITOR.println("Door Power: " + String(doorOptToString(systemSettingsCurrent.getDoorPower())));
	MONITOR.println("Alarm Power: " + String(powerOptToString(systemSettingsCurrent.getAlarmPower())));
	MONITOR.println("PowerOnTrigger: " + String(powerOnTriggerToString(systemInput.getPowerOnTrigger())));
	
	if (CurrentPowerOnTrigger == PowerOnTrigger::Applied ||
		CurrentPowerOnTrigger == PowerOnTrigger::PowerButtonPress)
	{
		MONITOR.println("Power Trigger: Applied");
		// Ignition OFF
		if (data_global.ignitionOn_current == false)
		{
			MONITOR.println("ignitionOn: false");
			// Door Popper Settings
			if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_OFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (ID_D9_PowerDownByHAandDPSetToAlwaysOFF);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (ID_D8_PowerDownByIgnitionOFF);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (ID_D1_NOK9LeftBehind);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (ID_C2_HA_ONLY);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (ID_C2_HA_ONLY);
				}
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONCarOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (ID_D8_PowerDownByIgnitionOFF);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (ID_D1_NOK9LeftBehind);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (ID_C2_HA_ONLY);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (ID_C2_HA_ONLY);
				}
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONManOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (ID_C3_DP_ONLY);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (ID_D1_NOK9LeftBehind);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (ID_C1_HA_DP);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (ID_C1_HA_DP);
				}
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_ManONManOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (ID_C3_DP_ONLY);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (ID_C1_HA_DP);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (ID_C1_HA_DP);
				}
			}

		}

		// Ignition ON
		else if (data_global.ignitionOn_current == true)
		{
			// Door Popper Settings
			if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_OFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (ID_D9_PowerDownByHAandDPSetToAlwaysOFF);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left ||
						 systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (ID_C2_HA_ONLY);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (ID_C2_HA_ONLY);
				// }
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONCarOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (ID_C3_DP_ONLY);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left ||
						 systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (ID_C1_HA_DP);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (ID_C1_HA_DP);
				// }
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONManOFF || systemSettingsCurrent.getDoorPower() == DoorOpt::d_ManONManOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (ID_C3_DP_ONLY);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left ||
						 systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (ID_C1_HA_DP);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (ID_C1_HA_DP);
				// }
			}
		}

	}
	else if (CurrentPowerOnTrigger == PowerOnTrigger::IgnitionOn)
	{
		MONITOR.println("Power Trigger: Ignition On");
		// Door Popper Settings
		if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_OFF)
		{
			// Alarm Settings
			if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
			{
				return (ID_D8_PowerDownByIgnitionOFF);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF ||
					 systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
			{
				return (ID_C2_HA_ONLY);
			}
			// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
			// {
			// 	return (ID_C2_HA_ONLY);
			// }
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
			{
				return (ID_E3_UNKNOWN);
			}

		}
		else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONCarOFF || systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONManOFF)
		{
			// Alarm Settings
			if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
			{
				return (ID_C3_DP_ONLY);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
			{
				return (ID_C1_HA_DP);
			}
			// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
			// {
			// 	return (ID_C1_HA_DP);
			// }
		}
		else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_ManONManOFF)
		{
			// Alarm Settings
			if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
			{
				return (ID_E3_UNKNOWN);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF ||
					 systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
			{
				return (ID_C2_HA_ONLY);
			}
			// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
			// {
			// 	return (ID_C2_HA_ONLY);
			// }
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
			{
				return (ID_E3_UNKNOWN);
			}

		}
	
	}
	MONITOR.println("WTF?");

return (ID_E3_UNKNOWN);
}

// Implementing Set_Determined_State as setDeterminedState method
void StateMachine::setDeterminedState() {
    setNextState(determineNextState());
}

