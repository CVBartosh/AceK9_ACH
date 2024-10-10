#include "StateMachine.hpp"
#include "FOTAOps.hpp" // Include FOTAOps for FOTA code access
#include "SerialComms.hpp"
#include "SystemSettings.hpp"

// Accessor methods for SystemState
int getIndex() const {
    return index;
}

String SystemState::getName() const {
    return name;
}

// Mutator methods for SystemState
void SystemState::setIndex(int idx) {
    index = idx;
}

void SystemState::setName(const String& name) {
    this->name = name;
}

// StateMachine constructor
StateMachine::StateMachine()
    : systemStateCurrent(),
      systemStatePrevious(),
      storedNextState() {
}

// Accessor methods for StateMachine
SystemState StateMachine::getCurrentState() const {
    return systemStateCurrent;
}

SystemState StateMachine::getPreviousState() const {
    return systemStatePrevious;
}

SystemState StateMachine::getStoredNextState() const {
    return storedNextState;
}

// Implementing Set_Next_State as setNextState method
void StateMachine::setNextState(const SystemState& nextState) {
    // FOTA functionality checks
    FOTACode currentFOTACode = fotaOps.getFOTACode();

    if ((currentFOTACode == FOTACode::FOTA_Downloading || currentFOTACode == FOTACode::FOTA_Waiting) &&
        (nextState.getIndex() == D6_NoK9LeftBehindPowerDownByDoorOpened_State.getIndex() ||
         nextState.getIndex() == D7_PowerDownByOKPress_State.getIndex() ||
         nextState.getIndex() == D8_PowerDownByIgnitionOFF_State.getIndex() ||
         nextState.getIndex() == D9_PowerDownByHAandDPSetToAlwaysOFF_State.getIndex() ||
         nextState.getIndex() == D10_PowerDownByPowerPress_State.getIndex())) {

        storedNextState = nextState;

        MONITOR.println("Next System State: " + D11_UpdatingFirmware_State.getName());
        MONITOR.println("Stored Next System State: " + storedNextState.getName());

        systemStatePrevious = systemStateCurrent;
        systemStateCurrent = D11_UpdatingFirmware_State;

    } else {
        MONITOR.println("Next System State: " + nextState.getName());
        systemStatePrevious = systemStateCurrent;
        systemStateCurrent = nextState;
    }
}

// Implementing Determine_Next_State as determineNextState method
SystemState StateMachine::determineNextState()
{
	MONITOR.println("Determine Next State");
	MONITOR.printf("IngitionOn:  %s\r\n",data_global.ignitionOn_current?"TRUE":"FALSE");
	switch (systemSettingsCurrent.getDoorPower())
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
	switch (systemSettingsCurrent.getAlarmPower())
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
			if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_OFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (D9_PowerDownByHAandDPSetToAlwaysOFF_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (D8_PowerDownByIgnitionOFF_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (D1_NOK9LeftBehind_State);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (C2_HA_ONLY_State);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (C2_HA_ONLY_State);
				}
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONCarOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (D8_PowerDownByIgnitionOFF_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (D8_PowerDownByIgnitionOFF_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (D1_NOK9LeftBehind_State);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (C2_HA_ONLY_State);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (C2_HA_ONLY_State);
				}
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONManOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (D1_NOK9LeftBehind_State);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (C1_HA_DP_State);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (C1_HA_DP_State);
				}
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_ManONManOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (C3_DP_ONLY_State);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (C1_HA_DP_State);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (C1_HA_DP_State);
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
					return (D9_PowerDownByHAandDPSetToAlwaysOFF_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (C2_HA_ONLY_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (C2_HA_ONLY_State);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (C2_HA_ONLY_State);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (C2_HA_ONLY_State);
				}
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONCarOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (C1_HA_DP_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (C1_HA_DP_State);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (C1_HA_DP_State);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
				{
					return (C1_HA_DP_State);

				}
			}
			else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONManOFF || systemSettingsCurrent.getDoorPower() == DoorOpt::d_ManONManOFF)
			{
				// Alarm Settings
				if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
				{
					return (C3_DP_ONLY_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF)
				{
					return (C1_HA_DP_State);
				}
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
				{
					return (C1_HA_DP_State);
				}
				// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
				// {
				// 	return (C1_HA_DP_State);
				// }
				else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
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
		if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_OFF)
		{
			// Alarm Settings
			if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
			{
				return (D8_PowerDownByIgnitionOFF_State);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF)
			{
				return (C2_HA_ONLY_State);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
			{
				return (C2_HA_ONLY_State);
			}
			// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
			// {
			// 	return (C2_HA_ONLY_State);
			// }
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
			{
				return (E3_Unknown_State);
			}

		}
		else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONCarOFF || systemSettingsCurrent.getDoorPower() == DoorOpt::d_CarONManOFF)
		{
			// Alarm Settings
			if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
			{
				return (C3_DP_ONLY_State);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF)
			{
				return (C1_HA_DP_State);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
			{
				return (C1_HA_DP_State);
			}
			// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
			// {
			// 	return (C1_HA_DP_State);
			// }
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
			{
				return (C3_DP_ONLY_State);
			}

		}
		else if (systemSettingsCurrent.getDoorPower() == DoorOpt::d_ManONManOFF)
		{
			// Alarm Settings
			if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_OFF)
			{
				return (E3_Unknown_State);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONCarOFF || systemSettingsCurrent.getAlarmPower() == PowerOpt::p_CarONManOFF)
			{
				return (C2_HA_ONLY_State);
			}
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_NoK9Left)
			{
				return (C2_HA_ONLY_State);
			}
			// else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_AutoStart)
			// {
			// 	return (C2_HA_ONLY_State);
			// }
			else if (systemSettingsCurrent.getAlarmPower() == PowerOpt::p_ManONManOFF)
			{
				return (E3_Unknown_State);
			}

		}
	}

	MONITOR.println("WTF?");

return (E3_Unknown_State);
}


// Implementing Set_Determined_State as setDeterminedState method
void StateMachine::setDeterminedState() {
    setNextState(determineNextState());
}