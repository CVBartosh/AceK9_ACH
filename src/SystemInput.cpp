#include "SystemInput.hpp"

// Constructor implementation
SystemInput::SystemInput()
    : currentDoorPopCondition(dc_Unpopped),
      currentDoorPopTrigger(dt_None),
      updateDoorFlag(false),
      currentPowerOnTrigger(NoTrigger){
    // Additional initialization if needed
}

// Accessor and Mutator methods implementation

void SystemInput::setDoorPopCondition(DoorPopCondition condition) {
    currentDoorPopCondition = condition;
}

DoorPopCondition SystemInput::getDoorPopCondition() const {
    return currentDoorPopCondition;
}

void SystemInput::setDoorPopTrigger(DoorPopTrigger trigger) {
    currentDoorPopTrigger = trigger;
}

DoorPopTrigger SystemInput::getDoorPopTrigger() const {
    return currentDoorPopTrigger;
}

void SystemInput::setUpdateDoorFlag(bool flag) {
    updateDoorFlag = flag;
}

bool SystemInput::getUpdateDoorFlag() const {
    return updateDoorFlag;
}

void SystemInput::setPowerOnTrigger(PowerOnTrigger trigger) {
    currentPowerOnTrigger = trigger;
}

PowerOnTrigger SystemInput::getPowerOnTrigger() const {
    return currentPowerOnTrigger;
}

// Global instance of SystemInput
SystemInput systemInput;
