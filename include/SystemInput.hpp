#ifndef SYSTEMINPUT_HPP
#define SYSTEMINPUT_HPP

#include <Arduino.h>

#define GEAR_INGEAR 0
#define GEAR_PARKED 1

// Enumerations for Door Pop Condition
enum DoorPopCondition {
    dc_Popped = 0,
    dc_Unpopped = 1
};

// Enumeration for Door Pop Trigger
enum DoorPopTrigger {
    dt_None,
    dt_Salute,
    dt_Remote
};

// Enumeration for Power On Trigger
enum PowerOnTrigger {
    NoTrigger,
    Applied,
    PowerButtonPress,
    IgnitionOn
};

const char* powerOnTriggerToString(PowerOnTrigger trigger);

// SystemInput class definition
class SystemInput {
public:
    // Constructor
    SystemInput();

    // Accessor and Mutator methods for Door Pop Condition
    void setDoorPopCondition(DoorPopCondition condition);
    DoorPopCondition getDoorPopCondition() const;

    // Accessor and Mutator methods for Door Pop Trigger
    void setDoorPopTrigger(DoorPopTrigger trigger);
    DoorPopTrigger getDoorPopTrigger() const;

    // Accessor and Mutator methods for Update Door Flag
    void setUpdateDoorFlag(bool flag);
    bool getUpdateDoorFlag() const;

    // Accessor and Mutator methods for Power On Trigger
    void setPowerOnTrigger(PowerOnTrigger trigger);
    PowerOnTrigger getPowerOnTrigger() const;

    // Accessor and Mutator methods for Gear Status
    //void setGearStatus(int status);
    //bool getGearStatus() const;
    

    // Example usage:
    // SystemInput input;
    // input.setDoorPopCondition(dc_Popped);
    // DoorPopCondition condition = input.getDoorPopCondition();

private:
    // Member variables
    DoorPopCondition currentDoorPopCondition;
    DoorPopTrigger currentDoorPopTrigger;
    bool updateDoorFlag;
    PowerOnTrigger currentPowerOnTrigger;
    
};

// Global instance of SystemInput
extern SystemInput systemInput;

#endif // SYSTEMINPUT_HPP
