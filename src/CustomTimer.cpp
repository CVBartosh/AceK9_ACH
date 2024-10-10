#include "CustomTimer.hpp"

// Constructor implementation
CustomTimer::CustomTimer()
    : index(Timer_E3_UNKNOWN), name(""), timeVal(0), threshold(0), overflowFlag(false), timerEnable(false) {}

// Method implementations
void CustomTimer::setTimerID(TimerID id) {
    index = id;
}

void CustomTimer::setName(const String& name) {
    this->name = name;
}

void CustomTimer::setTimerVal(unsigned long timeval) {
    timeVal = timeval;
}

void CustomTimer::syncTimerVal() {
    timeVal = millis();
}

void CustomTimer::setThreshold(unsigned long thresholdval) {
    threshold = thresholdval;
}

unsigned long CustomTimer::readTimerVal() const {
    return timeVal;
}

bool CustomTimer::checkOverflow() {
    if (timerEnable) {
        if (millis() - timeVal >= threshold) {
            overflowFlag = true;
            return true;
        }
    }
    return false;
}

void CustomTimer::startTimer() {
    overflowFlag = false;
    timerEnable = true;
    syncTimerVal();
}

void CustomTimer::stopTimer() {
    overflowFlag = false;
    timerEnable = false;
}

bool CustomTimer::isOverflowed() const {
    return overflowFlag;
}

bool CustomTimer::isEnabled() const {
    return timerEnable;
}

TimerID CustomTimer::getTimerID() const {
    return index;
}

String CustomTimer::getName() const {
    return name;
}

// New methods to set overflowFlag and timerEnable directly
void CustomTimer::setOverflowFlag(bool flag) {
    overflowFlag = flag;
}

void CustomTimer::setTimerEnable(bool enable) {
    timerEnable = enable;
    if (enable) {
        syncTimerVal(); // Sync time when enabling the timer
    }
}
