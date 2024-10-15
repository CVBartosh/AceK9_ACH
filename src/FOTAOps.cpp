#include "FOTAOps.hpp"

// Constructor implementation
FOTAOps::FOTAOps()
    : currentFOTACode(FOTACode::FOTA_None),
      previousFOTACode(FOTACode::FOTA_None),
      packetNum(0),
      fotaAllowed(false) {
    // Additional initialization if needed
}

// Public method implementations

// Current FOTA code methods
void FOTAOps::setFOTACode(FOTACode code) {
    currentFOTACode = code;
}

FOTACode FOTAOps::getFOTACode() const {
    return currentFOTACode;
}

// Previous FOTA code methods
void FOTAOps::setPreviousFOTACode(FOTACode code) {
    previousFOTACode = code;
}

FOTACode FOTAOps::getPreviousFOTACode() const {
    return previousFOTACode;
}

void FOTAOps::setTotalPackets(uint32_t total){
    totalPackets = total;
}

uint32_t FOTAOps::getTotalPackets() const {
    return totalPackets;
}
    
// Packet number methods
void FOTAOps::setPacketNum(int num) {
    packetNum = num;
}

int FOTAOps::getPacketNum() const {
    return packetNum;
}

// Packet number methods
void FOTAOps::incPacketNum() {
    packetNum++;
}

// FOTA allowed methods
void FOTAOps::setFOTAAllowed(bool allowed) {
    fotaAllowed = allowed;
}

bool FOTAOps::isFOTAAllowed() const {
    return fotaAllowed;
}

// Definition of the global FOTAOps instance
FOTAOps fotaOps;
