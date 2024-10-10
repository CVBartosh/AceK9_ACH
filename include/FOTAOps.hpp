#ifndef FOTAOPS_HPP
#define FOTAOPS_HPP

#include <Arduino.h>

// FOTACode enum definition with assigned integer values
enum class FOTACode {
    FOTA_CheckFW,
    FOTA_DeleteFW,
    FOTA_Initiate,
    FOTA_Done,
    FOTA_Downloading = 68,
    FOTA_Waiting = 87,
    FOTA_Begin = 80,
    FOTA_Success = 83,
    FOTA_Rollback = 82,
    FOTA_Fail = 70,
    FOTA_None = 78
};

// FOTAOps class definition
class FOTAOps {
public:
    // Constructor
    FOTAOps();

    // Public methods
    void setFOTACode(FOTACode code);
    FOTACode getFOTACode() const;

    void setPreviousFOTACode(FOTACode code);
    FOTACode getPreviousFOTACode() const;

    void setPacketNum(int num);
    int getPacketNum() const;
    void incPacketNum();

    void setFOTAAllowed(bool allowed);
    bool isFOTAAllowed() const;

    static constexpr int FOTA_Packet_Length = 1024; // Replaces #define

    // Example usage:
    // fotaOps.setFOTACode(FOTACode::FOTA_Downloading);
    // FOTACode currentCode = fotaOps.getFOTACode();

private:
    // Member variables
    FOTACode currentFOTACode;
    FOTACode previousFOTACode;
    int packetNum;
    bool fotaAllowed;
};

// Extern declaration of FOTAOps instance
extern FOTAOps fotaOps;

#endif // FOTAOPS_HPP
