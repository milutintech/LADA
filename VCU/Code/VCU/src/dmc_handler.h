#pragma once
#include "config.h"
#include "vehicle_state.h"

class DMCHandler {
public:
    DMCHandler(mcp2515_can& can);
    void sendDMC();
    void receiveDMC();
    int16_t calculateTorque();
    void processTorque();
    
    float getDMC_SpdAct() const { return DMC_SpdAct; }
    float getDMC_TempInv() const { return DMC_TempInv; }
    float getDMC_TempMot() const { return DMC_TempMot; }

private:
    mcp2515_can& canBus;
    byte readDataDMC[MAX_DATA_SIZE] = {0};
    unsigned char controllBufferDMC[8] = {0};
    unsigned char limitBufferDMC[8] = {0};
    
    // State variables
    bool enableDMC = false;
    bool modeDMC = false;
    bool oscLim = true;
    bool negTrqSpd = true;
    bool posTrqSpd = true;
    bool errLatch = false;
    
    // Motor parameters
    float DMC_SpdAct = 0;
    float DMC_TempInv = 0;
    float DMC_TempMot = 0;
    int16_t DMC_TrqRq_Scale = 0;
    int DMC_SpdRq = 6000;
    
    // Torque calculation
    int16_t sampleSetPedal[5] = {0};
    float lastTorque = 0;
    uint16_t speed = 0;
    
    // Helper functions
    float calculateRegenTorque(float rawSpeed, float throttlePosition);
    float calculateDriveTorque(float throttlePosition, bool isReverse);
    float applyTorqueLimits(float calculatedTorque, bool isReverse);
    void updateSpeedCalculation();
};