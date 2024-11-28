#pragma once
#include "config.h"
#include "vehicle_state.h"
#include "vehicle_parameters.h"

class DMCHandler {
public:
    DMCHandler(mcp2515_can& can);
    void sendDMC();
    void receiveDMC();
    int16_t calculateTorque();
    
    // Getters
    float getSpeed() const { return DMC_SpdAct; }
    float getTemp() const { return DMC_TempInv; }
    float getMotorTemp() const { return DMC_TempMot; }
    bool isEnabled() const { return enableDMC; }
    bool isRunning() const { return DMC_Running; }

private:
    mcp2515_can& canBus;
    byte readDataDMC[MAX_DATA_SIZE] = {0};
    unsigned char controllBufferDMC[8] = {0};
    unsigned char limitBufferDMC[8] = {0};
    unsigned char controllBuffer2DMC[8] = {0};

    // State variables
    bool enableDMC = false;
    bool modeDMC = false;
    bool DMC_Running = false;
    bool DMC_Ready = false;
    float DMC_SpdAct = 0;
    float DMC_TempInv = 0;
    float DMC_TempMot = 0;
    float DMC_DcVltAct = 0;
    float DMC_DcCurrAct = 0;
    int32_t DMC_MechPwr = 0;

    // Control variables
    int16_t DMC_TrqRq_Scale = 0;
    int DMC_SpdRq = 6000;
    int DMC_DcVLimMot = MIN_U_BAT;
    int DMC_DcVLimGen = MAX_U_BAT;
    int16_t sampleSetPedal[5] = {0};
    float lastTorque = 0;

    // Helper functions
    float calculateRegenTorque(float speed, float throttle);
    float calculateDriveTorque(float throttle, bool reverse);
    void applyTorqueLimits(float& torque, bool reverse);
    void calculateSpeed();
    void scaleAndPackValues();
};