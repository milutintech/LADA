// dmc_handler.h
#pragma once
#include "base_can_handler.h"
#include "message_scaling.h"
#include "vehicle_parameters.h"

class DMCHandler : public BaseCANHandler, protected MessageScaling {
public:
    explicit DMCHandler(mcp2515_can& can);
    void sendDMC();
    void receiveDMC();
    int16_t calculateTorque();

    // Getters
    float getSpeed() const { return DMC_SpdAct; }
    float getTemp() const { return DMC_TempInv; }
    float getMotorTemp() const { return DMC_TempMot; }
    bool isEnabled() const { return enableDMC; }
    bool isRunning() const { return DMC_Running; }
    void setEnable(bool enable) { enableDMC = enable; }

private:
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
    int DMC_DcVLimMot = VehicleParams::Power::MIN_U_BAT;
    int DMC_DcVLimGen = VehicleParams::Power::MAX_U_BAT;
    int16_t sampleSetPedal[5] = {0};
    float lastTorque = 0;
    bool LowRange = false;

    void processStatusMessage(const CANMessage& msg);
    void processTemperatureMessage(const CANMessage& msg);
    void processVoltageCurrentMessage(const CANMessage& msg);
    float calculateRegenTorque(float speed, float throttle);
    float calculateDriveTorque(float throttle, bool reverse);
    void applyTorqueLimits(float& torque, bool reverse);
    void calculateSpeed();
    void scaleAndPackValues();
    float calculateMedianThrottle();
    void packControlMessage();
    void packLimitMessage();
};
