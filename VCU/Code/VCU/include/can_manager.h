#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include "config.h"
#include "vehicle_parameters.h"

struct BMSData {
    uint8_t soc;
    uint16_t voltage;
    int16_t current;
    uint16_t maxDischarge;
    uint8_t maxCharge;
};

struct DMCData {
    bool ready;
    bool running;
    float torqueAvailable;
    float torqueActual;
    float speedActual;
    float dcVoltageAct;
    float dcCurrentAct;
    float acCurrentAct;
    int32_t mechPower;
    float tempInverter;
    float tempMotor;
    int8_t tempSystem;
};

struct BSCData {
    float hvVoltageAct;
    float lvVoltageAct;
    float hvCurrentAct;
    float lvCurrentAct;
    uint8_t mode;
};

struct NLGData {
    uint8_t stateCtrlPilot;
    uint16_t dcHvVoltageAct;
    uint8_t stateAct;
    uint16_t dcHvCurrentAct;
    bool connectorLocked;
    uint8_t coolingRequest;
    float tempCoolPlate;
    bool unlockRequest;
    uint8_t stateDemand;
    uint8_t ledDemand;
};

class CANManager {
public:
    explicit CANManager(uint8_t cs_pin);
    ~CANManager();
    
    // Initialization
    void begin();
    void update();
    
    // Send methods
    void sendBSC();
    void sendDMC();
    void sendNLG();
    
    // Data access methods
    const BMSData& getBMSData() const { return bmsData; }
    const DMCData& getDMCData() const { return dmcData; }
    const BSCData& getBSCData() const { return bscData; }
    const NLGData& getNLGData() const { return nlgData; }
    
    // Control methods
    void setTorqueDemand(float torque) { torqueDemand = torque; }
    void setSpeedDemand(int16_t speed) { speedDemand = speed; }
    void setEnableDMC(bool enable) { enableDMC = enable; }
    void setEnableBSC(bool enable) { enableBSC = enable; }
    void setModeBSC(bool mode) { modeBSC = mode; }
    
private:
    // Interrupt handling
    static void IRAM_ATTR handleInterrupt();
    static volatile bool messageAvailable;
    void processCANMessage();
    
    // Message processing methods
    void processBMSMessage(uint8_t* buf);
    void processBSCMessage(uint8_t* buf);
    void processDMCMessage(uint32_t id, uint8_t* buf);
    void processNLGMessage(uint32_t id, uint8_t* buf);
    
    // Internal methods
    void resetMessageBuffers();
    
    // Hardware
    mcp2515_can CAN;
    SPIClass* customSPI;
    
    // Message buffers
    uint8_t controlBufferDMC[8];
    uint8_t controlBufferBSC[8];
    uint8_t limitBufferBSC[8];
    uint8_t limitBufferDMC[8];
    uint8_t controlBufferNLG[8];
    
    // Data storage
    BMSData bmsData;
    DMCData dmcData;
    BSCData bscData;
    NLGData nlgData;
    
    // Control parameters
    float torqueDemand;
    int16_t speedDemand;
    bool enableDMC;
    bool enableBSC;
    bool modeBSC;
    uint16_t lvVoltage;
    uint16_t hvVoltage;
    
    // Timing
    unsigned long lastFastCycle;
    unsigned long lastSlowCycle;
};