#include "can_manager.h"
#include <esp_task_wdt.h>

volatile bool CANManager::messageAvailable = false;

CANManager::CANManager(uint8_t cs_pin) 
    : CAN(cs_pin)
    , lastFastCycle(0)
    , lastSlowCycle(0)
    , enableDMC(false)
    , enableBSC(false)
    , modeBSC(false)
    , torqueDemand(0)
    , speedDemand(0)
{
    customSPI = new SPIClass(HSPI);
    resetMessageBuffers();
}

CANManager::~CANManager() {
    delete customSPI;
}

void CANManager::begin() {
    customSPI->begin(Pins::SCK, Pins::MISO, Pins::MOSI, Pins::SPI_CS_PIN);
    CAN.setSPI(customSPI);
    
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN BUS Shield init fail");
        delay(100);
    }
    
    // Setup interrupt
    pinMode(Pins::CAN_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(Pins::CAN_INT_PIN), handleInterrupt, FALLING);
    
    Serial.println("CAN init OK!");
}

void IRAM_ATTR CANManager::handleInterrupt() {
    messageAvailable = true;
}

void CANManager::update() {
    // Process any pending CAN messages
    if (messageAvailable) {
        processCANMessage();
        messageAvailable = false;
    }
    
    // Time-based message sending
    unsigned long currentTime = millis();
    
    // Fast cycle (10ms) - DMC messages
    if (currentTime - lastFastCycle >= Constants::FAST_CYCLE_MS) {
        lastFastCycle = currentTime;
        sendDMC();
    }
    
    // Slow cycle (50ms) - BSC and NLG messages
    if (currentTime - lastSlowCycle >= Constants::SLOW_CYCLE_MS) {
        lastSlowCycle = currentTime;
        sendBSC();
        sendNLG();
    }
}

void CANManager::processCANMessage() {
    uint8_t len;
    uint8_t buf[8];
    
    if (CAN.readMsgBuf(&len, buf) == CAN_OK) {
        uint32_t id = CAN.getCanId();
        
        switch(id) {
            case 0x001:  // BMS message
                processBMSMessage(buf);
                break;
                
            case 0x26A:  // BSC message
                processBSCMessage(buf);
                break;
                
            case 0x258:  // DMC Status
            case 0x259:  // DMC Power
            case 0x458:  // DMC Temperature
                processDMCMessage(id, buf);
                break;
                
            case CANIds::NLG_ACT_LIM:
            case CANIds::NLG_ACT_PLUG:
                processNLGMessage(id, buf);
                break;
        }
    }
}

void CANManager::processBMSMessage(uint8_t* buf) {
    bmsData.soc = buf[0] / 2;
    bmsData.voltage = (buf[1] | (buf[2] << 8)) / 100;
    bmsData.current = (buf[3] | (buf[4] << 8)) / 100;
    bmsData.maxDischarge = (buf[5] | (buf[6] << 8)) / 100;
    bmsData.maxCharge = buf[7] * 2;
}

void CANManager::processBSCMessage(uint8_t* buf) {
    bscData.hvVoltageAct = ((buf[0] << 8) | buf[1]) * 0.1;
    bscData.lvVoltageAct = buf[2] * 0.1;
    bscData.hvCurrentAct = (((buf[3] << 8) | buf[4]) * 0.1) - 25;
    bscData.lvCurrentAct = ((buf[5] << 8) | buf[6]) - 280;
    bscData.mode = buf[7] >> 4;
}

void CANManager::processDMCMessage(uint32_t id, uint8_t* buf) {
    switch(id) {
        case 0x258:  // Status message
            dmcData.ready = buf[0] & 0x80;
            dmcData.running = buf[0] & 0x40;
            dmcData.torqueAvailable = ((buf[2] << 8) | buf[3]) * 0.01;
            dmcData.torqueActual = ((buf[4] << 8) | buf[5]) * 0.01;
            dmcData.speedActual = static_cast<float>((buf[6] << 8) | buf[7]);
            break;
            
        case 0x259:  // Power message
            dmcData.dcVoltageAct = ((buf[0] << 8) | buf[1]) * 0.1;
            dmcData.dcCurrentAct = ((buf[2] << 8) | buf[3]) * 0.1;
            dmcData.acCurrentAct = ((buf[4] << 8) | buf[5]) * 0.25;
            dmcData.mechPower = ((buf[6] << 8) | buf[7]) * 16;
            break;
            
        case 0x458:  // Temperature message
            dmcData.tempInverter = ((buf[0] << 8) | buf[1]) * 0.5;
            dmcData.tempMotor = ((buf[2] << 8) | buf[3]) * 0.5;
            dmcData.tempSystem = buf[4] - 50;
            break;
    }
}

void CANManager::processNLGMessage(uint32_t id, uint8_t* buf) {
    switch(id) {
        case CANIds::NLG_ACT_LIM:
            nlgData.stateCtrlPilot = buf[0] >> 5;
            nlgData.dcHvVoltageAct = ((buf[0] & 0x1F) << 8) | buf[1];
            nlgData.stateAct = buf[2] >> 5;
            nlgData.dcHvCurrentAct = ((buf[2] & 0x07) << 8) | buf[3];
            nlgData.connectorLocked = (buf[7] >> 5) & 0x01;
            break;
            
        case CANIds::NLG_ACT_PLUG:
            nlgData.coolingRequest = buf[4];
            nlgData.tempCoolPlate = ((buf[6] << 8) | buf[7]) * 0.1;
            break;
    }
}

void CANManager::sendBSC() {
    // Scale values according to BSC protocol
    uint8_t lvVoltageScale = static_cast<uint8_t>(lvVoltage * 10);
    uint8_t hvVoltageScale = static_cast<uint8_t>((hvVoltage - 220));
    
    // BSC control message (0x260)
    controlBufferBSC[0] = (enableBSC << 0) | (modeBSC << 1) | 0x80;
    controlBufferBSC[1] = lvVoltageScale;
    controlBufferBSC[2] = hvVoltageScale;
    
    // BSC limits message (0x261)
    limitBufferBSC[0] = static_cast<uint8_t>(VehicleParams::Battery::MIN_VOLTAGE - 220);
    limitBufferBSC[1] = VehicleParams::Power::BSC_LV_BUCK;
    limitBufferBSC[2] = static_cast<uint8_t>(VehicleParams::Battery::PRECHARGE_CURRENT * 10);
    limitBufferBSC[3] = static_cast<uint8_t>(9 * 10); // 9V minimum
    limitBufferBSC[4] = VehicleParams::Power::BSC_LV_BOOST;
    limitBufferBSC[5] = static_cast<uint8_t>(VehicleParams::Battery::PRECHARGE_CURRENT * 10);
    
    // Send messages
    CAN.sendMsgBuf(CANIds::BSC_COMM, 0, 3, controlBufferBSC);
    CAN.sendMsgBuf(CANIds::BSC_LIM, 0, 6, limitBufferBSC);
}

void CANManager::sendDMC() {
    // Scale torque and speed values
    int16_t scaledTorque = static_cast<int16_t>(torqueDemand * 10);
    
    // DMC control message (0x210)
    controlBufferDMC[0] = (enableDMC << 7) | (false << 6) | (1 << 5) | (1 << 1) | 1;
    controlBufferDMC[2] = speedDemand >> 8;
    controlBufferDMC[3] = speedDemand & 0xFF;
    controlBufferDMC[4] = scaledTorque >> 8;
    controlBufferDMC[5] = scaledTorque & 0xFF;
    
    // DMC limits message (0x211)
    int dcVoltLimMotor = VehicleParams::Battery::MIN_VOLTAGE * 10;
    int dcVoltLimGen = VehicleParams::Battery::MAX_VOLTAGE * 10;
    int dcCurrLimMotor = VehicleParams::Battery::MAX_DMC_CURRENT * 10;
    int dcCurrLimGen = VehicleParams::Power::DMC_DC_GEN * 10;
    
    limitBufferDMC[0] = dcVoltLimMotor >> 8;
    limitBufferDMC[1] = dcVoltLimMotor & 0xFF;
    limitBufferDMC[2] = dcVoltLimGen >> 8;
    limitBufferDMC[3] = dcVoltLimGen & 0xFF;
    limitBufferDMC[4] = dcCurrLimMotor >> 8;
    limitBufferDMC[5] = dcCurrLimMotor & 0xFF;
    limitBufferDMC[6] = dcCurrLimGen >> 8;
    limitBufferDMC[7] = dcCurrLimGen & 0xFF;
    
    // Send messages
    CAN.sendMsgBuf(CANIds::DMCCTRL, 0, 8, controlBufferDMC);
    CAN.sendMsgBuf(CANIds::DMCLIM, 0, 8, limitBufferDMC);
}

void CANManager::sendNLG() {
    // Scale values for NLG
    int nlgVoltageScale = static_cast<int>(VehicleParams::Battery::MAX_VOLTAGE * 10);
    int nlgCurrentScale = static_cast<int>((VehicleParams::Battery::MAX_NLG_CURRENT + 102.4) * 10);
    
    controlBufferNLG[0] = (false << 7) | (nlgData.unlockRequest << 6) | 
                         (false << 5) | ((nlgVoltageScale >> 8) & 0x1F);
    controlBufferNLG[1] = nlgVoltageScale & 0xFF;
    controlBufferNLG[2] = (nlgData.stateDemand << 5) | ((nlgCurrentScale >> 8) & 0x07);
    controlBufferNLG[3] = nlgCurrentScale & 0xFF;
    controlBufferNLG[4] = (nlgData.ledDemand << 4) | ((nlgCurrentScale >> 8) & 0x07);
    controlBufferNLG[5] = nlgCurrentScale & 0xFF;
    
    CAN.sendMsgBuf(CANIds::NLG_DEM_LIM, 0, 8, controlBufferNLG);
}

void CANManager::resetMessageBuffers() {
    memset(controlBufferDMC, 0, 8);
    memset(controlBufferBSC, 0, 8);
    memset(limitBufferBSC, 0, 8);
    memset(limitBufferDMC, 0, 8);
    memset(controlBufferNLG, 0, 8);
}