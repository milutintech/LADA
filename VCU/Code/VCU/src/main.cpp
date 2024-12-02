#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include <esp_adc_cal.h>
#include <esp32-hal-adc.h>
#include "ADS1X15.h"
#include "AD5593R.h"

#include "state_manager.h"
#include "can_manager.h"
#include "vehicle_control.h"
#include "setup.h"
#include "config.h"
#include "SerialConsole.h"

// Global instances
ADS1115 ads(0x48);
CANManager canManager(Pins::SPI_CS_PIN);
StateManager stateManager;
VehicleControl vehicleControl(ads);
SerialConsole serialConsole(canManager, stateManager, vehicleControl);

TaskHandle_t canTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;

void canTask(void* parameter) {
    Serial.print("CAN Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    // Initialize CAN and ADC hardware
    SPI.begin(Pins::SCK, Pins::MISO, Pins::MOSI, Pins::SPI_CS_PIN);
    Wire.begin();
    Wire.setClock(400000);
    ads.begin();
    ads.setGain(2);
    canManager.begin();
    
    esp_task_wdt_init(5, true);
    
    for(;;) {
        esp_task_wdt_init(5, true);
        canManager.update();
        
        const DMCData& dmcData = canManager.getDMCData();
        vehicleControl.setMotorSpeed(dmcData.speedActual);
        
        if (stateManager.getCurrentState() == VehicleState::RUN) {
            int16_t torque = vehicleControl.calculateTorque();
            canManager.setTorqueDemand(torque);
            canManager.setEnableDMC(vehicleControl.isDMCEnabled());
        }
        
        vTaskDelay(pdMS_TO_TICKS(Constants::FAST_CYCLE_MS));
    }
}

void controlTask(void* parameter) {
    Serial.print("Control Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    esp_task_wdt_init(5, true);
    
    stateManager.handleWakeup();
    
    for(;;) {
        esp_task_wdt_init(5, true);
        
        stateManager.update();
        serialConsole.update();
        
        const BMSData& bmsData = canManager.getBMSData();
        const DMCData& dmcData = canManager.getDMCData();
        const NLGData& nlgData = canManager.getNLGData();
        
        stateManager.setBatteryVoltage(bmsData.voltage);
        stateManager.setInverterTemp(dmcData.tempInverter);
        stateManager.setMotorTemp(dmcData.tempMotor);
        stateManager.setCoolingRequest(nlgData.coolingRequest);
        
        canManager.setEnableBSC(stateManager.isBatteryArmed());
        
        vTaskDelay(pdMS_TO_TICKS(Constants::SLOW_CYCLE_MS));
    }
}

void setup() {
    Serial.begin(115200);
    SystemSetup::initializeGPIO();
    SystemSetup::initializeSleep();
    
    xTaskCreatePinnedToCore(canTask, "CAN_Task", 10000, NULL, 1, &canTaskHandle, 0);
    xTaskCreatePinnedToCore(controlTask, "Control_Task", 20000, NULL, 1, &controlTaskHandle, 1);
}

void loop() {
    vTaskDelete(NULL);
}