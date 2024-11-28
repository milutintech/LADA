// Author: Christian Obrecht
// Description: Main entry point and core management for VCU

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

// Global instances
ADS1115 ads(0x48);
CANManager canManager(Pins::SPI_CS_PIN);
StateManager stateManager;
VehicleControl vehicleControl(ads);

// Task handles for dual core operation
TaskHandle_t canTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;

// System state variables
uint8_t vehicleMode = 0;  // 0=Standby, 1=Run, 2=Charging
uint8_t wakeupReason = 0;

// Create custom SPI instance
SPIClass *customSPI = nullptr;

// CAN Communication Task (Core 0)
void canTask(void* parameter) {
    Serial.print("CAN Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    // Initialize watchdog for this task
    esp_task_wdt_init(5, true);
    
    for(;;) {
        // Reset watchdog
        esp_task_wdt_init(5, true);
        
        // Handle CAN communication
        canManager.update();
        
        // Update vehicle control with latest motor data
        const DMCData& dmcData = canManager.getDMCData();
        vehicleControl.setMotorSpeed(dmcData.speedActual);
        
        // Calculate and apply torque demand if in RUN mode
        if (stateManager.getCurrentState() == VehicleState::RUN) {
            int16_t torque = vehicleControl.calculateTorque();
            canManager.setTorqueDemand(torque);
            canManager.setEnableDMC(vehicleControl.isDMCEnabled());
        }
        
        // Fast cycle delay (10ms)
        vTaskDelay(pdMS_TO_TICKS(Constants::FAST_CYCLE_MS));
    }
}

// Vehicle Control Task (Core 1)
void controlTask(void* parameter) {
    Serial.print("Control Task running on core: ");
    Serial.println(xPortGetCoreID());
    
    // Initialize watchdog for this task
    esp_task_wdt_init(5, true);
    
    // Initial wakeup handling
    stateManager.handleWakeup();
    
    for(;;) {
        // Reset watchdog
        esp_task_wdt_init(5, true);
        
        // Update state management
        stateManager.update();
        
        // Update system parameters from CAN data
        const BMSData& bmsData = canManager.getBMSData();
        const DMCData& dmcData = canManager.getDMCData();
        const NLGData& nlgData = canManager.getNLGData();
        
        // Update state manager with latest data
        stateManager.setBatteryVoltage(bmsData.voltage);
        stateManager.setInverterTemp(dmcData.tempInverter);
        stateManager.setMotorTemp(dmcData.tempMotor);
        stateManager.setCoolingRequest(nlgData.coolingRequest);
        
        // Update CAN manager with state information
        canManager.setEnableBSC(stateManager.isBatteryArmed());
        
        // Slow cycle delay (50ms)
        vTaskDelay(pdMS_TO_TICKS(Constants::SLOW_CYCLE_MS));
    }
}

void createTasks() {
    // Create CAN task on Core 0
    xTaskCreatePinnedToCore(
        canTask,          // Task function
        "CAN_Task",       // Task name
        10000,            // Stack size
        NULL,             // Parameters
        1,               // Priority
        &canTaskHandle,   // Task handle
        0                // Core ID (0)
    );
    
    // Create Control task on Core 1
    xTaskCreatePinnedToCore(
        controlTask,      // Task function
        "Control_Task",   // Task name
        20000,           // Stack size
        NULL,            // Parameters
        1,               // Priority
        &controlTaskHandle, // Task handle
        1                // Core ID (1)
    );
}

void setup() {
    // Initialize hardware
    SystemSetup::initializeSystem(
        ads,
        canManager,
        stateManager,
        vehicleControl,
        canTaskHandle,
        controlTaskHandle
    );

    // Create the tasks
    createTasks();

    Serial.println("Tasks created and system initialized!");
}

void loop() {
    // Empty - tasks handle everything
    vTaskDelete(NULL);
}
