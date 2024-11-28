#pragma once
#include <Arduino.h>

// Pin Definitions
namespace Pins {
    // SPI Pins
    constexpr uint8_t SCK = 4;
    constexpr uint8_t MOSI = 6;
    constexpr uint8_t MISO = 5;
    constexpr uint8_t SPI_CS_PIN = 36;
    constexpr uint8_t CAN_INT_PIN = 45;
    
    // Relay Pins
    constexpr uint8_t PUMP = 38;         // Cooling Pump PW0
    constexpr uint8_t PW1 = 39;
    constexpr uint8_t CONTACTOR = 11;    // HV Battery LPW0
    constexpr uint8_t NLGKL15 = 12;      // NLG KL15 LPW1
    constexpr uint8_t DMCKL15 = 13;      // DMC KL15 LWP2
    constexpr uint8_t BSCKL15 = 14;      // BSC KL15 LWP3
    constexpr uint8_t BCKLIGHT = 17;     // Reverse Signal LWP4
    constexpr uint8_t LWP5 = 18;
    constexpr uint8_t LWP6 = 21;
    constexpr uint8_t LWP7 = 16;
    
    // Input Pins
    constexpr uint8_t NLG_HW_Wakeup = 7;
    constexpr uint8_t IGNITION = 8;
    constexpr uint8_t UNLCKCON = 9;
    constexpr uint32_t BUTTON_PIN_BITMASK = 0x380;
}

// CAN Message IDs
namespace CANIds {
    // BSC Messages
    constexpr uint16_t BSC_COMM = 0x260;
    constexpr uint16_t BSC_LIM = 0x261;
    
    // DMC Messages
    constexpr uint16_t DMCCTRL = 0x210;
    constexpr uint16_t DMCLIM = 0x211;
    constexpr uint16_t DMCCTRL2 = 0x212;
    
    // NLG Messages
    constexpr uint16_t NLG_DEM_LIM = 0x711;
    constexpr uint16_t NLG_ACT_ERR = 0x799;
    constexpr uint16_t NLG_ACT_LIM = 0x728;
    constexpr uint16_t NLG_ACT_PLUG = 0x739;
}

// Timing Constants
namespace Constants {
    constexpr uint16_t FAST_CYCLE_MS = 10;
    constexpr uint16_t SLOW_CYCLE_MS = 50;
    constexpr uint16_t VERY_SLOW_CYCLE_MS = 100;
    constexpr uint32_t MODE_CHANGE_DELAY_MS = 2000;
    constexpr uint32_t UNLOCK_TIMEOUT_MS = 3000;
    constexpr uint32_t PRECHARGE_TIMEOUT_MS = 5000;
}

// BSC Operating Modes
namespace BSCModes {
    constexpr uint8_t BSC6_BUCK = 0;
    constexpr uint8_t BSC6_BOOST = 1;
}

// Charger States
namespace ChargerStates {
    // Actual States
    constexpr uint8_t NLG_ACT_SLEEP = 0;
    constexpr uint8_t NLG_ACT_WAKEUP = 1;
    constexpr uint8_t NLG_ACT_STANDBY = 2;
    constexpr uint8_t NLG_ACT_READY2CHARGE = 3;
    constexpr uint8_t NLG_ACT_CHARGE = 4;
    constexpr uint8_t NLG_ACT_SHUTDOWN = 5;
    
    // Demanded States
    constexpr uint8_t NLG_DEM_STANDBY = 0;
    constexpr uint8_t NLG_DEM_CHARGE = 1;
    constexpr uint8_t NLG_DEM_SLEEP = 6;
}
// Add to config.h, replacing previous enums

// ADC Configuration
namespace ADC {
    constexpr uint8_t GASPEDAL1 = 0;
    constexpr uint8_t GASPEDAL2 = 1;
    constexpr uint8_t REVERSE = 0;
    constexpr int MinValPot = 15568;
    constexpr int MaxValPot = 11200;
}

// Vehicle States
enum class VehicleState {
    STANDBY,
    RUN,
    CHARGING
};

// Gear States
enum class GearState {
    NEUTRAL,
    DRIVE,
    REVERSE
};

// Gear Ratios (Changed LOW to REDUCED)
enum class GearRatio {
    NORMAL,
    REDUCED
};

// Drive Modes (Changed DrivingMode to DriveMode)
enum class DriveMode {
    LEGACY,
    REGEN,
    OPD
};