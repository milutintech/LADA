#pragma once

namespace VehicleParams {
    // Battery Parameters
    struct Battery {
        static constexpr int MIN_VOLTAGE = 360;    // 3.2V * 104S
        static constexpr int NOM_VOLTAGE = 382;    // 3.67V * 104S
        static constexpr int MAX_VOLTAGE = 436;    // 4.2V * 104S
        static constexpr int MAX_DMC_CURRENT = 300;// A
        static constexpr int MAX_NLG_CURRENT = 72; // A
        static constexpr int PRECHARGE_CURRENT = 20;// A Current of BSC in boost mode
    };

    // Temperature Thresholds
    struct Temperature {
        static constexpr float INV_HIGH = 65.0f;   // Inverter high temperature limit
        static constexpr float MOT_HIGH = 80.0f;   // Motor high temperature limit
        static constexpr float INV_LOW = 40.0f;    // Inverter low temperature threshold
        static constexpr float MOT_LOW = 50.0f;    // Motor low temperature threshold
        static constexpr float NLG_MAX = 80.0f;    // Maximum NLG temperature
    };

    // Motor Parameters
    struct Motor {
        static constexpr int MAX_TRQ = 850;        // Maximum torque
        static constexpr int MAX_REQ_TRQ = 850;    // Maximum request torque
        static constexpr int MAX_REVERSE_TRQ = 220;// Maximum reverse torque
        static constexpr float MAX_ACCEL_STEP = 8.0f;  // Maximum acceleration step
        static constexpr float MAX_DECEL_STEP = 25.0f; // Maximum deceleration step
        static constexpr float TORQUE_DEADBAND_HIGH = 25.0f;  // Upper deadband threshold
        static constexpr float TORQUE_DEADBAND_LOW = 18.0f;   // Lower deadband threshold
    };

    // Regenerative Braking Parameters
    struct Regen {
        static constexpr float FADE_START = 400.0f;    // Speed where regen starts fading
        static constexpr float ZERO_SPEED = 0.5f;      // Speed considered as zero
        static constexpr float MIN_SPEED = 100.0f;     // Minimum speed for regen
        static constexpr float END_POINT = 35.0f;      // Pedal position where regen ends
        static constexpr float COAST_END = 40.0f;      // Pedal position where coast ends
    };

    // Power Management
    struct Power {
        static constexpr int DMC_DC_MOT = 600;     // DMC motor power limit
        static constexpr int DMC_DC_GEN = 420;     // DMC generator power limit
        static constexpr int BSC_LV_BUCK = 100;    // BSC buck converter power limit
        static constexpr int BSC_LV_BOOST = 100;   // BSC boost converter power limit
        static constexpr int NLG_MAX_AC = 32;      // Maximum NLG AC power
    };

    // Transmission Parameters
    struct Transmission {
        static constexpr float NORMAL_RATIO = 1.2f;    // Normal gear ratio
        static constexpr float REDUCED_RATIO = 2.1f;   // Reduced gear ratio
        static constexpr float DIFF_RATIO = 3.9f;      // Differential ratio
        static constexpr float WHEEL_CIRC = 2.08f;     // Wheel circumference in meters
        static constexpr float RPM_SHIFT_THRESHOLD = 100.0f; // RPM threshold for gear shifts
    };

    // Control Parameters
    struct Control {
        static constexpr float PEDAL_GAMMA = 1.5f;     // Pedal response curve
        static constexpr float SPEED_FACTOR = 1.2f;    // Speed scaling factor
        static constexpr float MIN_PEDAL_THRESHOLD = 2.0f;  // Minimum pedal threshold
        static constexpr float COAST_POSITION_MIN = 20.0f;  // Minimum coast pedal position
        static constexpr float COAST_POSITION_MAX = 50.0f;  // Maximum coast pedal position
    };

    // System Timing
    struct Timing {
        static constexpr unsigned long FAST_CYCLE_MS = 10;     // Fast control loop timing
        static constexpr unsigned long SLOW_CYCLE_MS = 50;     // Slow control loop timing
        static constexpr unsigned long NLG_UNLOCK_TIMEOUT = 3000;  // NLG unlock timeout
        static constexpr unsigned long PRECHARGE_TIMEOUT = 5000;   // Precharge timeout
    };
    
    // ADC Parameters
    struct ADC {
        static constexpr int MIN_POT_VALUE = 15568;    // Minimum potentiometer value
        static constexpr int MAX_POT_VALUE = 11200;    // Maximum potentiometer value
        static constexpr float POT_DEADBAND = 0.02f;   // Potentiometer deadband (2%)
    };

    // Vehicle Limits
    struct Limits {
        static constexpr float MAX_SPEED = 120.0f;     // Maximum vehicle speed in kph
        static constexpr float MAX_REVERSE_SPEED = 20.0f;  // Maximum reverse speed in kph
        static constexpr int MAX_MOTOR_RPM = 8000;     // Maximum motor RPM
        static constexpr float MAX_ACCELERATION = 3.0f; // Maximum acceleration m/s²
    };
};
