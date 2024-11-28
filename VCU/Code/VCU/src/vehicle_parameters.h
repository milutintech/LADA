#pragma once

// Motor Parameters
#define DMC_MAXTRQ 850         // Maximum motor torque (Nm)
#define DMC_MAXREQTRQ 850      // Maximum requested torque (Nm)
#define MAX_REVERSE_TRQ 220    // Maximum reverse torque (Nm)

// Torque Control Parameters
#define TORQUE_DEADBAND_HIGH 25  // Upper deadband threshold
#define TORQUE_DEADBAND_LOW 18   // Lower deadband threshold
#define MAX_ACCEL_STEP 8.0f     // Maximum acceleration rate change
#define MAX_DECEL_STEP 25.0f    // Maximum deceleration rate change

// Regen Parameters
#define REGEN_FADE_START 400.0f
#define ZERO_SPEED_WINDOW 0.5f
#define MIN_SPEED_FOR_REGEN 100.0f
#define REGEN_END_POINT 35.0f
#define COAST_END_POINT 40.0f

// Speed and Gear Parameters
#define RPM_SHIFT_THRESHOLD 100
#define FORWARD_SWITCH_THRESHOLD 200
#define REVERSE_SWITCH_THRESHOLD 200

// Temperature Thresholds
#define TEMP_INV_HIGH 65.0f
#define TEMP_MOT_HIGH 80.0f
#define TEMP_INV_LOW 40.0f
#define TEMP_MOT_LOW 50.0f

// Current Limits
#define DMC_DcCLimMot 600      // Driving current limit
#define DMC_DcCLimGen 420      // Regen current limit
#define DMC_TrqSlewrate 150    // Torque slew rate
#define DMC_SpdSlewrate 655    // Speed slew rate
#define DMC_MechPwrMaxMot 20000 // Maximum mechanical power motor
#define DMC_MechPwrMaxGen 20000 // Maximum mechanical power generator

// BSC Parameters
#define BSC6_LVCUR_UPLIM_BUCK 100
#define BSC6_LVCUR_UPLIM_BOOST 100
#define BSC6_HVVOL_LOWLIM MIN_U_BAT

// NLG Parameters (Charger)
#define NLG_UNLOCK_TIMEOUT 3000    // Connector unlock timeout (ms)
#define NLG_MAX_AC_CURRENT 32      // Maximum AC current
#define NLG_MAX_TEMP 80            // Maximum temperature