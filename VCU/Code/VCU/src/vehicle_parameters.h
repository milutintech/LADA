// vehicle_parameters.h
#pragma once

namespace VehicleParams {
    struct Temperature {
        static constexpr float INV_HIGH = 65.0f;
        static constexpr float MOT_HIGH = 80.0f;
        static constexpr float INV_LOW = 40.0f;
        static constexpr float MOT_LOW = 50.0f;
        static constexpr float NLG_MAX = 80.0f;
    };

    struct Motor {
        static constexpr int MAX_TRQ = 850;
        static constexpr int MAX_REQ_TRQ = 850;
        static constexpr int MAX_REVERSE_TRQ = 220;
        static constexpr float MAX_ACCEL_STEP = 8.0f;
        static constexpr float MAX_DECEL_STEP = 25.0f;
        static constexpr float TORQUE_DEADBAND_HIGH = 25.0f;
        static constexpr float TORQUE_DEADBAND_LOW = 18.0f;
    };

    struct Regen {
        static constexpr float FADE_START = 400.0f;
        static constexpr float ZERO_SPEED = 0.5f;
        static constexpr float MIN_SPEED = 100.0f;
        static constexpr float END_POINT = 35.0f;
        static constexpr float COAST_END = 40.0f;
    };

    struct Power {
        static constexpr int DMC_DC_MOT = 600;
        static constexpr int DMC_DC_GEN = 420;
        static constexpr int BSC_LV_BUCK = 100;
        static constexpr int BSC_LV_BOOST = 100;
        static constexpr int NLG_MAX_AC = 32;
        static constexpr int MIN_U_BAT = 360;
        static constexpr int NOM_U_BAT = 382;
        static constexpr int MAX_U_BAT = 436;
        static constexpr int MAX_DMC_CURRENT = 300;
        static constexpr int MAX_NLG_CURRENT = 72;
        static constexpr int PRECHARGE_CURRENT = 20;
    };

    struct Transmission {
        static constexpr float NORMAL_RATIO = 1.2f;
        static constexpr float REDUCED_RATIO = 2.1f;
        static constexpr float DIFF_RATIO = 3.9f;
        static constexpr float WHEEL_CIRC = 2.08f;
        static constexpr float RPM_SHIFT_THRESHOLD = 100.0f;
    };

    struct Timing {
        static constexpr unsigned long FAST_CYCLE_MS = 10;
        static constexpr unsigned long SLOW_CYCLE_MS = 50;
        static constexpr unsigned long NLG_UNLOCK_TIMEOUT = 3000;
    };
};