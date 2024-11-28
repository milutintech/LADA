#pragma once
#include <cstdint>

enum class VehicleMode {
    Standby = 0,
    Run = 1,
    Charging = 2
};

enum class Gear {
    Neutral,
    Drive,
    Reverse
};

enum class DrivingMode {
    LEGACY,
    REGEN,
    OPD
};

// NLG States
#define NLG_ACT_SLEEP 0
#define NLG_ACT_WAKEUP 1
#define NLG_ACT_STANDBY 2
#define NLG_ACT_READY2CHARGE 3
#define NLG_ACT_CHARGE 4
#define NLG_ACT_SHUTDOWN 5

#define NLG_DEM_STANDBY 0
#define NLG_DEM_CHARGE 1
#define NLG_DEM_SLEEP 6

// BSC Modes
#define BSC6_BUCK 0
#define BSC6_BOOST 1