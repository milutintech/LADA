from dataclasses import dataclass
from enum import Enum, auto

class DriveMode(Enum):
    P = auto()
    R = auto()
    N = auto()
    D = auto()
    S = auto()

@dataclass
class CANIds:
    # Transmit Messages
    TX_DRIVE_STATUS = 0x200  # Drive mode, Temperature
    TX_PERFORMANCE = 0x201   # Speed and Torque

    # Receive Messages
    RX_ODOMETER = 0x300      # Total KM
    RX_STATUS_FLAGS = 0x301  # Various indicator flags
    RX_INFO = 0x3           # Info messages
    RX_BMS = 0x1            # BMS data including SOC

@dataclass
class BMSData:
    stateOfCharge: int      # 0-100%
    batteryVoltage: float   # 0-450V
    batteryCurrent: float   # 0-650A
    maxDischarge: float     # 0-650A
    maxCharge: float        # 0-250A

    @staticmethod
    def parse_message(data: bytes) -> 'BMSData':
        soc = data[0]
        voltage = int.from_bytes(data[1:3], 'big') * (450.0 / 65535.0)
        current = int.from_bytes(data[3:5], 'big') * (650.0 / 65535.0)
        max_discharge = int.from_bytes(data[5:7], 'big') * (650.0 / 65535.0)
        max_charge = data[7] * (250.0 / 255.0)
        
        return BMSData(soc, voltage, current, max_discharge, max_charge)

@dataclass
class StatusFlags:
    checkEngine: bool = False
    windowHeating: bool = False
    rearFogLight: bool = False
    highBeam: bool = False
    normalLights: bool = False
    indicator: bool = False
    batteryLow: bool = False
    brakeSystem: bool = False
    fluidLow: bool = False
    difLock: bool = False
    batteryLight: bool = False
    seatBuckle: bool = False
    parkingBrake: bool = False

    @staticmethod
    def parse_message(data: bytes) -> 'StatusFlags':
        flags = StatusFlags()
        value = int.from_bytes(data[:2], 'big')
        
        flags.checkEngine = bool(value & (1 << 0))
        flags.windowHeating = bool(value & (1 << 1))
        flags.rearFogLight = bool(value & (1 << 2))
        flags.highBeam = bool(value & (1 << 3))
        flags.normalLights = bool(value & (1 << 4))
        flags.indicator = bool(value & (1 << 5))
        flags.batteryLow = bool(value & (1 << 6))
        flags.brakeSystem = bool(value & (1 << 7))
        flags.fluidLow = bool(value & (1 << 8))
        flags.difLock = bool(value & (1 << 9))
        flags.batteryLight = bool(value & (1 << 10))
        flags.seatBuckle = bool(value & (1 << 11))
        flags.parkingBrake = bool(value & (1 << 12))
        
        return flags

@dataclass
class InfoMessage:
    dmcTrqRq: int         # 0-1100Nm
    nlgAcCurrMax: int     # 0-32A
    offroadMode: bool     # 0-1

    @staticmethod
    def parse_message(data: bytes) -> 'InfoMessage':
        trq = int.from_bytes(data[1:3], 'big')
        curr = data[3]
        mode = bool(data[4])
        
        return InfoMessage(trq, curr, mode)