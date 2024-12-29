# CAN Bus Speedometer Utility

A Python-based desktop utility for CAN bus communication with a speedometer system using ESP32-S3 CAN adapter.

## Features

- Multi-platform support (Windows, macOS, Linux)
- Support for multiple CAN interfaces:
  - ESP32-S3 CAN Adapter (Custom)
  - SocketCAN (Linux)
  - PCAN-USB
- Real-time CAN message monitoring
- LED status indicators for adapter state
- Fixed window size for consistent UI experience

## Installation

1. Clone the repository
2. Install dependencies:
```bash
pip install PySide6 pyserial
```

## Usage

Run the utility:
```bash
python main.py
```

## CAN Message Specification

### Transmit Messages (Host → Speedometer)

#### Drive Status (ID: 0x200)
- Drive Mode Selection
  - P = 0
  - R = 1
  - N = 2
  - D = 3
  - S = 4
- Temperature: 30-110°C
- Reserved bytes for future use

#### Performance Data (ID: 0x201)
- Speed: 0-220 km/h (scaled * 10)
- Torque: -200 to 800 Nm
- Reserved bytes for future use

### Receive Messages (Speedometer → Host)

#### BMS Data (ID: 0x1)
- Byte 0: BMS_SOC (0-100%)
- Bytes 1-2: BMS_U_BAT (0-450V)
- Bytes 3-4: BMS_I_BAT (0-650A)
- Bytes 5-6: BMS_MAX_Discharge (0-650A)
- Byte 7: BMS_MAX_Charge (0-250A)

#### Info Messages (ID: 0x3)
- Bytes 1-2: DMC_TrqRq (0-1100Nm)
- Byte 3: NLG_AcCurrLimMax (0-32A)
- Byte 4: OFFROAD_MODE (0-1)
- Bytes 5-8: Reserved

#### Odometer Data (ID: 0x300)
- Total kilometers driven
- 4-byte integer value

#### Status Flags (ID: 0x301)
16-bit flag register:
- Bit 0: Check Engine
- Bit 1: Window Heating
- Bit 2: Rear Fog Light
- Bit 3: High Beam
- Bit 4: Normal Driving Lights
- Bit 5: Indicator Enabled
- Bit 6: Battery Low (12V)
- Bit 7: Brake System Malfunction
- Bit 8: Fluid/Oil Low
- Bit 9: Diflock
- Bit 10: Fuel Light / HV Battery Light
- Bit 11: Seat Buckle Light
- Bit 12: Parking Brake Enabled
- Bits 13-15: Reserved

## ESP32-S3 CAN Adapter Protocol

The utility communicates with the ESP32-S3 CAN adapter using the following commands:

- 0x01: Initialize CAN
- 0x02: Read CAN message
- 0x03: Write CAN message

Serial Configuration:
- Baud Rate: 115200
- Data Bits: 8
- Stop Bits: 1
- Parity: None

## LED Status Indicators

The utility displays the following adapter states:
- White: Ready/Idle
- Red: Error
- Yellow: Processing
- Green: Transmitting
- Blue: Receiving

## Project Structure

```
project/
├── main.py          # Main utility implementation
├── can_defs.py      # CAN message definitions and structures
└── requirements.txt  # Python dependencies
```

## Development Notes for Speedometer Implementation

For speedometer software development, consider:

1. Message Priorities:
   - Status flags (0x301) - High priority for warning lights
   - Performance data (0x201) - High priority for speed/torque display
   - BMS data (0x1) - Medium priority for SOC/voltage display
   - Info messages (0x3) - Lower priority for auxiliary information

2. Update Rates:
   - Speed/Torque: 100ms refresh recommended
   - Status Flags: 100ms refresh recommended
   - BMS Data: 500ms refresh acceptable
   - Odometer: 1s refresh acceptable

3. Display Considerations:
   - Speed should be the primary focus
   - Warning lights should be immediately visible
   - SOC should be clearly visible but not dominate
   - Temperature warnings should trigger visual alerts

## Contributing

Feel free to submit issues and pull requests to improve the utility.

## License

[Add your license information here]