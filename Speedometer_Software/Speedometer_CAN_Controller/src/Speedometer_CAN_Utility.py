import sys
import platform
from typing import Optional, Union
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QComboBox, QPushButton, QLabel, 
                             QGroupBox, QSpinBox, QDoubleSpinBox, QCheckBox)
from PySide6.QtCore import Qt, QTimer

# Import our CAN definitions
from can_defs import CANIds, BMSData, StatusFlags, InfoMessage, DriveMode


class CANInterface:
    """Abstract base class for CAN interfaces"""
    def __init__(self):
        self.connected = False
        self.bus = None
    
    def connect(self) -> bool:
        raise NotImplementedError
        
    def disconnect(self) -> None:
        if self.bus:
            self.bus.shutdown()
        self.connected = False
        
    def send_message(self, can_id: int, data: bytes) -> bool:
        raise NotImplementedError
        
    def receive_message(self) -> tuple[int, bytes]:
        raise NotImplementedError

class SocketCANInterface(CANInterface):
    """Linux SocketCAN implementation"""
    def __init__(self, interface='can0'):
        super().__init__()
        self.interface = interface
        
    def connect(self) -> bool:
        try:
            import socket
            import struct
            self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.socket.bind((self.interface,))
            self.connected = True
            return True
        except Exception as e:
            print(f"Failed to connect to SocketCAN: {e}")
            return False

    def disconnect(self) -> None:
        if self.connected:
            self.socket.close()
            self.connected = False

    def send_message(self, can_id: int, data: bytes) -> bool:
        if not self.connected:
            return False
        try:
            message = struct.pack("=IB3x8s", can_id, len(data), data)
            self.socket.send(message)
            return True
        except Exception as e:
            print(f"Failed to send CAN message: {e}")
            return False

class CustomESP32Interface(CANInterface):
    """Custom ESP32-S3 CAN adapter implementation"""
    def __init__(self, port: str):
        super().__init__()
        self.port = port
        self.serial = None
        
    def connect(self) -> bool:
        try:
            import serial
            print(f"Attempting to connect to port: {self.port}")
            self.serial = serial.Serial(
                port=self.port,
                baudrate=115200,
                timeout=1.0
            )
            
            print("Serial port opened successfully")
            
            # Initialize CAN (Command 0x01)
            print("Sending CAN initialization command")
            self.serial.write(bytes([0x01]))
            
            # Wait for initialization response
            response = self.serial.readline().decode().strip()
            print(f"Received initialization response: {response}")
            
            if "CAN initialization complete" in response:
                self.connected = True
                return True
            return False
            
        except Exception as e:
            print(f"Failed to connect to ESP32-S3 CAN adapter: {e}")
            print(f"Port: {self.port}")
            print(f"Available ports: {[port.device for port in serial.tools.list_ports.comports()]}")
            return False

    def disconnect(self) -> None:
        if self.serial:
            self.serial.close()
        self.connected = False

    def send_message(self, can_id: int, data: bytes) -> bool:
        if not self.connected or not self.serial:
            return False
        try:
            # Command 0x03 for write, followed by ID and data
            msg = bytes([0x03])  # Write command
            msg += can_id.to_bytes(2, 'big')  # 2 bytes for CAN ID
            msg += bytes([len(data)])  # Data length
            msg += data  # Data bytes
            self.serial.write(msg)
            return True
        except Exception as e:
            print(f"Failed to send CAN message: {e}")
            return False
            
    def receive_message(self) -> tuple[int, bytes]:
        if not self.connected or not self.serial:
            return (0, b'')
        try:
            # Send read command (0x02)
            self.serial.write(bytes([0x02]))
            
            # Check if there's data available
            if self.serial.in_waiting:
                # Read message header (3 bytes: ID MSB, ID LSB, length)
                header = self.serial.read(3)
                if len(header) == 3:
                    can_id = int.from_bytes(header[:2], 'big')
                    length = header[2]
                    
                    # Read data bytes
                    data = self.serial.read(length)
                    if len(data) == length:
                        return (can_id, data)
            
            return (0, b'')
        except Exception as e:
            print(f"Error receiving CAN message: {e}")
            return (0, b'')

class PCANInterface(CANInterface):
    """PCAN-USB implementation"""
    def __init__(self):
        super().__init__()
        self.pcan = None
        
    def connect(self) -> bool:
        try:
            import PCANBasic
            self.pcan = PCANBasic.PCANBasic()
            result = self.pcan.Initialize(PCANBasic.PCAN_USBBUS1, PCANBasic.PCAN_BAUD_500K)
            self.connected = result == PCANBasic.PCAN_ERROR_OK
            return self.connected
        except Exception as e:
            print(f"Failed to connect to PCAN: {e}")
            return False

    def disconnect(self) -> None:
        if self.connected and self.pcan:
            self.pcan.Uninitialize(PCANBasic.PCAN_USBBUS1)
            self.connected = False

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN Speedometer Utility")
        # Set fixed window size
        self.setFixedSize(800, 600)  # Width: 800px, Height: 600px
        self.setWindowFlag(Qt.WindowType.MSWindowsFixedSizeDialogHint)  # Disables resize handles on Windows
        self.can_interface: Optional[CANInterface] = None
        self.setup_ui()
        
        # Update timer for receiving CAN messages
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_received_data)
        self.timer.start(100)  # 100ms update interval

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # CAN Interface Selection
        interface_group = QGroupBox("CAN Interface")
        interface_layout = QHBoxLayout()
        self.interface_combo = QComboBox()
        self.detect_interfaces()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        interface_layout.addWidget(self.interface_combo)
        interface_layout.addWidget(self.connect_button)
        interface_group.setLayout(interface_layout)
        layout.addWidget(interface_group)

        # Transmit Group
        transmit_group = QGroupBox("Transmit Data")
        transmit_layout = QVBoxLayout()
        
        # Drive Mode
        drive_layout = QHBoxLayout()
        drive_layout.addWidget(QLabel("Drive Mode:"))
        self.drive_mode = QComboBox()
        self.drive_mode.addItems(['P', 'R', 'N', 'D', 'S'])
        drive_layout.addWidget(self.drive_mode)
        transmit_layout.addLayout(drive_layout)
        
        # SOC, Temperature, Speed, Torque
        for name, min_val, max_val in [
            ("State of Charge (%)", 0, 100),
            ("Temperature (°C)", 30, 110),
            ("Speed (km/h)", 0, 220),
            ("Torque (Nm)", -200, 800)
        ]:
            row_layout = QHBoxLayout()
            row_layout.addWidget(QLabel(f"{name}:"))
            spinbox = QSpinBox()
            spinbox.setRange(min_val, max_val)
            setattr(self, name.split()[0].lower() + "_spin", spinbox)
            row_layout.addWidget(spinbox)
            transmit_layout.addLayout(row_layout)
        
        transmit_group.setLayout(transmit_layout)
        layout.addWidget(transmit_group)

        # Receive Group
        receive_group = QGroupBox("Received Data")
        receive_layout = QVBoxLayout()
        
        # Total KM
        km_layout = QHBoxLayout()
        km_layout.addWidget(QLabel("Total KM:"))
        self.total_km = QLabel("0")
        km_layout.addWidget(self.total_km)
        receive_layout.addLayout(km_layout)
        
        # Status Flags
        flags_layout = QVBoxLayout()
        self.status_flags = {}
        for flag in [
            "Check Engine", "Window Heating", "Rear Fog Light", "High Beam",
            "Normal Lights", "Indicator", "Battery Low", "Brake System",
            "Fluid Low", "Diflock", "Battery Light", "Seat Buckle",
            "Parking Brake"
        ]:
            checkbox = QCheckBox(flag)
            checkbox.setEnabled(False)
            self.status_flags[flag] = checkbox
            flags_layout.addWidget(checkbox)
        receive_layout.addLayout(flags_layout)
        
        receive_group.setLayout(receive_layout)
        layout.addWidget(receive_group)

    def detect_interfaces(self):
        self.interface_combo.clear()
        
        # Detect ESP32-S3 CAN Adapter ports
        import serial.tools.list_ports
        
        # Add all available serial ports
        for port in serial.tools.list_ports.comports():
            # Look for ESP32-S3 CAN Adapter by VID:PID
            if port.vid == 0x303A and port.pid == 0x1001:
                if platform.system() == "Darwin":
                    # On macOS, store the full device path
                    self.interface_combo.addItem(f"ESP32-S3 CAN: {port.device}")
                else:
                    self.interface_combo.addItem(f"ESP32-S3 CAN: {port.name}")
            # Also add other potential serial ports as fallback
            elif platform.system() == "Windows" and port.name.startswith("COM"):
                self.interface_combo.addItem(f"Serial: {port.name}")
            elif platform.system() == "Darwin" and port.device.startswith("/dev/tty.usbmodem"):
                self.interface_combo.addItem(f"Serial: {port.device}")
            elif platform.system() == "Linux" and port.name.startswith("/dev/ttyACM"):
                self.interface_combo.addItem(f"Serial: {port.name}")
                
        # Add SocketCAN interfaces for Linux
        if platform.system() == "Linux":
            import os
            if os.path.exists('/sys/class/net'):
                for iface in os.listdir('/sys/class/net'):
                    if iface.startswith('can'):
                        self.interface_combo.addItem(f"SocketCAN: {iface}")
        
        # Add PCAN-USB interfaces
        try:
            import PCANBasic
            self.interface_combo.addItem("PCAN-USB")
        except ImportError:
            pass

    def toggle_connection(self):
        if not self.can_interface or not self.can_interface.connected:
            self.connect_to_can()
        else:
            self.disconnect_from_can()

    def connect_to_can(self):
        interface_text = self.interface_combo.currentText()
        
        if interface_text.startswith("ESP32-S3 CAN") or interface_text.startswith("Serial"):
            port_name = interface_text.split(": ")[1]
            # Add /dev/ prefix for macOS if not present
            if platform.system() == "Darwin" and not port_name.startswith("/dev/"):
                port_name = f"/dev/{port_name}"
            self.can_interface = CustomESP32Interface(port_name)
        elif interface_text.startswith("SocketCAN"):
            interface_name = interface_text.split(": ")[1]
            self.can_interface = SocketCANInterface(interface_name)
        elif interface_text == "PCAN-USB":
            self.can_interface = PCANInterface()
            
        if self.can_interface and self.can_interface.connect():
            self.connect_button.setText("Disconnect")
            self.add_status_indicators()
            print("Connected to CAN interface")
        else:
            print("Failed to connect to CAN interface")

    def add_status_indicators(self):
        """Add LED status indicators to the UI"""
        if not hasattr(self, 'status_layout'):
            status_group = QGroupBox("Adapter Status")
            status_layout = QHBoxLayout()
            
            # Create colored indicators
            self.led_indicators = {}
            for color, desc in [
                ("white", "Ready"),
                ("red", "Error"),
                ("yellow", "Processing"),
                ("green", "Transmitting"),
                ("blue", "Receiving")
            ]:
                led = QLabel("⬤")  # Unicode circle
                led.setStyleSheet(f"color: {color};")
                label = QLabel(desc)
                indicator_layout = QVBoxLayout()
                indicator_layout.addWidget(led)
                indicator_layout.addWidget(label)
                status_layout.addLayout(indicator_layout)
                self.led_indicators[color] = led
            
            status_group.setLayout(status_layout)
            self.layout().insertWidget(1, status_group)  # Add after interface selection

    def disconnect_from_can(self):
        if self.can_interface:
            self.can_interface.disconnect()
            self.connect_button.setText("Connect")
            print("Disconnected from CAN interface")

    def update_received_data(self):
        if not self.can_interface or not self.can_interface.connected:
            return
            
        try:
            can_id, data = self.can_interface.receive_message()
            
            if can_id == CANIds.RX_ODOMETER:
                # Update total KM display
                km = int.from_bytes(data[:4], 'big')
                self.total_km.setText(str(km))
                
            elif can_id == CANIds.RX_STATUS_FLAGS:
                # Update status flags
                flags = int.from_bytes(data[:2], 'big')
                for i, (name, checkbox) in enumerate(self.status_flags.items()):
                    checkbox.setChecked(bool(flags & (1 << i)))
                    
            elif can_id == CANIds.RX_INFO:
                # Handle info message
                max_soc = data[0]
                dmc_trq = int.from_bytes(data[1:3], 'big')
                ac_curr = data[3]
                offroad = bool(data[4])
                # Update UI elements as needed
                
        except Exception as e:
            print(f"Error receiving CAN message: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())