from can import Message
from can import can



with can.Bus(interface='socketcan',
              channel='vcan0',
              receive_own_messages=True) as bus:
    
    print(Message(is_extended_id=False, arbitration_id=100))