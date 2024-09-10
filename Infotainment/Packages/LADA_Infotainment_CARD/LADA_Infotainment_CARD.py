import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

class CarControlRFID:
    def __init__(self):
        self.reader = SimpleMFRC522()

    def write_new_card(self, card_id, valet_mode, max_torque, offroad_mode):
        """
        Write new card with the specified settings.
        
        :param card_id: Identifier for the card.
        :param valet_mode: True or False for valet mode.
        :param max_torque: Maximum torque in NM.
        :param offroad_mode: True or False for offroad mode.
        """
        try:
            data = f"{card_id},{valet_mode},{max_torque},{offroad_mode}"
            print("Place the card to write")
            self.reader.write(data)
            print("Data written to card:", data)
        except Exception as e:
            print("Failed to write to card:", str(e))
        finally:
            GPIO.cleanup()

    def read_card(self):
        """
        Read data from the card.
        
        :return: Dictionary with card settings.
        """
        try:
            print("Place the card to read")
            id, text = self.reader.read()
            print("Card read successfully")
            data = text.split(',')
            card_info = {
                "card_id": data[0],
                "valet_mode": data[1].lower() == 'true',
                "max_torque": int(data[2]),
                "offroad_mode": data[3].lower() == 'true'
            }
            return card_info
        except Exception as e:
            print("Failed to read card:", str(e))
            return None
        finally:
            GPIO.cleanup()

# Usage example
if __name__ == "__main__":
    car_rfid = CarControlRFID()
    
    # Write to a new card
    car_rfid.write_new_card("CAR12345", True, 250, False)
    
    # Read from a card
    card_data = car_rfid.read_card()
    if card_data:
        print("Card Data:", card_data)