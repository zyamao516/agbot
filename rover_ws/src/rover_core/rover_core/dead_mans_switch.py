
class Robot_Enable:

    def __init__(self):
        self.board_id = ""
        self.contactorPin = 7  # Pin number used for contactor control
        self.state = False

        # Raspberry Pi (lgpio) setup
        try:
            import lgpio
            self.lgpio = lgpio  # Store lgpio module for later use
            self.h = self.lgpio.gpiochip_open(0)  # Open GPIO chip and store handler
            self.lgpio.gpio_claim_output(self.h, self.contactorPin)  # Claim the pin as output
            self.board_id = "pi"  # Set board ID to 'pi' for Raspberry Pi
            print("Raspberry Pi GPIO setup completed.")
        except Exception as e:
            print(f"Failed to initialize Raspberry Pi GPIO: {e}")
            self.h = None

        # NVIDIA Jetson (Jetson.GPIO) setup
        try:
            import Jetson.GPIO as GPIO
            self.GPIO = GPIO  # Store Jetson.GPIO module for later use
            self.GPIO.setmode(self.GPIO.BOARD)
            self.GPIO.setup(self.contactorPin, self.GPIO.OUT, initial=self.GPIO.LOW)
            self.board_id = "jetson"  # Set board ID to 'jetson' for NVIDIA Jetson
            print("Jetson GPIO setup completed.")
        except Exception as e:
            print(f"Failed to initialize Jetson GPIO: {e}")
            self.GPIO = None

    def contactor_ctrl(self, val):
        if self.state == val:
            return
        self.state = val
        # Raspberry Pi control
        if self.board_id == "pi" and self.h is not None:
            self.lgpio.gpio_write(self.h, self.contactorPin, val)
        
        # Jetson control
        elif self.board_id == "jetson" and self.GPIO is not None:
            self.GPIO.output(self.contactorPin, self.GPIO.HIGH if val else self.GPIO.LOW)
        
        # No valid board detected
        else:
            print("Contactor control disabled. No compatible board detected.")
