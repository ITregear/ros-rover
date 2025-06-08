#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

def test_gpio():
    try:
        # Clean up any existing setup
        GPIO.cleanup()
        
        # Set mode to BCM
        GPIO.setmode(GPIO.BCM)
        print("GPIO mode set to BCM")
        
        # Test pins
        test_pins = [20, 21]
        
        # Setup pins
        for pin in test_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            print(f"Pin {pin} setup complete")
            
            # Read pin state
            state = GPIO.input(pin)
            print(f"Pin {pin} state: {state}")
            
            # Try to add event detection
            GPIO.add_event_detect(pin, GPIO.BOTH, 
                                callback=lambda x: print(f"Pin {pin} changed"), 
                                bouncetime=10)
            print(f"Event detection added for pin {pin}")
        
        print("All tests completed successfully")
        
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    test_gpio() 