import RPi.GPIO as GPIO
import time

# Set up GPIO mode and pin
raspi_pin = 23  # GPIO pin number you want to use

def edge_callback(channel):
    if GPIO.input(channel):  # Rising edge detected
        print("Rising edge detected on GPIO pin", channel)
    else:  # Falling edge detected
        print("Falling edge detected on GPIO pin", channel)

try:
    # Set up GPIO mode
    GPIO.setmode(GPIO.BCM)

    # Set up GPIO pin as input with internal pull-down resistor
    GPIO.setup(raspi_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Add event detection for both rising and falling edges
    GPIO.add_event_detect(raspi_pin, GPIO.BOTH, callback=edge_callback, bouncetime=200)

    print("Edge detection enabled. Press Ctrl+C to exit.")

    # Keep the script running
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    # Clean up GPIO
    GPIO.remove_event_detect(raspi_pin)
    GPIO.cleanup()