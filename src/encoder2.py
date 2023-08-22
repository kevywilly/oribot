#!/usr/bin/python3

import time
import Jetson.GPIO as GPIO

# GPIO pins for the encoder outputs
ENCODER_PIN_A = 18
ENCODER_PIN_B = 16

# Configure GPIO mode and setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ENCODER_PIN_A, GPIO.IN)
GPIO.setup(ENCODER_PIN_B, GPIO.IN)

# Variables for tracking ticks and time
tick_count = 0
prev_time = time.time()

def tick_callback(channel):
    global tick_count
    tick_count += 1

# Set up interrupt event detection for both encoder pins
GPIO.add_event_detect(ENCODER_PIN_A, GPIO.RISING, callback=tick_callback)
GPIO.add_event_detect(ENCODER_PIN_B, GPIO.RISING, callback=tick_callback)

rpms = []
try:
    while True:
        # Calculate RPM
        current_time = time.time()
        elapsed_time = current_time - prev_time
        if elapsed_time >= 1.0:  # Update RPM every 1 second
            rpm = (tick_count / 360.0) / elapsed_time  # Assuming 360 ticks per revolution
            print(f"RPM: {rpm:.2f}")
            if(rpm > 0):
                rpms.append(rpm)
            tick_count = 0
            prev_time = current_time

        time.sleep(0.1)  # Adjust the sleep time as needed

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
    print(rpms)
