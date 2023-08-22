#!/usr/bin/python3

import Jetson.GPIO as GPIO
import time

# Constants
TICKS_PER_REVOLUTION = 360  # Number of ticks per full revolution
UPDATE_INTERVAL = 1.0  # Interval in seconds to update RPM

# Pins for encoder A and B outputs
ENCODER_PIN_A = 18
ENCODER_PIN_B = 16

# Global variables to track encoder state and ticks
encoder_ticks = 0
last_encoder_a_state = 0

# Callback function for encoder A
def encoder_a_callback(channel):
    global encoder_ticks, last_encoder_a_state
    encoder_b_state = GPIO.input(ENCODER_PIN_B)
    
    if last_encoder_a_state != encoder_b_state:
        encoder_ticks += 1
    else:
        encoder_ticks -= 1
    
    last_encoder_a_state = encoder_b_state

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ENCODER_PIN_A, GPIO.IN)
GPIO.setup(ENCODER_PIN_B, GPIO.IN)

# Setup interrupt event detection for encoder A
GPIO.add_event_detect(ENCODER_PIN_A, GPIO.BOTH, callback=encoder_a_callback)

# Main loop
try:
    while True:
        start_time = time.time()
        start_ticks = encoder_ticks
        time.sleep(UPDATE_INTERVAL)
        
        end_time = time.time()
        elapsed_time = end_time - start_time
        tick_difference = encoder_ticks - start_ticks
        
        rpm = (tick_difference / TICKS_PER_REVOLUTION) / (elapsed_time / 60.0)
        print(f"RPM: {rpm:.2f}")
        
except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
