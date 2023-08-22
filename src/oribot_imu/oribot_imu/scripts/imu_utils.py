#!/usr/bin/python3

import adafruit_bno055
from adafruit_bno055 import BNO055
import board
import numpy as np
import time

RADIANS = 0.0174533

def calculate_covariances():
    
    sensor: BNO055 = adafruit_bno055.BNO055_I2C(board.I2C())

    # Number of samples to collect
    num_samples = 100

    # Lists to store sensor data
    accel_data = []
    gyro_data = []
    orient_data = []

    identity = np.array([1,0,0,1,0,0,1,0,0])

    # Collect sensor data
    for _ in range(num_samples):
        accel_data.append(sensor.linear_acceleration)
        gyro_data.append(sensor.gyro)
        orient_data.append(sensor.euler)
        time.sleep(0.1)  # Delay between samples

    # Convert data lists to numpy arrays for analysis
    accel_data = np.array(accel_data)
    gyro_data = np.array(gyro_data)
    orient_data = np.array(orient_data)*RADIANS

    # Calculate covariances
    accel_cov = np.cov(accel_data, rowvar=False).flatten()
    gyro_cov = np.cov(gyro_data, rowvar=False).flatten()
    orient_cov = np.cov(orient_data, rowvar=False).flatten()

    accel_cov = accel_cov*identity
    gyro_cov = gyro_cov*identity
    orient_cov = orient_cov*identity

    # Print covariance matrices
    print("Accelerometer Covariance Matrix:")
    print(accel_cov)

    print("Gyroscope Covariance Matrix:")
    print(gyro_cov)

    print("Orientation Covariance Matrix:")
    print(orient_cov)
        

if __name__ == "__main__":
    calculate_covariances()