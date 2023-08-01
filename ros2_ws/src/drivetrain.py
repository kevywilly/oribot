#!/usr/bin/env python3
import atexit
import time 
import math
from Adafruit_MotorHAT import Adafruit_MotorHAT


# https://kyrofa.com/posts/your-first-robot-the-driver-4-5/

# rosrun teleop_twist_joy teleop_node _scale_angular:=4

I2C = 7
_rads = 0.0174533
_max_duty = 255
_wheel_radius_meters = 65.0/2.0/1000  # radius in meters
_wheel_circumference = _wheel_radius_meters * 2 * math.pi
_max_rpm = 200
_max_speed = _max_rpm*_wheel_circumference/60
_max_angular_velocity = 10


_wheel_base = 175.0/1000.0
_turning_circumference = _wheel_base * math.pi
_max_degrees_per_second = 360*_max_speed / _turning_circumference
_max_radians_per_second = _rads * _max_degrees_per_second

_linear_factor = _max_speed/0.5
_angular_factor = _max_radians_per_second/0.5

print(f"Max Degrees Per Second: {_max_degrees_per_second}")
print(f"Max Radians Per Second: {_max_radians_per_second}")
print(f"Max Linear Velocity (meters / sec): {_max_speed}")
print(f"Max Angular Velocity (radians / sec): {_max_radians_per_second}")

print(f"rosrun teleop_twist_joy teleop_node _scale_linear:={_linear_factor} _scale_angular:={_angular_factor}")

def _clip(value, minimum, maximum):
   """Ensure value is between minimum and maximum."""

   if value < minimum:
       return minimum
   elif value > maximum:
       return maximum
   return value
   

class Drivetrain:
    def __init__(self):
        driver = Adafruit_MotorHAT(addr=0x60, i2c_bus=I2C)

        self.m1 = driver.getMotor(1)
        self.m2 = driver.getMotor(2)
        self.m3 = driver.getMotor(3)
        self.m4 = driver.getMotor(4)

        self.motors = [self.m1,self.m2,self.m3,self.m4]
        self.alphas = [1.0,1.0,1.0,1.0]

        self.stop()
        atexit.register(self.stop)

    def drive(self, linear: float = 0.5, angular: float = 0):

        lv = linear*_linear_factor-angular*_angular_factor*_wheel_base/2
        rv = linear*_linear_factor+angular*_angular_factor*_wheel_base/2
        left_speed_pct = int(_max_duty*lv/_max_speed)
        right_speed_pct = int(_max_duty*rv/_max_speed)

        for i in range(4):
            v = left_speed_pct if i % 2 else right_speed_pct
            self._set_speed(motor_id=i, speed_pct=v)

    def stop(self):
        self.drive(0,0)
        for motor in self.motors:
            motor.run(Adafruit_MotorHAT.RELEASE)
        

    def _set_speed(self, motor_id: int, speed_pct: int):
        direction = Adafruit_MotorHAT.FORWARD if speed_pct >=0 else Adafruit_MotorHAT.BACKWARD
        speed = _clip(abs(speed_pct), 0, 255)
        print(f"{motor_id}: {speed_pct}, {speed}, {direction}")
        self.motors[motor_id].setSpeed(speed)
        self.motors[motor_id].run(direction)
            


from drivetrain import Drivetrain
dt = Drivetrain()

dt.drive(.18,.1)
time.sleep(1)

dt.drive(.18,-.1)
time.sleep(1)

dt.drive(-.18,.1)
time.sleep(1)

dt.drive(-.18,-.1)
time.sleep(1)

'''

dt.drive(0.2,0)
time.sleep(3)

dt.drive(-0.2,0)
time.sleep(3)

dt.drive(0,0.2)
time.sleep(3)

dt.drive(0,-.2)
time.sleep(3)

dt.drive(.2,-.2)
time.sleep(3)
'''

dt.stop()

exit()