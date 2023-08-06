#!/usr/bin/env python3
import atexit
import time 
import math
from Adafruit_MotorHAT import Adafruit_MotorHAT
from oribot.scripts.settings import settings
from geometry_msgs.msg import Twist

# https://kyrofa.com/posts/your-first-robot-the-driver-4-5/

# rosrun teleop_twist_joy teleop_node _scale_angular:=4


RADIANS = 0.0174533
#_max_duty = 255
#_wheel_radius_meters = 65.0/2.0/1000  # radius in meters
#_wheel_circumference = _wheel_radius_meters * 2 * math.pi
#_max_rpm = 200
#_max_speed = _max_rpm*_wheel_circumference/60
#_max_angular_velocity = 10


#_wheel_base = 175.0/1000.0
#_turning_circumference = _wheel_base * math.pi
#_max_degrees_per_second = 360*_max_speed / _turning_circumference
#_max_radians_per_second = RADIANS * _max_degrees_per_second

def _clip(value, minimum, maximum):
   """Ensure value is between minimum and maximum."""

   if value < minimum:
       return minimum
   elif value > maximum:
       return maximum
   return value   

class Drivetrain:
    def __init__(self, logger, linear_max_input, angular_max_input):
        driver = Adafruit_MotorHAT(addr=0x60, i2c_bus=settings.i2c_port)

        self.logger = logger
        self._wheel_base = settings.wheel_base_meters
        self._max_linear_velocity = settings.motor_max_rpm*(settings.wheel_radius_meters*2*math.pi)/60 # meters per second
        self._max_angular_velocity = RADIANS * (360 * self._max_linear_velocity)/(settings.wheel_base_meters*math.pi)

        self._linear_factor = self._max_linear_velocity/linear_max_input
        self._angular_factor = self._max_angular_velocity/angular_max_input

        self.m1 = driver.getMotor(1)
        self.m2 = driver.getMotor(2)
        self.m3 = driver.getMotor(3)
        self.m4 = driver.getMotor(4)

        self.motors = [self.m1,self.m2,self.m3,self.m4]
        self.alphas = [1.0,1.0,1.0,1.0]

        self.stop()

        self.info()
        atexit.register(self.stop)

    def info(self):
        self.logger.info(f"Max Angular Velocity: {self._max_angular_velocity}")
        self.logger.info(f"Max Linear Velocity (meters / sec): {self._max_linear_velocity}")
        self.logger.info(f"_scale_linear:={self._linear_factor} _scale_angular:={self._angular_factor}")

    def drive_twist(self, twist: Twist):

        
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z

        self.logger.info(f"GOT linear: {x},{y} angular: {z}")
        
        if abs(y) > abs(x):
            linear = y
            sideways = True
        else:
            linear = x
            sideways = False
        
        if sideways:
            lv = linear*self._linear_factor
            rv = linear*self._linear_factor
        else:
            lv = linear*self._linear_factor-z*self._angular_factor*self._wheel_base/2
            rv = linear*self._linear_factor+z*self._angular_factor*self._wheel_base/2

        left_speed_pct = int(255*lv/self._max_linear_velocity)
        right_speed_pct = int(255*rv/self._max_linear_velocity)

        for i in range(4):
            if sideways:
                s = left_speed_pct if i % 2 else right_speed_pct
                v = s if i == 0 or i == 3 else -s
            else:
                v = left_speed_pct if i % 2 else right_speed_pct
            self.logger.info("writing to motor")
            self._set_speed(motor_id=i, speed_pct=v)

    def stop(self):
        self.drive_twist(Twist())
        for motor in self.motors:
            motor.run(Adafruit_MotorHAT.RELEASE)
        

    def _set_speed(self, motor_id: int, speed_pct: int):
        direction = Adafruit_MotorHAT.FORWARD if speed_pct >=0 else Adafruit_MotorHAT.BACKWARD
        speed = _clip(abs(speed_pct), 0, 255)
        self.logger.info(f"motor: {motor_id} - speed_pct: {speed_pct}, speed: {speed}, dir: {direction}")
        if not settings.debug:
            self.motors[motor_id].setSpeed(speed)
            self.motors[motor_id].run(direction)
        else:
            self.motors[motor_id].setSpeed(0)
            self.motors[motor_id].run(direction)

            