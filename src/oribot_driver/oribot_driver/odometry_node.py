import Jetson.GPIO as GPIO
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import adafruit_bno055
from adafruit_bno055 import BNO055
import board
import math
import numpy as np

''' 
board_to_tegra = {
k: list(GPIO.gpio_pin_data.get_data()[-1]['TEGRA_SOC'].keys())[i] for i, k in enumerate(GPIO.gpio_pin_data.get_data()[-1]['BOARD'])}

for k, v in board_to_tegra.items():
    print('board #:', k, 'tegra:', v)
'''

# Constants
TICKS_PER_REVOLUTION = 360  # Number of ticks per full revolution
UPDATE_INTERVAL = 1.0  # Interval in seconds to update RPM
WHEEL_SEPARATION_X = 171.0/1000.0
WHEEL_SEPARATION_Y = 185.0/1000.0
WHEEL_RADIUS = 65.0/2/1000.0
ODOMETRY_UPDATE_FREQUENCY = 30 # HZ
# RADIUS FROM CENTER OF BASE TO CIRCLE THAT ENCLOSES WHEEL BASE
ROBOT_RADIUS = 106/1000.0 # meters
ROTATION_CIRCUMFERENCE = 2*math.pi*ROBOT_RADIUS

RADIANS = math.pi/180.0
MAX_RADIANS = 2*math.pi

LINEAR_ACC_COV = [
    0.00117272, 
    0.0,
    0.0,         
    0.00192647, 
    0.0,         
    0.0,
    0.06533882,
    0.0,
    0.0
]

ANGULAR_VEL_COV = [
    3.03499624e-06,
    0.0,
    0.0,
    4.06372907e-07,
    0.0,
    -0.0,
    2.37621780e-07,
    -0.0,
    0.0
  ]

ORIENTATION_COV = [
    0.17209482,
    0.0,
    0.0,
    -0.00131232,
    0.0,
    -0.0,
    0.12788324,
    -0.0,
    0.0
  ]


GPIO.setmode(GPIO.TEGRA_SOC)



wheel_matrix = np.array([
        [1, 1, -(ROBOT_RADIUS)],
        [-1, 1, -(ROBOT_RADIUS)],
        [-1, -1, -(ROBOT_RADIUS)],
        [1, -1, -(ROBOT_RADIUS)]
    ])

def quarternion(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


def calculate_mecanum_velocities(wheel_velocities, wheel_radius, robot_width, robot_length):
    # Assuming wheel_velocities is a list containing the velocities of all 4 wheels in m/s
    # wheel_radius: Radius of the wheels in meters
    # robot_width: Width of the robot in meters (distance between left and right wheels)
    # robot_length: Length of the robot in meters (distance between front and rear wheels)

    # Calculate linear velocity components in x and y directions
    vx = (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3]) / 4
    vy = (-wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3]) / 4

    # Calculate linear velocity magnitude
    linear_velocity = math.sqrt(vx**2 + vy**2)

    # Calculate angular velocity
    angular_velocity = (wheel_velocities[0] - wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3]) \
                       * wheel_radius / (2 * (robot_width + robot_length))

    return vx, vy, -angular_velocity

def calc_pose(pose_x, pose_y, orientation_z, wheel_velocities, elapsed_time):

        #self.get_logger().info(f"velocities: {wheel_velocities}")

        linear_x, linear_y, angular_z = calculate_mecanum_velocities(wheel_velocities, WHEEL_RADIUS, WHEEL_SEPARATION_X,WHEEL_SEPARATION_Y )

        dx = linear_x * elapsed_time
        dy = linear_y * elapsed_time
        dtheta = angular_z * elapsed_time
        
        cumulative_theta = (orientation_z + dtheta) % MAX_RADIANS

        pose_x = pose_x + math.cos(cumulative_theta)*dx + math.sin(cumulative_theta)*dy
        pose_y = pose_y + math.cos(cumulative_theta)*dy + math.sin(cumulative_theta)*dx
        
        return pose_x, pose_y, cumulative_theta, linear_x, linear_y, angular_z

class TicksCounter:

    def __init__(self, enca, encb, radius: float, orientation: int = 1):
        #self.imu: Imu = Imu()
        self.enca = enca
        self.encb = encb
        self.last_enca_state = 0
        self.ticks = 0
        self.start_ticks = 0
        self.rpm = 0.0
        self.previous_rpm = 0.0
        self.average_rpm = 0.0
        self.velocity = 0.0
        self.previous_velocity = 0.0
        self.average_velocity = 0.0
        self.elapsed_time = 0.0
        self.radius = radius
        self.circumference = 2 * 3.1415 * self.radius
        self.orientation = orientation
        self.theta = 0.0

        GPIO.setup(self.enca, GPIO.IN)
        GPIO.setup(self.encb, GPIO.IN)

        GPIO.add_event_detect(self.enca, GPIO.BOTH, callback = self.enca_callback)

    def set_rpm(self, rpm):
        self.previous_rpm = self.rpm
        self.rpm = rpm * self.orientation
        self.average_rpm = (self.rpm + self.previous_rpm) / 2.0
        self.previous_velocity = self.velocity
        self.velocity = self.rpm * self.circumference / 60.0
        self.average_velocity = (self.velocity + self.previous_velocity) / 2.0
        
    def enca_callback(self, channel):
        encb_state = GPIO.input(self.encb)

        if self.last_enca_state != encb_state:
            self.ticks += 1
        else:
            self.ticks -= 1

        self.last_enca_state = encb_state

    def cleanup(self):
        GPIO.cleanup(self.enca)
        GPIO.cleanup(self.encb)

class OdometryNode(Node):

    def __init__(self):
        super().__init__("odometry_node")
        self.imu: BNO055 = adafruit_bno055.BNO055_I2C(board.I2C())
        self.odometry_publisher = self.create_publisher(Odometry,"oribot/odom", 10)
        self.transform_publisher = self.create_publisher(TFMessage, "/tf", 10)
        self.create_timer(0.33, self.loop)
        #self.create_timer(1, self.loop)

        # 18-green, 16-blue 
        # 24-black, 22-white
        # 15-orange, 13-yellow
        # 11-brown, 7-red 
        self.tickers = [ 
            TicksCounter(enca='GP39_SPI3_CS0_N', encb='GP40_SPI3_CS1_N', radius=WHEEL_RADIUS, orientation=-1), 
            TicksCounter(enca='GP50_SPI1_CS0_N', encb='GP37_SPI3_MISO', radius=WHEEL_RADIUS, orientation=1), 
            TicksCounter(enca='GP88_PWM1', encb='GP36_SPI3_CLK', radius=WHEEL_RADIUS, orientation=-1), 
            TicksCounter(enca='GP72_UART1_RTS_N', encb='GP167', radius=WHEEL_RADIUS, orientation=1)
            ]
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.orientation_z = 0.0
        
        self.orientation_z_imu = self.imu.euler[2]

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.start_time = time.time()

        # self.loop()
    
    def publish_odometry(self, elapsed_time):

        self.pose_x, self.pose_y, self.orientation_z, self.linear_x, self.linary_y, self.linear_z = calc_pose(
            self.pose_x,
            self.pose_y,
            self.orientation_z, 
            [w.average_velocity for w in self.tickers], 
            elapsed_time)
        
        self.get_logger().info(f"pose: {self.pose_x}, {self.pose_y}, {self.orientation_z}")

        stamp = self.get_clock().now().to_msg()

        transforms = TFMessage()
        tf = TransformStamped()

        odom: Odometry = Odometry()
        odom.header.frame_id = "base_link"
        odom.child_frame_id = "odom"
        odom.header.stamp = stamp
        
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.linear.y = self.linear_y
        odom.twist.twist.angular.z = self.angular_z
 
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y

        _,_,z,w = quarternion(0.0, 0.0, self.orientation_z)
        odom.pose.pose.orientation.w = w
        odom.pose.pose.orientation.z = z
        

        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.pose_x
        tf.transform.translation.y = self.pose_y
        tf.transform.rotation.z = z
        tf.transform.rotation.w = w

        transforms.transforms = [tf]

        self.odometry_publisher.publish(odom)
        self.transform_publisher.publish(transforms)

    def loop(self):
    
        
        for ticker in self.tickers:
            ticker.start_ticks = ticker.ticks

        time.sleep(UPDATE_INTERVAL)
    
        end_time = time.time()
        elapsed_time = end_time - self.start_time

        for ticker in self.tickers:
            tick_difference = ticker.ticks - ticker.start_ticks
            ticker.set_rpm((tick_difference / TICKS_PER_REVOLUTION) / (elapsed_time / 60.0))

        self.start_time = time.time()

        self.publish_odometry(elapsed_time)

        #for index, ticker in enumerate(self.tickers):
        #    self.get_logger().info(f"RPM-{index+2} {ticker.enca}: {ticker.average_rpm:.2f}, vel: {ticker.average_velocity:.3f}")
        
    def cleanup(self):
        for ticker in self.tickers:
            ticker.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node=node)
    node.cleanup()
    rclpy.shutdown()