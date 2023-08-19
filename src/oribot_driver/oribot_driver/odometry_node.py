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

''' 
board_to_tegra = {
k: list(GPIO.gpio_pin_data.get_data()[-1]['TEGRA_SOC'].keys())[i] for i, k in enumerate(GPIO.gpio_pin_data.get_data()[-1]['BOARD'])}

for k, v in board_to_tegra.items():
    print('bcm #:', k, 'tegra:', v)
'''

# Constants
TICKS_PER_REVOLUTION = 360  # Number of ticks per full revolution
UPDATE_INTERVAL = 1.0  # Interval in seconds to update RPM
WHEEL_SEPARATION = (134+18)/1000.0
ODOMETRY_UPDATE_FREQUENCY = 30 # HZ

RADIANS = 0.0174533

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

class TicksCounter:

    def __init__(self, enca, encb, radius: float):
        #self.imu: Imu = Imu()
        self.enca = enca
        self.encb = encb
        self.last_enca_state = 0
        self.ticks = 0
        self.start_ticks = 0
        self.rpm = 0
        self.previous_rpm = 0
        self.average_rpm = 0
        self.velocity = 0
        self.previous_velocity = 0
        self.average_velocity = 0
        self.elapsed_time = 0
        self.radius = radius
        self.circumference = 2 * 3.1415 * self.radius

        GPIO.setup(self.enca, GPIO.IN)
        GPIO.setup(self.encb, GPIO.IN)

        GPIO.add_event_detect(self.enca, GPIO.BOTH, callback = self.enca_callback)

    def set_rpm(self, rpm):
        self.previous_rpm = self.rpm
        self.rpm = rpm
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
        self.tickers = [TicksCounter('GP39_SPI3_CS0_N', 'GP40_SPI3_CS1_N', 65.5/1000/2.0)]
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.orientation_z = 0.0

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.start_time = time.time()

        # self.loop()

    def calc_pose(self, elapsed_time):
        
        velocity_x = self.tickers[0].average_velocity
        self.pose_x += (velocity_x * elapsed_time)
        self.linear_x = velocity_x
    

    def publish_odometry(self, elapsed_time):
        self.calc_pose(elapsed_time)
        # self.get_logger().info(f"pos_x: {self.pose_x}")

        stamp = self.get_clock().now().to_msg()

        transforms = TFMessage()
        tf = TransformStamped()

        odom: Odometry = Odometry()
        odom.header.frame_id = "base_link"
        odom.child_frame_id = "odom"
        odom.header.stamp = stamp
       
        
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.linear.y = self.linear_y

        a,b,c = self.imu.gyro
        odom.twist.twist.angular.z = c

        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        
        x,y,z,w = self.imu.quaternion
        odom.pose.pose.orientation.x = x * RADIANS
        odom.pose.pose.orientation.y = y * RADIANS
        odom.pose.pose.orientation.z = z * RADIANS
        odom.pose.pose.orientation.w = w * RADIANS

        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.pose_x
        tf.transform.translation.y = self.pose_y
        tf.transform.rotation.z = z * RADIANS
        tf.transform.rotation.w = w * RADIANS
        
        


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
        #    self.get_logger().info(f"RPM-{index}: {ticker.rpm:.2f}, vel: {ticker.velocity:.3f}")
        
    def cleanup(self):
        for ticker in self.tickers:
            ticker.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node=node)
    node.cleanup()
    rclpy.shutdown()