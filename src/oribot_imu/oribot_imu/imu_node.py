import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import adafruit_bno055
from adafruit_bno055 import BNO055
import board

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

class ImuNode(Node):

    def __init__(self):
        super().__init__("imu_node")
        self.imu_publisher = self.create_publisher(Imu,"/imu", 10)
        self.sensor: BNO055 = adafruit_bno055.BNO055_I2C(board.I2C())
        self.create_timer(0.033, self.publish_imu)

    def log(self, txt: str):
        self.get_logger().info(txt)

    def publish_imu(self, ):
        imu: Imu = Imu()
        x,y,z = self.sensor.euler

        imu.header.frame_id = "base_link"
        imu.header.stamp = self.get_clock().now().to_msg()
    
        x,y,z,w = self.sensor.quaternion
        imu.orientation.x = x * RADIANS
        imu.orientation.y = y * RADIANS
        imu.orientation.z = z * RADIANS
        imu.orientation.w = w * RADIANS
        
        x,y,z = self.sensor.linear_acceleration
        imu.linear_acceleration.x = x
        imu.linear_acceleration.y = y
        imu.linear_acceleration.z = z

        if x > 1.0 or y > 1.0:
            self.get_logger().info(f"linear: {x},{y},{z}")

        x,y,z = self.sensor.gyro
        imu.angular_velocity.x = x
        imu.angular_velocity.y = z
        imu.angular_velocity.z = z

        imu.angular_velocity_covariance = ANGULAR_VEL_COV
        imu.linear_acceleration_covariance = LINEAR_ACC_COV
        imu.orientation_covariance = ORIENTATION_COV
    
        self.imu_publisher.publish(imu)

       


    def shutdown(self):
        self.drivetrain.stop()

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node=node)
    node.shutdown()
    rclpy.shutdown()