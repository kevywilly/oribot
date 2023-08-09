import rclpy
from rclpy.node import Node
from Adafruit_MotorHAT import Adafruit_MotorHAT
from oribot.scripts.drivetrain import Drivetrain
from geometry_msgs.msg import Twist

class DriverNode(Node):

    def __init__(self):
        super().__init__("driver_node")
        self.drivetrain = Drivetrain(self.get_logger(), 2, 2)
        self.create_subscription(Twist, "/oribot/cmd_vel", self.handle_cmd_vel, 10)
        #self.twist_timer = self.create_timer(0.1, self.twist_callback)
        
    def log(self, txt: str):
        self.get_logger().info(txt)

    def handle_cmd_vel(self, msg: Twist):
        self.drivetrain.drive_twist(msg)

    def twist_callback(self):
        pass
            

    def shutdown(self):
        self.drivetrain.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    rclpy.spin(node=node)
    node.shutdown()
    rclpy.shutdown()