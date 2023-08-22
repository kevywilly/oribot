import rclpy
from rclpy.node import Node
from oribot_driver.scripts.drivetrain import Drivetrain
from geometry_msgs.msg import TransformStamped, Twist


class DriverNode(Node):

    def __init__(self):
        super().__init__("driver_node")
        self.drivetrain: Drivetrain = Drivetrain(self.get_logger(), 1, 1)
        self.create_subscription(Twist, "/oribot/cmd_vel", self.handle_cmd_vel, 10)
        
        self.x = 0.0

    def log(self, txt: str):
        self.get_logger().info(txt)

    def handle_cmd_vel(self, msg: Twist):
        
        self.twist = msg
        self.drivetrain.drive_twist(msg)
        #self.drivetrain.drive(msg.linear.x, msg.angular.z)

    def publish_transform(self):
        msg = TransformStamped
        msg.child_frame_id = "base_laser"
        msg.translation.x = 0.0
        msg.translation.y = 0.0
        msg.translation.z = 0.0
        self.transform_publisher.publish(msg)
    

    def shutdown(self):
        self.drivetrain.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    rclpy.spin(node=node)
    node.shutdown()
    rclpy.shutdown()