import rclpy
from rclpy.node import Node
from oribot_driver.scripts.drivetrain import Drivetrain
from geometry_msgs.msg import TransformStamped, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry


class DriverNode(Node):

    def __init__(self):
        super().__init__("driver_node")
        self.drivetrain: Drivetrain = Drivetrain(self.get_logger(), 1, 1)
        self.create_subscription(Twist, "/oribot/cmd_vel", self.handle_cmd_vel, 10)
        self.odometry_publisher = self.create_publisher(Odometry,"/odom", 10)
        self.create_timer(.033, self.publish_odometry)
        

    def log(self, txt: str):
        self.get_logger().info(txt)

    def handle_cmd_vel(self, msg: Twist):
        
        self.twist = msg
        self.drivetrain.drive_twist(msg)
        #self.drivetrain.drive(msg.linear.x, msg.angular.z)

    def publish_odometry(self):
        msg: Odometry = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
       
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        #msg.pose.pose.orientation.x = 0.0
        #msg.pose.pose.orientation.y = 0.0
        #msg.pose.pose.orientation.z = 0.0

        msg.twist.twist.angular.z = 0.0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0     
        
    
        self.odometry_publisher.publish(msg)

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