import rclpy
from rclpy.node import Node
from oribot_driver.scripts.drivetrain import Drivetrain
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


class DriverNode(Node):

    def __init__(self):
        super().__init__("driver_node")
        self.drivetrain: Drivetrain = Drivetrain(self.get_logger(), 1, 1)
        self.create_subscription(Twist, "/oribot/cmd_vel", self.handle_cmd_vel, 10)
        self.odometry_publisher = self.create_publisher(Odometry,"oribot/odom", 10)
        self.transform_publisher = self.create_publisher(TFMessage, "/tf", 10)
        self.create_timer(.033, self.publish_odometry)
        self.x = 0.0

    def log(self, txt: str):
        self.get_logger().info(txt)

    def handle_cmd_vel(self, msg: Twist):
        
        self.twist = msg
        self.drivetrain.drive_twist(msg)
        #self.drivetrain.drive(msg.linear.x, msg.angular.z)

    def publish_odometry(self):
        stamp = self.get_clock().now().to_msg()
        msg: Odometry = Odometry()
        msg.header.frame_id = "base_link"
        msg.child_frame_id = "odom"
        msg.header.stamp = stamp
       
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        #msg.pose.pose.orientation.x = 0.0
        #msg.pose.pose.orientation.y = 0.0
        #msg.pose.pose.orientation.z = 0.0

        msg.twist.twist.angular.z = 0.0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0     
        
        # self.x = self.x + 1/1000.0
    
        self.odometry_publisher.publish(msg)

        m = TFMessage()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation.z = msg.pose.pose.orientation.z
        m.transforms = [tf]
        self.transform_publisher.publish(m)

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