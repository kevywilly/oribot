import math
import numpy as np

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

    return vx, vy, angular_velocity

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


velocities = [
    [1.0, 1.0, 1.0, 1.0],
    [1.0, -1.0, 1.0, -1.0],
    [1,1,1,1],
    [0,0,0,0],
]

thetas = [0,0,0,0]

pose_x = 0.0
pose_y = 0.0
orientation_z = 0.0

duration = 2.0

for i in range(4):

    pose_x, pose_y, orientation_z, linear_x, linear_y, angular_z = calc_pose(
         pose_x, pose_y, orientation_z, velocities[i], duration
    )
    x,y, z, w = quarternion(0,0,orientation_z)

    print(f"({pose_x:.1f}, {pose_y:.1f}, {orientation_z:.1f}, {z:.1f}, {w:.1f}) : ({linear_x:.1f}, {linear_y:.1f}, {angular_z:.1f})")
    